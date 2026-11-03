#include "bme_driver.h"
#include "board_pins.h"
#include "spi_interface.h"
#include <stddef.h>
#include <string.h>

/* ---- BME280 register map --------------------------------------------- */
#define BME_REG_CALIB00     0x88   /* 26 bytes: dig_T1..T3, dig_P1..P9, dig_H1 */
#define BME_REG_ID          0xD0
#define BME_REG_RESET       0xE0
#define BME_REG_CALIB26     0xE1   /* 7 bytes: dig_H2..H6 */
#define BME_REG_CTRL_HUM    0xF2
#define BME_REG_STATUS      0xF3
#define BME_REG_CTRL_MEAS   0xF4
#define BME_REG_CONFIG      0xF5
#define BME_REG_PRESS_MSB   0xF7   /* 8 bytes: press[3], temp[3], hum[2] */

#define BME_READ_BIT        0x80   /* SPI: read sets bit 7, write clears it */
#define BME_CHIP_ID         0x60
#define BME_RESET_WORD      0xB6

/* ctrl_hum  = 0x01 : osrs_h x1
   ctrl_meas = 0x57 : osrs_t x2 (010), osrs_p x16 (101), mode normal (11)
   config    = 0x10 : t_sb 0.5 ms (000), IIR filter 16 (100), spi3w off
   Pressure gets the heavy oversampling because it is the measurement the
   vessel-integrity check depends on. */
#define BME_CTRL_HUM_VAL    0x01
#define BME_CTRL_MEAS_VAL   0x57
#define BME_CONFIG_VAL      0x10

#define BME_SPI_CLOCK_HZ    8000000   /* BME280 max is 10 MHz */
#define BME_SPI_MODE        0         /* mode 0 and 3 both work; 0 chosen */

/* ---- calibration, exactly as packed in the device -------------------- */
typedef struct {
    uint16_t dig_T1; int16_t dig_T2, dig_T3;
    uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5,
                             dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t  dig_H1; int16_t dig_H2; uint8_t dig_H3;
    int16_t  dig_H4, dig_H5; int8_t dig_H6;
} bme_calib_t;

typedef struct {
    spi_device_handle_t dev;
    bme_calib_t         cal;
    int32_t             t_fine;
    bool                present;
} bme_ctx_t;

static bme_ctx_t ctx[BME_COUNT];

static const gpio_num_t CS_PIN[BME_COUNT] = {
    [BME_INTERNAL] = PIN_CS_BME280_IN,
    [BME_EXTERNAL] = PIN_CS_BME280_OUT,
};

/* ---- bus helpers ------------------------------------------------------ */

static esp_err_t reg_write(bme_ctx_t *c, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), value };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    return spi_device_polling_transmit(c->dev, &t);
}

static esp_err_t reg_read(bme_ctx_t *c, uint8_t reg, uint8_t *buf, size_t n)
{
    uint8_t tx[1 + 26] = { (uint8_t)(reg | BME_READ_BIT) };
    uint8_t rx[1 + 26] = { 0 };
    if (n > 26) return ESP_ERR_INVALID_ARG;

    spi_transaction_t t = {
        .length    = 8u * (n + 1u),
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_polling_transmit(c->dev, &t);
    if (err != ESP_OK) return err;

    memcpy(buf, &rx[1], n);
    return ESP_OK;
}

/* ---- calibration unpacking ------------------------------------------- *
 * This is the part that is easy to get wrong and NOT the part that needs
 * the datasheet's arithmetic: it is a register map, and the signedness of
 * each field is what matters. dig_H4 and dig_H5 share one byte (0xE5) and
 * are 12-bit, split differently from each other. Getting that wrong gives
 * a humidity reading that looks plausible and is not.
 */
static esp_err_t read_calibration(bme_ctx_t *c)
{
    uint8_t a[26], b[7];
    esp_err_t err = reg_read(c, BME_REG_CALIB00, a, sizeof a);
    if (err != ESP_OK) return err;
    err = reg_read(c, BME_REG_CALIB26, b, sizeof b);
    if (err != ESP_OK) return err;

    #define U16(lo, hi) ((uint16_t)((uint16_t)(hi) << 8 | (lo)))
    #define S16(lo, hi) ((int16_t)U16((lo), (hi)))

    c->cal.dig_T1 = U16(a[0],  a[1]);
    c->cal.dig_T2 = S16(a[2],  a[3]);
    c->cal.dig_T3 = S16(a[4],  a[5]);
    c->cal.dig_P1 = U16(a[6],  a[7]);
    c->cal.dig_P2 = S16(a[8],  a[9]);
    c->cal.dig_P3 = S16(a[10], a[11]);
    c->cal.dig_P4 = S16(a[12], a[13]);
    c->cal.dig_P5 = S16(a[14], a[15]);
    c->cal.dig_P6 = S16(a[16], a[17]);
    c->cal.dig_P7 = S16(a[18], a[19]);
    c->cal.dig_P8 = S16(a[20], a[21]);
    c->cal.dig_P9 = S16(a[22], a[23]);
    /* a[24] is 0xA0, reserved. */
    c->cal.dig_H1 = a[25];                               /* 0xA1, unsigned */

    c->cal.dig_H2 = S16(b[0], b[1]);                     /* 0xE1,0xE2      */
    c->cal.dig_H3 = b[2];                                /* 0xE3, unsigned */
    c->cal.dig_H4 = (int16_t)(((int16_t)(int8_t)b[3] << 4)
                              | (b[4] & 0x0Fu));         /* 0xE4, 0xE5[3:0] */
    c->cal.dig_H5 = (int16_t)(((int16_t)(int8_t)b[5] << 4)
                              | (b[4] >> 4));            /* 0xE6, 0xE5[7:4] */
    c->cal.dig_H6 = (int8_t)b[6];                        /* 0xE7, SIGNED   */

    #undef U16
    #undef S16
    return ESP_OK;
}

/* ====================================================================== *
 *  COMPENSATION — NOT YET WRITTEN. DO NOT GUESS THESE.
 *
 *  Copy the three reference functions verbatim from the BME280 datasheet
 *  (Bosch BST-BME280-DS002, section 4.2.3, "Compensation formulas",
 *  32-bit integer versions):
 *
 *      BME280_S32_t  BME280_compensate_T_int32 (BME280_S32_t adc_T);
 *      BME280_U32_t  BME280_compensate_P_int64 (BME280_S32_t adc_P);
 *      BME280_U32_t  bme280_compensate_H_int32 (BME280_S32_t adc_H);
 *
 *  Paste them below with these three changes and nothing else:
 *    1. take `bme_ctx_t *c` as a first parameter;
 *    2. replace every `dig_XX` with `c->cal.dig_XX`;
 *    3. replace the file-scope `t_fine` with `c->t_fine`.
 *
 *  Returned units, which the wrappers below already assume:
 *      _T_int32  ->  0.01 C          (5123 == 51.23 C)
 *      _P_int64  ->  Q24.8 Pa        (divide by 256 for Pa)
 *      _H_int32  ->  Q22.10 %RH      (divide by 1024 for %RH)
 *
 *  Call order is not optional: _T_int32 sets c->t_fine, and both of the
 *  others read it. Compensating pressure before temperature uses last
 *  cycle's t_fine, or zero on the first read.
 *
 *  I have deliberately not typed these from memory. They are ~40 magic
 *  constants and shift counts; a single transposed digit produces a
 *  reading that looks right and is wrong by a few percent, which is
 *  exactly the failure the vessel-integrity check would not catch.
 * ====================================================================== */
static int32_t  compensate_T(bme_ctx_t *c, int32_t adc_T);   /* TODO: paste */
static uint32_t compensate_P(bme_ctx_t *c, int32_t adc_P);   /* TODO: paste */
static uint32_t compensate_H(bme_ctx_t *c, int32_t adc_H);   /* TODO: paste */

/* ---- public API ------------------------------------------------------- */

static esp_err_t init_one(bme_id_t id)
{
    bme_ctx_t *c = &ctx[id];
    c->present = false;

    esp_err_t err = spi_interface_add_device(CS_PIN[id], BME_SPI_CLOCK_HZ,
                                             BME_SPI_MODE, &c->dev);
    if (err != ESP_OK) return err;

    uint8_t chip_id = 0;
    err = reg_read(c, BME_REG_ID, &chip_id, 1);
    if (err != ESP_OK)            return err;
    if (chip_id != BME_CHIP_ID)   return ESP_ERR_NOT_FOUND;

    err = read_calibration(c);
    if (err != ESP_OK) return err;

    /* ctrl_hum must be written BEFORE ctrl_meas: a change to ctrl_hum
       only takes effect when ctrl_meas is written afterwards. Written in
       the other order, humidity silently stays disabled. */
    err = reg_write(c, BME_REG_CTRL_HUM, BME_CTRL_HUM_VAL);
    if (err != ESP_OK) return err;
    err = reg_write(c, BME_REG_CONFIG,   BME_CONFIG_VAL);
    if (err != ESP_OK) return err;
    err = reg_write(c, BME_REG_CTRL_MEAS, BME_CTRL_MEAS_VAL);
    if (err != ESP_OK) return err;

    c->present = true;
    return ESP_OK;
}

esp_err_t bme_driver_init(void)
{
    esp_err_t first_err = ESP_OK;
    for (bme_id_t id = 0; id < BME_COUNT; id++) {
        esp_err_t err = init_one(id);
        if (err != ESP_OK && first_err == ESP_OK) first_err = err;
    }
    return first_err;
}

bool bme_driver_present(bme_id_t id)
{
    if ((unsigned)id >= BME_COUNT) return false;
    return ctx[id].present;
}

esp_err_t bme_driver_read(bme_id_t id, bme_sample_t *out)
{
    if ((unsigned)id >= BME_COUNT || out == NULL) return ESP_ERR_INVALID_ARG;

    bme_ctx_t *c = &ctx[id];
    if (!c->present) return ESP_ERR_INVALID_STATE;

    uint8_t b[8];
    esp_err_t err = reg_read(c, BME_REG_PRESS_MSB, b, sizeof b);
    if (err != ESP_OK) return err;

    /* 20-bit pressure and temperature, 16-bit humidity, all big-endian
       and left-aligned in their bytes. */
    const int32_t adc_P = (int32_t)(((uint32_t)b[0] << 12)
                                  | ((uint32_t)b[1] << 4)
                                  | ((uint32_t)b[2] >> 4));
    const int32_t adc_T = (int32_t)(((uint32_t)b[3] << 12)
                                  | ((uint32_t)b[4] << 4)
                                  | ((uint32_t)b[5] >> 4));
    const int32_t adc_H = (int32_t)(((uint32_t)b[6] << 8) | b[7]);

    /* 0x80000 / 0x8000 are the reset values: the measurement is disabled
       or has not completed. Compensating them yields a confident wrong
       answer, so reject them here. */
    if (adc_T == 0x80000 || adc_P == 0x80000) return ESP_ERR_INVALID_RESPONSE;

    out->temperature_centi_c = compensate_T(c, adc_T);   /* must be first */
    out->pressure_pa         = compensate_P(c, adc_P) / 256u;
    out->humidity_centi_pct  = (adc_H == 0x8000)
                             ? 0u
                             : (compensate_H(c, adc_H) * 100u) / 1024u;
    return ESP_OK;
}