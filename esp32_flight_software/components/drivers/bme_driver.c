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
#define BME_SPI_MODE        0         /* modes 0 and 3 both work; 0 chosen */

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

    memcpy(buf, &rx[1], n);   /* byte 0 is clocked out during the address */
    return ESP_OK;
}

/* ---- calibration unpacking ------------------------------------------- *
 * The signedness of each field is what matters here, and dig_H4 / dig_H5
 * share one byte (0xE5) as 12-bit values split differently from each
 * other. Getting that wrong gives a humidity reading that looks plausible
 * and is not.
 */
static esp_err_t read_calibration(bme_ctx_t *c)
{
    uint8_t a[26], b[7];
    esp_err_t err = reg_read(c, BME_REG_CALIB00, a, sizeof a);
    if (err != ESP_OK) return err;
    err = reg_read(c, BME_REG_CALIB26, b, sizeof b);
    if (err != ESP_OK) return err;

    #define BME_U16(lo, hi) ((uint16_t)((uint16_t)(hi) << 8 | (uint16_t)(lo)))
    #define BME_S16(lo, hi) ((int16_t)BME_U16((lo), (hi)))

    c->cal.dig_T1 = BME_U16(a[0],  a[1]);
    c->cal.dig_T2 = BME_S16(a[2],  a[3]);
    c->cal.dig_T3 = BME_S16(a[4],  a[5]);
    c->cal.dig_P1 = BME_U16(a[6],  a[7]);
    c->cal.dig_P2 = BME_S16(a[8],  a[9]);
    c->cal.dig_P3 = BME_S16(a[10], a[11]);
    c->cal.dig_P4 = BME_S16(a[12], a[13]);
    c->cal.dig_P5 = BME_S16(a[14], a[15]);
    c->cal.dig_P6 = BME_S16(a[16], a[17]);
    c->cal.dig_P7 = BME_S16(a[18], a[19]);
    c->cal.dig_P8 = BME_S16(a[20], a[21]);
    c->cal.dig_P9 = BME_S16(a[22], a[23]);
    /* a[24] is register 0xA0, reserved. */
    c->cal.dig_H1 = a[25];                                /* 0xA1, unsigned  */

    c->cal.dig_H2 = BME_S16(b[0], b[1]);                  /* 0xE1, 0xE2      */
    c->cal.dig_H3 = b[2];                                 /* 0xE3, unsigned  */
    c->cal.dig_H4 = (int16_t)(((int16_t)(int8_t)b[3] << 4)
                              | (int16_t)(b[4] & 0x0Fu)); /* 0xE4, 0xE5[3:0] */
    c->cal.dig_H5 = (int16_t)(((int16_t)(int8_t)b[5] << 4)
                              | (int16_t)(b[4] >> 4));    /* 0xE6, 0xE5[7:4] */
    c->cal.dig_H6 = (int8_t)b[6];                         /* 0xE7, SIGNED    */

    #undef BME_U16
    #undef BME_S16
    return ESP_OK;
}

/* ====================================================================== *
 *  Compensation, from the Bosch Sensortec BME280 SensorAPI (BSD-3-Clause,
 *  (c) 2020 Bosch Sensortec GmbH), file bme280.c, integer versions with
 *  64-bit pressure. The arithmetic is transcribed unchanged; the only
 *  edits are the names:
 *      calib_data->dig_t1       ->  c->cal.dig_T1   (and all siblings)
 *      calib_data->t_fine       ->  c->t_fine
 *      uncomp_data->temperature ->  adc_T  (and pressure / humidity)
 *  Every constant, cast, shift and division is Bosch's.
 *
 *  UNITS -- these are NOT the units of the older datasheet listing:
 *      compensate_T  ->  0.01 C        (2150     == 21.50 C)
 *      compensate_P  ->  0.01 Pa       (10132500 == 101325 Pa)  NOT Q24.8
 *      compensate_H  ->  Q22.10 %RH    (divide by 1024)
 *
 *  Call order is not optional: compensate_T sets c->t_fine, and both of
 *  the others read it.
 * ====================================================================== */

static int32_t compensate_T(bme_ctx_t *c, int32_t adc_T)
{
    int32_t var1, var2, temperature;
    const int32_t temperature_min = -4000;
    const int32_t temperature_max =  8500;

    var1 = (int32_t)((adc_T / 8) - ((int32_t)c->cal.dig_T1 * 2));
    var1 = (var1 * ((int32_t)c->cal.dig_T2)) / 2048;
    var2 = (int32_t)((adc_T / 16) - ((int32_t)c->cal.dig_T1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)c->cal.dig_T3)) / 16384;
    c->t_fine = var1 + var2;
    temperature = (c->t_fine * 5 + 128) / 256;

    if (temperature < temperature_min)      temperature = temperature_min;
    else if (temperature > temperature_max) temperature = temperature_max;

    return temperature;
}

static uint32_t compensate_P(bme_ctx_t *c, int32_t adc_P)
{
    int64_t var1, var2, var3, var4;
    uint32_t pressure;
    const uint32_t pressure_min =  3000000;   /* 0.01 Pa ->  30000 Pa */
    const uint32_t pressure_max = 11000000;   /* 0.01 Pa -> 110000 Pa */

    var1 = ((int64_t)c->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)c->cal.dig_P6;
    var2 = var2 + ((var1 * (int64_t)c->cal.dig_P5) * 131072);
    var2 = var2 + (((int64_t)c->cal.dig_P4) * 34359738368);
    var1 = ((var1 * var1 * (int64_t)c->cal.dig_P3) / 256)
         + ((var1 * ((int64_t)c->cal.dig_P2) * 4096));
    var3 = ((int64_t)1) * 140737488355328;
    var1 = (var3 + var1) * ((int64_t)c->cal.dig_P1) / 8589934592;

    /* To avoid divide by zero exception */
    if (var1 != 0) {
        var4 = 1048576 - adc_P;
        var4 = (((var4 * INT64_C(2147483648)) - var2) * 3125) / var1;
        var1 = (((int64_t)c->cal.dig_P9) * (var4 / 8192) * (var4 / 8192)) / 33554432;
        var2 = (((int64_t)c->cal.dig_P8) * var4) / 524288;
        var4 = ((var4 + var1 + var2) / 256) + (((int64_t)c->cal.dig_P7) * 16);
        pressure = (uint32_t)(((var4 / 2) * 100) / 128);

        if (pressure < pressure_min)      pressure = pressure_min;
        else if (pressure > pressure_max) pressure = pressure_max;
    } else {
        pressure = pressure_min;
    }

    return pressure;
}

static uint32_t compensate_H(bme_ctx_t *c, int32_t adc_H)
{
    int32_t var1, var2, var3, var4, var5;
    uint32_t humidity;
    const uint32_t humidity_max = 102400;     /* Q22.10 -> 100.00 %RH */

    var1 = c->t_fine - ((int32_t)76800);
    var2 = (int32_t)(adc_H * 16384);
    var3 = (int32_t)(((int32_t)c->cal.dig_H4) * 1048576);
    var4 = ((int32_t)c->cal.dig_H5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)c->cal.dig_H6)) / 1024;
    var3 = (var1 * ((int32_t)c->cal.dig_H3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)c->cal.dig_H2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)c->cal.dig_H1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    humidity = (uint32_t)(var5 / 4096);

    if (humidity > humidity_max) humidity = humidity_max;

    return humidity;
}

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
    if (err != ESP_OK)          return err;
    if (chip_id != BME_CHIP_ID) return ESP_ERR_NOT_FOUND;

    err = read_calibration(c);
    if (err != ESP_OK) return err;

    /* ctrl_hum must be written BEFORE ctrl_meas: a change to ctrl_hum
       only takes effect when ctrl_meas is written afterwards. Written in
       the other order, humidity silently stays disabled. */
    err = reg_write(c, BME_REG_CTRL_HUM, BME_CTRL_HUM_VAL);
    if (err != ESP_OK) return err;
    err = reg_write(c, BME_REG_CONFIG, BME_CONFIG_VAL);
    if (err != ESP_OK) return err;
    err = reg_write(c, BME_REG_CTRL_MEAS, BME_CTRL_MEAS_VAL);
    if (err != ESP_OK) return err;

    c->present = true;
    return ESP_OK;
}

esp_err_t bme_driver_init(void)
{
    esp_err_t first_err = ESP_OK;
    for (int id = 0; id < BME_COUNT; id++) {
        esp_err_t err = init_one((bme_id_t)id);
        if (err != ESP_OK && first_err == ESP_OK) first_err = err;
    }
    return first_err;
}

bool bme_driver_present(bme_id_t id)
{
    if ((unsigned)id >= (unsigned)BME_COUNT) return false;
    return ctx[id].present;
}

esp_err_t bme_driver_read(bme_id_t id, bme_sample_t *out)
{
    if ((unsigned)id >= (unsigned)BME_COUNT || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

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
    const int32_t adc_H = (int32_t)(((uint32_t)b[6] << 8) | (uint32_t)b[7]);

    /* 0x80000 / 0x8000 are the reset values: the measurement is disabled
       or has not completed. Compensating them yields a confident wrong
       answer, so reject them here. */
    if (adc_T == 0x80000 || adc_P == 0x80000) return ESP_ERR_INVALID_RESPONSE;

    out->temperature_centi_c = compensate_T(c, adc_T);          /* must be first */
    out->pressure_pa         = compensate_P(c, adc_P) / 100u;   /* 0.01 Pa -> Pa */
    out->humidity_centi_pct  = (adc_H == 0x8000)
                             ? 0u
                             : (compensate_H(c, adc_H) * 100u) / 1024u;
    return ESP_OK;
}