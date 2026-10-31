#include "bme_driver.h"
#include "board_pins.h"
#include "driver/spi_master.h"

/* --- BME280 register map ---------------------------------------------
 * VERIFY against the BME280 datasheet, "Memory map". */
#define BME_REG_ID         0xD0
#define BME_ID_VALUE       0x60
#define BME_REG_RESET      0xE0
#define BME_REG_CALIB00    0x88   /* 0x88..0xA1, 26 bytes */
#define BME_REG_CALIB26    0xE1   /* 0xE1..0xE7,  7 bytes */
#define BME_REG_CTRL_HUM   0xF2
#define BME_REG_CTRL_MEAS  0xF4
#define BME_REG_CONFIG     0xF5
#define BME_REG_PRESS_MSB  0xF7   /* 0xF7..0xFE, 8 bytes burst */
#define BME_READ_FLAG      0x80   /* bit 7: 1 = read, 0 = write */

/* Calibration words, read once at init. Named exactly as the datasheet
   names them so the compensation code below can be diffed against it
   line by line. */
typedef struct {
    uint16_t dig_T1; int16_t dig_T2, dig_T3;
    uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5,
                             dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t  dig_H1; int16_t dig_H2; uint8_t dig_H3;
    int16_t  dig_H4, dig_H5; int8_t dig_H6;
    int32_t  t_fine;          /* set by the temperature compensation,
                                 consumed by pressure and humidity */
} bme_calib_t;

static spi_device_handle_t dev[BME_COUNT];
static bme_calib_t         calib[BME_COUNT];


/* =====================================================================
 * COMPENSATION - DO NOT WRITE THIS FROM MEMORY
 * =====================================================================
 * The BME280 raw ADC values are meaningless without the manufacturer's
 * compensation formulas. They are about forty lines of specific magic
 * constants and shifts, published as reference C in the BME280
 * datasheet (section 4.2.3, "Compensation formulas") and in Bosch's
 * BME280_driver on GitHub.
 *
 * COPY THEM VERBATIM. Do not retype them, do not simplify them, do not
 * let anything generate them. A single wrong shift gives temperatures
 * that look entirely plausible - off by a degree or two - and nothing
 * in any test will catch it. This is the one place in the firmware
 * where a transcription error is invisible.
 *
 * The signatures to fill:
 *   static int32_t  bme280_compensate_T_int32(bme_calib_t *c, int32_t adc_T);
 *   static uint32_t bme280_compensate_P_int64(bme_calib_t *c, int32_t adc_P);
 *   static uint32_t bme280_compensate_H_int32(bme_calib_t *c, int32_t adc_H);
 *
 * Datasheet output units, and the conversion to ours:
 *   T  0.01 degC          -> already our unit, cast to int16_t
 *   P  Q24.8 Pa           -> (P / 256) / 2  to reach 2 Pa/LSB
 *   H  Q22.10 %RH         -> (H / 1024) * 100  to reach 0.01 %RH/LSB
 *
 * Verify with the datasheet's own worked example before wiring these
 * into telemetry.
 * ===================================================================== */


static bool bme_spi_read(bme_id_t id, uint8_t reg, uint8_t *buf, size_t len)
{
    spi_transaction_t t = {
        .addr      = reg | BME_READ_FLAG,
        .length    = len * 8,
        .rxlength  = len * 8,
        .rx_buffer = buf,
    };
    return spi_device_polling_transmit(dev[id], &t) == ESP_OK;
}


bool bme_init(bme_id_t id)
{
    if (id >= BME_COUNT) { return false; }

    /* TODO:
       1. spi_bus_add_device() with PIN_CS_BME_INT or PIN_CS_BME_EXT.
          BME280 is SPI mode 0 or 3 - VERIFY.
       2. Read BME_REG_ID, require 0x60. A BMP280 answers 0x58 and has
          no humidity channel; catching that here is cheaper than
          wondering why humidity is always zero.
       3. Read both calibration blocks and unpack into calib[id].
          dig_H4 and dig_H5 are 12-bit values packed across shared
          bytes - this unpacking is a classic source of wrong humidity
          and must come from the datasheet, not from intuition.
       4. ctrl_hum BEFORE ctrl_meas: the humidity setting only takes
          effect when ctrl_meas is written afterwards. Writing them in
          the other order silently leaves humidity disabled.
       5. Oversampling and filter: TODO, decide from the measurement
          rate. sensors_read() runs at 10 Hz. */
    return false;
}


bool bme_read(bme_id_t id,
              int16_t *temp_raw, uint16_t *pressure_raw, uint16_t *humidity_raw)
{
    if (id >= BME_COUNT) { return false; }

    uint8_t b[8];
    if (!bme_spi_read(id, BME_REG_PRESS_MSB, b, sizeof(b))) {
        return false;
    }

    /* One burst read of all three, so pressure, temperature and
       humidity come from the same conversion. Three separate reads
       could straddle a conversion and mix two measurements. */
    const int32_t adc_P = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[2] >> 4);
    const int32_t adc_T = ((int32_t)b[3] << 12) | ((int32_t)b[4] << 4) | (b[5] >> 4);
    const int32_t adc_H = ((int32_t)b[6] <<  8) |  (int32_t)b[7];

    /* 0x80000 in a 20-bit channel means "no measurement yet": the
       device was reset, or is in sleep mode, or the first conversion
       has not completed. It is not a reading. */
    if (adc_T == 0x80000 || adc_P == 0x80000) {
        return false;
    }

    /* TODO: call the three compensation functions, in this order -
       temperature first, because it sets t_fine, which the other two
       need. Then convert to our scaled units as documented above. */
    (void)adc_H; (void)temp_raw; (void)pressure_raw; (void)humidity_raw;
    return false;
}