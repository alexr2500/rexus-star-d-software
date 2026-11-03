#include "sensors.h"
#include "protocol.h"
#include "error_manager.h"
#include "bme_driver.h"
#include "imu_driver.h"
#include "pt100_driver.h"
#include "abp_driver.h"
#include "i2c_interface.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

/* The published snapshot, and the lock that keeps it whole.
 *
 * sensors_read() spends milliseconds talking to five devices; a reader
 * that walked the struct during that time could take the temperature
 * from one cycle and the pressure from the next. So the readings are
 * assembled into a local, and only the final copy is done inside the
 * critical section -- a few hundred nanoseconds with interrupts off,
 * rather than milliseconds. */
static sensor_snapshot_t published;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

/* Clamps a wider reading into the int16 the wire carries. Casting a
   value outside int16 range is undefined behaviour in C, not a wrap,
   so the clamp is not optional. */
static int16_t clamp_i16(int32_t v)
{
    if (v > INT16_MAX) return INT16_MAX;
    if (v < INT16_MIN + 1) return (int16_t)(INT16_MIN + 1);  /* leave
                                 SENSOR_RAW_INVALID free as a sentinel */
    return (int16_t)v;
}

bool sensors_init(void)
{
    bool all_ok = true;

    if (pt100_driver_init()  != ESP_OK) all_ok = false;
    if (bme_driver_init()    != ESP_OK) all_ok = false;
    if (imu_driver_init()    != ESP_OK) all_ok = false;
    if (i2c_interface_init() != ESP_OK) all_ok = false;
    if (abp_driver_init()    != ESP_OK) all_ok = false;

    memset(&published, 0, sizeof published);
    published.valid = 0u;
    return all_ok;
}

void sensors_read(void)
{
    sensor_snapshot_t s;
    memset(&s, 0, sizeof s);
    s.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);

    /* Start every field invalid. A driver that fails leaves its field at
       the sentinel rather than at a stale value from last cycle, so a
       frozen sensor cannot masquerade as a working one. */
    s.pt100_temp   = SENSOR_RAW_INVALID;
    s.int_bme_temp = SENSOR_RAW_INVALID;
    s.ext_bme_temp = SENSOR_RAW_INVALID;
    s.int_bme_press = SENSOR_RAW_INVALID;
    s.ext_bme_press = SENSOR_RAW_INVALID;
    s.int_bme_hum  = SENSOR_RAW_INVALID;
    s.ext_bme_hum  = SENSOR_RAW_INVALID;
    s.slf3s_flow   = SENSOR_RAW_INVALID;
    s.slf3s_temp   = SENSOR_RAW_INVALID;
    s.abp_pressure = SENSOR_RAW_INVALID;
    s.ax = s.ay = s.az = SENSOR_RAW_INVALID;
    s.gx = s.gy = s.gz = SENSOR_RAW_INVALID;

    /* ---- PT100, via MAX31865 ---------------------------------------- */
    int16_t t = 0;
    if (pt100_driver_read(&t) == ESP_OK) {
        s.pt100_temp = t;
        s.valid |= SENSOR_VALID_PT100;
    } else {
        error_raise(ERR_BIT_PT100);
    }

    /* ---- the two BME280s -------------------------------------------- */
    bme_sample_t b;
    if (bme_driver_read(BME_INTERNAL, &b) == ESP_OK) {
        s.int_bme_temp  = clamp_i16(b.temperature_centi_c);
        s.int_bme_press = clamp_i16((int32_t)(b.pressure_pa / 2u));  /* BME_PRESSURE_SCALE_PA */
        s.int_bme_hum   = clamp_i16((int32_t)b.humidity_centi_pct);
        s.valid |= SENSOR_VALID_INT_BME;
    } else {
        error_raise(ERR_BIT_INT_BME);
    }

    if (bme_driver_read(BME_EXTERNAL, &b) == ESP_OK) {
        s.ext_bme_temp  = clamp_i16(b.temperature_centi_c);
        s.ext_bme_press = clamp_i16((int32_t)(b.pressure_pa / 2u));
        s.ext_bme_hum   = clamp_i16((int32_t)b.humidity_centi_pct);
        s.valid |= SENSOR_VALID_EXT_BME;
    } else {
        error_raise(ERR_BIT_EXT_BME);
    }

    /* ---- IMU --------------------------------------------------------- */
    imu_sample_t imu;
    if (imu_driver_read(&imu) == ESP_OK) {
        s.ax = imu.ax; s.ay = imu.ay; s.az = imu.az;
        s.gx = imu.gx; s.gy = imu.gy; s.gz = imu.gz;
        s.valid |= SENSOR_VALID_IMU;
    } else {
        error_raise(ERR_BIT_LSM6DSM);
    }

    /* ---- SLF3S flow sensor, the only I2C device ---------------------- */
    int16_t flow = 0, ftemp = 0;
    uint16_t fflags = 0;
    if (i2c_interface_read_flow(&flow, &ftemp, &fflags) == ESP_OK) {
        s.slf3s_flow  = flow;
        s.slf3s_temp  = ftemp;
        s.slf3s_flags = fflags;
        s.valid |= SENSOR_VALID_SLF3S;
    } else {
        error_raise(ERR_BIT_SLF3S);
    }

    /* ---- ABP pressure sensor, off-board on J1 ------------------------ */
    /* Raw 14-bit counts, not pascals: the counts-to-pressure transfer
       function needs the part's range, which is not confirmed yet. See
       abp_driver.h. Transmitting counts keeps the flight data correct
       and lets the conversion be fixed on the ground afterwards. */
    int16_t abp_counts = 0;
    if (abp_driver_read(&abp_counts, NULL) == ESP_OK) {
        s.abp_pressure = abp_counts;
        s.valid |= SENSOR_VALID_ABP;
    } else {
        error_raise(ERR_BIT_ABP);
    }

    /* ---- bus voltage -------------------------------------------------- */
    /* FIXME: no source on this board revision. Left at zero rather than
       filled with the potentiometer reading from GPIO4, which would be a
       plausible-looking number that means nothing. */
    s.voltage_mV = 0u;

    portENTER_CRITICAL(&lock);
    published = s;
    portEXIT_CRITICAL(&lock);
}

void sensors_get(sensor_snapshot_t *out)
{
    if (out == NULL) return;
    portENTER_CRITICAL(&lock);
    *out = published;
    portEXIT_CRITICAL(&lock);
}