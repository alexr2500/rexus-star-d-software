/* Sensor aggregation. Owns the snapshot; owns no device.
 *
 * Every read below is a TODO because components/drivers is empty. The
 * structure is final though: when the drivers land, each TODO becomes
 * one call and nothing else in the firmware changes.
 */

#include "sensors.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

static sensor_snapshot_t snapshot;
static portMUX_TYPE snapshot_mux = portMUX_INITIALIZER_UNLOCKED;


void sensors_init(void)
{
    /* Start every field invalid. A sensor that was never read must not
       look like a sensor that read zero. */
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.ext_bme_temp = SENSOR_RAW_INVALID;
    snapshot.int_bme_temp = SENSOR_RAW_INVALID;
    snapshot.slf3s_flow   = SENSOR_RAW_INVALID;
    snapshot.pt100_temp   = SENSOR_RAW_INVALID;
    for (int i = 0; i < 3; i++) {
        snapshot.imu_accel[i] = SENSOR_RAW_INVALID;
        snapshot.imu_gyro[i]  = SENSOR_RAW_INVALID;
    }
    snapshot.valid = 0;

    /* TODO(step 8): i2c_interface_init(), then each device's init.
       The LSM6DSM init must check WHO_AM_I - telemetry_flags.md bit 5
       names that as a fault condition. */
}


void sensors_read(void)
{
    /* Build into a local, then publish in one go. Writing straight into
       the shared snapshot would let a reader see half-old, half-new. */
    sensor_snapshot_t s;
    memset(&s, 0, sizeof(s));
    s.valid = 0;

    /* TODO(step 8): each block becomes
           if (bme280_read(BME_INT, &t, &p, &h) == OK) {
               s.int_bme_temp = t; ...; s.valid |= SENSOR_VALID_INT_BME;
           } else {
               s.int_bme_temp = SENSOR_RAW_INVALID; ...
           }
       Set the valid bit ONLY on success. health_monitor turns a missing
       valid bit into the latched error bit; this file does not raise
       error bits itself, so there is one place that decides what counts
       as a fault. */

    s.ext_bme_temp     = SENSOR_RAW_INVALID;   /* TODO bme280_read */
    s.ext_bme_pressure = 0;
    s.ext_bme_humidity = 0;

    s.int_bme_temp     = SENSOR_RAW_INVALID;   /* TODO bme280_read */
    s.int_bme_pressure = 0;
    s.int_bme_humidity = 0;

    s.abp_pressure     = 0;                    /* TODO abp_read    */
    s.slf3s_flow       = SENSOR_RAW_INVALID;   /* TODO slf3s_read  */
    s.pt100_temp       = SENSOR_RAW_INVALID;   /* TODO max31865_read */

    for (int i = 0; i < 3; i++) {              /* TODO lsm6dsm_read */
        s.imu_accel[i] = SENSOR_RAW_INVALID;
        s.imu_gyro[i]  = SENSOR_RAW_INVALID;
    }

    s.voltage_mV = 0;                          /* TODO ADC + divider */

    /* Publish. Short enough for a critical section - it is a memcpy of
       ~34 bytes with interrupts disabled, not a bus transaction. */
    portENTER_CRITICAL(&snapshot_mux);
    snapshot = s;
    portEXIT_CRITICAL(&snapshot_mux);
}


void sensors_get(sensor_snapshot_t *out)
{
    portENTER_CRITICAL(&snapshot_mux);
    *out = snapshot;
    portEXIT_CRITICAL(&snapshot_mux);
}