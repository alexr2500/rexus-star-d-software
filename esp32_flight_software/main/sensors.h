#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

/* One consistent snapshot of every sensor on the ESP32's buses.
 *
 * Read and get are separate on purpose. sensors_read() talks to the
 * hardware and is slow; sensors_get() copies the latest complete
 * snapshot and is fast. Anything building a packet calls get, so it
 * always sees values from ONE instant - six separate reads during
 * packet assembly could straddle an update and produce a packet that
 * mixes two moments.
 *
 * Field order and types mirror MSG_SENSOR_DATA in protocol.h, so the
 * Pi link can serialise straight from this struct.
 */

/* Which readings in this snapshot are trustworthy RIGHT NOW.
 *
 * Not the same question as the latched error bits: those answer "did
 * this sensor ever fail", which is what ground needs post-flight. This
 * answers "is the number next to it usable this tick", which is what
 * telemetry and the thermal loop need. Bit order matches
 * ERR_BIT_PT100..ERR_BIT_LSM6DSM so the two are easy to correlate.
 */
#define SENSOR_VALID_PT100    (1u << 0)
#define SENSOR_VALID_INT_BME  (1u << 1)
#define SENSOR_VALID_EXT_BME  (1u << 2)
#define SENSOR_VALID_SLF3S    (1u << 3)
#define SENSOR_VALID_ABP      (1u << 4)
#define SENSOR_VALID_IMU      (1u << 5)
#define SENSOR_VALID_VOLTAGE  (1u << 6)

/* Written into any field whose read failed. INT16_MIN is -327.68 in
   every scaled unit we use - visibly not a reading, so a half-finished
   build cannot be mistaken for a working one on the ground display. */
#define SENSOR_RAW_INVALID  INT16_MIN

typedef struct {
    int16_t  ext_bme_temp;        /* 0.01 degC/LSB  */
    uint16_t ext_bme_pressure;    /* 2 Pa/LSB       */
    uint16_t ext_bme_humidity;    /* 0.01 %RH/LSB   */
    int16_t  int_bme_temp;
    uint16_t int_bme_pressure;
    uint16_t int_bme_humidity;
    uint16_t abp_pressure;
    int16_t  slf3s_flow;          /* 0.002 ml/min/LSB */
    int16_t  pt100_temp;          /* 0.01 degC/LSB  */
    int16_t  imu_accel[3];        /* 0.488 mg/LSB, X Y Z */
    int16_t  imu_gyro[3];         /* 70 mdps/LSB         */
    uint16_t voltage_mV;          /* main PCB rail       */
    uint16_t valid;               /* SENSOR_VALID_* bits */
} sensor_snapshot_t;

/* Bring up the buses and the devices. Call once from app_main. */
void sensors_init(void);

/* Read every sensor and replace the stored snapshot. Call at
   SENSOR_DATA_PERIOD_MS (100 ms) from the sensor task. */
void sensors_read(void);

/* Copy the latest complete snapshot. Safe to call from any task. */
void sensors_get(sensor_snapshot_t *out);

#endif /* SENSORS_H */