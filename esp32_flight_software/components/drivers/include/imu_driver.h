#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* LSM6DSM (U4) on the shared sensor SPI bus, INT1 on GPIO18. */

typedef struct {
    int16_t ax, ay, az;   /* raw accelerometer counts */
    int16_t gx, gy, gz;   /* raw gyroscope counts     */
} imu_sample_t;

/* Scale at the configured full scale. Kept here so sensors.c and the
   ground station convert with the same numbers rather than two copies. */
#define IMU_ACCEL_MG_PER_LSB    0.488f    /* +-16 g  */
#define IMU_GYRO_MDPS_PER_LSB   70.0f     /* +-2000 dps */

esp_err_t imu_driver_init(void);

/* Reads all six axes in one burst. Returns ESP_OK and fills *out, or an
   error with *out untouched. */
esp_err_t imu_driver_read(imu_sample_t *out);

/* True when a new sample is waiting. Reads the INT1 pin, which costs no
   SPI traffic — unlike polling STATUS_REG. */
bool imu_driver_data_ready(void);

/* WHO_AM_I as read at init. 0x6A means the part answered correctly;
   0x00 or 0xFF almost always means the bus or the CS line is wrong. */
uint8_t imu_driver_whoami(void);

#endif /* IMU_DRIVER_H */