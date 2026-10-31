#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

/* LSM6DSM accelerometer + gyroscope, SPI.
 *
 * Ranges are fixed by protocol.h: IMU_ACCEL_SCALE_MG 0.488 is +/-16 g
 * and IMU_GYRO_SCALE_MDPS 70.0 is +/-2000 dps. Do not change either
 * without changing protocol.h and the Pi's framing in the same commit.
 *
 * +/-16 g is also what makes launch detection possible - peak launch
 * acceleration must fit inside it. Confirm against the REXUS User
 * Manual acceleration profile.
 */

bool imu_init(void);

/* Six raw values, LSB as above. accel[] and gyro[] are X, Y, Z in the
 * sensor's own frame.
 * TODO: which index is along the rocket axis depends on how the PCB
 * sits in the structure. Verify by tilting the assembled hardware, not
 * by reading the drawing.
 */
bool imu_read(int16_t accel[3], int16_t gyro[3]);

#endif /* IMU_DRIVER_H */