#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#include <stdint.h>
#include "esp_err.h"

/* I2C bus and the one device on it: the Sensirion SLF3S-1300F liquid flow
   sensor, off-board on J4 (SDA on J4.2, SCL on J4.5).
   It is the only I2C device on this board; everything else is SPI. */

/* Brings up the bus and puts the sensor into continuous measurement. */
esp_err_t i2c_interface_init(void);

/* One measurement. Raw counts, scaled by the caller:
     flow_raw  / SLF3S_SCALE_FLOW_WATER  -> ml/min
     temp_raw  / SLF3S_SCALE_TEMP        -> degrees C
   flags carries the sensor's own status bits (air-in-line, high flow).
   Any of the three pointers may be NULL if that value is not wanted. */
esp_err_t i2c_interface_read_flow(int16_t *flow_raw,
                                  int16_t *temp_raw,
                                  uint16_t *flags);

#define SLF3S_SCALE_FLOW_WATER  500   /* counts per ml/min, water calibration */
#define SLF3S_SCALE_TEMP        200   /* counts per degree C */

/* Flag bits inside the third word, per the SLF3S datasheet. */
#define SLF3S_FLAG_AIR_IN_LINE  0x0002u
#define SLF3S_FLAG_HIGH_FLOW    0x0004u

#endif /* I2C_INTERFACE_H */