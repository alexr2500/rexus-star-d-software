#ifndef PT100_DRIVER_H
#define PT100_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

/* PT100 RTD via MAX31865, SPI.
 *
 * Returns temperature in 0.01 degC/LSB to match PT100_TEMP_SCALE_C in
 * protocol.h - the conversion to a scaled integer happens here, once,
 * so nothing downstream ever handles a float.
 */

bool pt100_init(void);

/* Returns false on SPI failure OR on a MAX31865 fault (open circuit,
   short, out-of-range). Caller must not set SENSOR_VALID_PT100 unless
   this returns true. */
bool pt100_read(int16_t *temp_raw);

#endif /* PT100_DRIVER_H */