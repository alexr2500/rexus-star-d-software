#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* I2C master. Exactly one device: the SLF3S-1300F flow sensor.
 * Everything else on this board is SPI (SED 4.8.2.2, and the CS_* nets
 * in the schematic).
 *
 * Every call has a timeout and returns false rather than blocking. An
 * I2C bus can hang indefinitely if a slave holds SDA low, and a sensor
 * read must never be able to stall the task that feeds the watchdog.
 */

void i2c_interface_init(void);

bool i2c_write(uint8_t addr7, const uint8_t *data, size_t len);
bool i2c_read(uint8_t addr7, uint8_t *data, size_t len);

/* Sensirion devices return data as 16-bit words each followed by a
   CRC-8 byte. This checks and strips it. A CRC mismatch is what
   telemetry_flags.md bit 3 means by "CRC mismatch on the sensor word" -
   a different CRC entirely from the CRC-16 used for framing. */
bool i2c_read_sensirion(uint8_t addr7, uint16_t *words, size_t nwords);

#endif /* I2C_INTERFACE_H */