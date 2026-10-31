#ifndef SPI_INTERFACE_H
#define SPI_INTERFACE_H

#include <stdbool.h>
#include "driver/spi_master.h"

/* The SENSOR SPI bus, where the ESP32 is master: both BME280s, the
 * MAX31865, the LSM6DSM and the ABP.
 *
 * Separate peripheral and separate file from spi_pi_link, where the
 * ESP32 is a slave. Two SPI roles on one chip is normal; conflating
 * them in one module is not.
 *
 * Call once before any driver init. Each driver then adds itself as a
 * device with its own CS, mode and clock speed - because the devices
 * disagree about all three and the bus arbitrates per transaction.
 */
bool spi_interface_init(void);

/* Convenience wrapper so the four drivers do not each repeat the
   bus-config boilerplate. */
bool spi_interface_add_device(int cs_pin, uint8_t mode, int clock_hz,
                              spi_device_handle_t *out);

#endif /* SPI_INTERFACE_H */