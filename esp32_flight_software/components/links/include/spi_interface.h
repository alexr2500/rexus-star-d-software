#ifndef SPI_INTERFACE_H
#define SPI_INTERFACE_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

/* The sensor SPI bus, on which the ESP32 is MASTER.
 *
 * Five devices share it: both BME280s, the LSM6DSM, the MAX31865 and the
 * off-board ABP. Each has its own chip select and its own clock rate and
 * SPI mode, which is why every device gets a handle rather than the bus
 * exposing a raw transfer call.
 *
 * This is NOT the Raspberry Pi link. That is a separate bus on separate
 * pins where the ESP32 is the slave -- see spi_pi_link.h.
 */

/* Initialises the bus itself. Call once, before any driver init. */
esp_err_t spi_interface_init(void);

/* Attaches one device to the bus. The returned handle is what the driver
   passes to spi_device_polling_transmit(). */
esp_err_t spi_interface_add_device(gpio_num_t cs_pin,
                                   int clock_hz,
                                   uint8_t spi_mode,
                                   spi_device_handle_t *out);

#endif /* SPI_INTERFACE_H */