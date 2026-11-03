#ifndef PT100_DRIVER_H
#define PT100_DRIVER_H

#include <stdint.h>
#include "esp_err.h"

/* MAX31865 (IC1) + PT100, on the shared sensor SPI bus. */

esp_err_t pt100_driver_init(void);

/* Temperature in hundredths of a degree C (2150 == 21.50 C), matching the
   raw units used in telemetry and temperature_control.c.
   Returns ESP_OK only when the MAX31865 reports no fault AND the result
   is inside the sensor's valid range. On any error *out is untouched. */
esp_err_t pt100_driver_read(int16_t *out_centi_c);

/* Last fault byte read from register 0x07. Zero when healthy. For
   telemetry and ground diagnosis only; pt100_driver_read already acts
   on it. */
uint8_t pt100_driver_last_fault(void);

#endif /* PT100_DRIVER_H */