#ifndef BME_DRIVER_H
#define BME_DRIVER_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

/* BME280, two of them on the shared sensor SPI bus:
     internal  U2    CS GPIO10   (inside the pressure vessel)
     external  J7.6  CS GPIO11   (outside, ambient)
   Both are driven through one set of functions with an instance handle,
   so there is one copy of the compensation code, not two. */

typedef enum {
    BME_INTERNAL = 0,
    BME_EXTERNAL = 1,
    BME_COUNT
} bme_id_t;

typedef struct {
    int32_t  temperature_centi_c;  /* 0.01 C     : 2150 == 21.50 C */
    uint32_t pressure_pa;          /* Pa         : 101325 == 1 atm */
    uint32_t humidity_centi_pct;   /* 0.01 %RH   : 4523 == 45.23 % */
} bme_sample_t;

/* Brings up both devices. Returns the first error encountered; use
   bme_driver_present() to find out which one answered. */
esp_err_t bme_driver_init(void);

/* True if that device returned the correct chip ID at init. */
bool bme_driver_present(bme_id_t id);

/* Reads and compensates one device. *out untouched on error. */
esp_err_t bme_driver_read(bme_id_t id, bme_sample_t *out);

#endif /* BME_DRIVER_H */