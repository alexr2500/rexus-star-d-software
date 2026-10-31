#ifndef BME_DRIVER_H
#define BME_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

/* Two BME280s on the sensor SPI bus: one inside the pressure vessel
 * (CS_BME280), one outside (CS_BME280_OUT).
 *
 * Outputs are already scaled to protocol.h units, so nothing downstream
 * sees a float:
 *   temp     0.01 degC/LSB   (BME_TEMP_SCALE_C)
 *   pressure 2 Pa/LSB        (BME_PRESSURE_SCALE_PA)
 *   humidity 0.01 %RH/LSB    (BME_HUMIDITY_SCALE_RH)
 */

typedef enum {
    BME_INTERNAL = 0,   /* inside the PV */
    BME_EXTERNAL = 1,   /* ambient       */
    BME_COUNT
} bme_id_t;

bool bme_init(bme_id_t id);

bool bme_read(bme_id_t id,
              int16_t  *temp_raw,
              uint16_t *pressure_raw,
              uint16_t *humidity_raw);

#endif /* BME_DRIVER_H */