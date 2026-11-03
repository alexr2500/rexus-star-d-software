#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

/* One consistent set of readings from every sensor on the board.
 *
 * Two questions get confused if they share a variable, so they do not:
 *   - is this reading usable RIGHT NOW?   -> the `valid` bitmask here
 *   - has this sensor EVER failed?        -> the latched error word in
 *                                            error_manager
 * A sensor that drops out for one cycle and recovers clears its valid
 * bit and then sets it again; its error bit stays latched, because the
 * ground needs to know it happened.
 */

/* Raw counts, scaled by protocol.h's *_SCALE_* factors. Fixed point
   rather than float: the wire cost halves and no sensor here has the
   accuracy to justify a float. */
#define SENSOR_RAW_INVALID  INT16_MIN

#define SENSOR_VALID_PT100    0x0001u
#define SENSOR_VALID_INT_BME  0x0002u
#define SENSOR_VALID_EXT_BME  0x0004u
#define SENSOR_VALID_SLF3S    0x0008u
#define SENSOR_VALID_ABP      0x0010u
#define SENSOR_VALID_IMU      0x0020u

typedef struct {
    uint32_t timestamp_ms;      /* when this set was taken */

    int16_t  pt100_temp;        /* 0.01 C  */
    int16_t  int_bme_temp;      /* 0.01 C  */
    int16_t  ext_bme_temp;      /* 0.01 C  */
    int16_t  int_bme_press;     /* BME_PRESSURE_SCALE_PA */
    int16_t  ext_bme_press;     /* BME_PRESSURE_SCALE_PA */
    int16_t  int_bme_hum;       /* 0.01 %RH */
    int16_t  ext_bme_hum;       /* 0.01 %RH */
    int16_t  slf3s_flow;        /* SLF3S_FLOW_SCALE */
    int16_t  slf3s_temp;        /* 0.005 C, SLF3S internal */
    int16_t  abp_pressure;      /* raw 14-bit ABP counts, 0..16383.
                                   NOT pascals -- see abp_driver.h */

    int16_t  ax, ay, az;        /* IMU_ACCEL_SCALE_MG  */
    int16_t  gx, gy, gz;        /* IMU_GYRO_SCALE_MDPS */

    uint16_t voltage_mV;        /* TODO: no source on this board revision.
                                   GPIO_ADC (GPIO4) is the potentiometer of
                                   SED fig. 38, not a bus-voltage monitor.
                                   Held at 0 until hardware provides one. */

    uint16_t slf3s_flags;       /* SLF3S_FLAG_* straight from the sensor */
    uint16_t valid;             /* SENSOR_VALID_* bitmask */
} sensor_snapshot_t;

/* Brings up every sensor driver. Returns false if any failed; the
   per-sensor outcome is visible through the valid bits after the first
   sensors_read(). Never aborts: one dead sensor must not stop the
   experiment. */
bool sensors_init(void);

/* Performs the actual bus traffic and publishes a new snapshot.
   Call from one task only, at the sensor rate. Slow: it talks to five
   devices. */
void sensors_read(void);

/* Returns the most recent complete snapshot. Cheap, safe from any task,
   and never returns a half-updated set. */
void sensors_get(sensor_snapshot_t *out);

#endif /* SENSORS_H */