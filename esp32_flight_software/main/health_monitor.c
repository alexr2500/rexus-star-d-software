#include "health_monitor.h"
#include "error_manager.h"
#include "protocol.h"
#include <stdlib.h>

/* Limits in RAW units, derived from the scale factors in protocol.h so
   the conversion is written once and visible. */

/* BME_TEMP_SCALE_C / PT100_TEMP_SCALE_C are both 0.01 degC per LSB. */
#define TEMP_RAW_MIN   (-4000)   /* -40.00 degC */
#define TEMP_RAW_MAX     8500    /* +85.00 degC */

/* BME_PRESSURE_SCALE_PA is 2 Pa per LSB. */
#define BME_P_RAW_MIN   15000u   /*  300 hPa =  30000 Pa */
#define BME_P_RAW_MAX   55000u   /* 1100 hPa = 110000 Pa */

/* SLF3S_FLOW_SCALE is 0.002 ml/min per LSB; the part is 40 ml/min FS. */
#define FLOW_RAW_ABS_MAX 20000

/* Sample temperature band, SED 4.6 / 4.10: 18-25 degC biological
   working range. Outside it raises THERMAL_FAULT (bit 10, not latched -
   it describes a condition that can recover). */
#define SAMPLE_TEMP_RAW_MIN  1800
#define SAMPLE_TEMP_RAW_MAX  2500

/* Consecutive identical IMU samples before we call it frozen.
   telemetry_flags.md bit 5: "reads returning static values". Real
   readings always dither by at least one LSB from noise, so identical
   values repeated this many times means the bus or the device is
   stuck rather than genuinely still. */
#define IMU_STATIC_LIMIT  20


/* Raise or clear in one place, so no caller can forget the clear half.
   Bits in ERR_LATCH_MASK ignore the clear - that is error_manager's
   job, not ours. */
static void flag(bool fault, uint8_t bit)
{
    if (fault) {
        error_raise(bit);
    } else {
        error_clear(bit);
    }
}

static bool temp_out_of_range(int16_t raw)
{
    return (raw < TEMP_RAW_MIN) || (raw > TEMP_RAW_MAX);
}


/* Returns true when the IMU has returned byte-identical readings for
   too long. Static across a few samples is normal; static across two
   seconds on a vibrating rocket is not. */
static bool imu_is_frozen(const sensor_snapshot_t *s)
{
    static int16_t last[6];
    static uint16_t identical = 0;

    int16_t now[6] = {
        s->imu_accel[0], s->imu_accel[1], s->imu_accel[2],
        s->imu_gyro[0],  s->imu_gyro[1],  s->imu_gyro[2],
    };

    bool same = true;
    for (int i = 0; i < 6; i++) {
        if (now[i] != last[i]) { same = false; }
        last[i] = now[i];
    }

    if (same) {
        if (identical < IMU_STATIC_LIMIT) { identical++; }
    } else {
        identical = 0;
    }

    return identical >= IMU_STATIC_LIMIT;
}


void health_monitor_check(const sensor_snapshot_t *s, bool flight_confirmed)
{
    /* --- bit 0: PT100 ------------------------------------------------ */
    flag(!(s->valid & SENSOR_VALID_PT100) || temp_out_of_range(s->pt100_temp),
         ERR_BIT_PT100);

    /* --- bit 1: internal BME280 -------------------------------------- */
    /* Inside the sealed pressure vessel, so the range check applies for
       the whole mission - unlike the external one below. */
    flag(!(s->valid & SENSOR_VALID_INT_BME)
         || temp_out_of_range(s->int_bme_temp)
         || s->int_bme_pressure < BME_P_RAW_MIN
         || s->int_bme_pressure > BME_P_RAW_MAX,
         ERR_BIT_INT_BME);

    /* --- bit 2: external BME280 -------------------------------------- */
    bool ext_fault = !(s->valid & SENSOR_VALID_EXT_BME)
                     || temp_out_of_range(s->ext_bme_temp);

    /* Pressure only checked on the ground. Above ~9 km the reading is
       legitimately below the sensor's floor; flagging that would mark a
       perfectly good sensor faulty for most of the flight, and the bit
       latches, so it would never clear again. */
    if (!flight_confirmed) {
        ext_fault = ext_fault
                    || s->ext_bme_pressure < BME_P_RAW_MIN
                    || s->ext_bme_pressure > BME_P_RAW_MAX;
    }
    flag(ext_fault, ERR_BIT_EXT_BME);

    /* --- bit 3: SLF3S flow ------------------------------------------- */
    /* The driver owns the sensor-word CRC check (flags doc bit 3); a
       failed CRC must clear the valid bit. Here we only range-check. */
    flag(!(s->valid & SENSOR_VALID_SLF3S)
         || abs((int)s->slf3s_flow) > FLOW_RAW_ABS_MAX,
         ERR_BIT_SLF3S);

    /* --- bit 4: ABP pressure ----------------------------------------- */
    /* TODO: add the range check once the part's rated pressure range is
       in the SED. Until then a read failure is the only detectable
       fault, and an in-range-looking garbage value would pass. */
    flag(!(s->valid & SENSOR_VALID_ABP), ERR_BIT_ABP);

    /* --- bit 5: LSM6DSM ---------------------------------------------- */
    flag(!(s->valid & SENSOR_VALID_IMU) || imu_is_frozen(s),
         ERR_BIT_LSM6DSM);

    /* --- bit 10: sample temperature out of band ----------------------- */
    /* Only meaningful with a trustworthy reading: an unknown temperature
       is not evidence of a thermal fault, and raising the bit on a dead
       sensor would hide which of the two actually failed. The PT100's
       own bit already reports that case. */
    if (s->valid & SENSOR_VALID_PT100) {
        flag(s->pt100_temp < SAMPLE_TEMP_RAW_MIN
             || s->pt100_temp > SAMPLE_TEMP_RAW_MAX,
             ERR_BIT_THERMAL_FAULT);
    }
}