#include "health_monitor.h"
#include "protocol.h"
#include "error_manager.h"
#include "i2c_interface.h"   /* SLF3S_FLAG_* */
#include <stdlib.h>

/* ---- limits, in raw units (see telemetry_flags.md) -------------------- */

/* Biological band is 18-25 C; outside 15-30 C something is wrong with
   the control loop, not just the setpoint. 0.01 C per count. */
#define TEMP_MIN_RAW        1500
#define TEMP_MAX_RAW        3000

/* Vessel pressure, BME_PRESSURE_SCALE_PA (2 Pa per count) in an int16.
 *
 * KNOWN LIMITATION, deliberately not worked around here. At 2 Pa per
 * count an int16 tops out at 32767 counts = 65534 Pa, which is BELOW
 * sea-level pressure (101325 Pa). Both pressure fields therefore sit
 * saturated at 32767 under normal conditions, and small leaks are
 * invisible. The format is being left alone until bench testing; see
 * protocol.h BME_PRESSURE_SCALE_PA.
 *
 * The check below is still worth having: a serious breach drops the
 * reading out of saturation and into a detectable range. The threshold
 * is set well inside int16 so the comparison is reachable -- 25000
 * counts is 50000 Pa, roughly half an atmosphere.
 *
 * There is no upper check: the field cannot exceed 32767 by
 * construction, so one would be dead code. */
#define PRESS_BREACH_BELOW_RAW   25000

/* Flow, SLF3S_FLOW_SCALE. Sustained zero while the pump is running is
   a blockage; the pump state is not visible here, so this only flags
   impossible values. */
#define FLOW_ABS_MAX_RAW    30000

/* An IMU is considered frozen when consecutive samples differ by less
   than this on every axis for IMU_STATIC_CYCLES cycles. 20 counts at
   0.488 mg is about 10 mg -- below real sensor noise, so a genuinely
   still sensor still jitters past it. */
#define IMU_STATIC_LIMIT    20
#define IMU_STATIC_CYCLES   50

/* A saturated axis also reports a constant value, and at +-16 g the
   accelerometer saturates through the whole REXUS boost (~17.5 g plus
   1 g of gravity). Without this guard the frozen-IMU check would fire
   during launch -- exactly when the sensor is working correctly. A
   pegged sensor is not a stuck sensor, so saturation suspends the
   check rather than feeding it. */
#define IMU_SATURATED_RAW   32000

static int16_t  last_ax, last_ay, last_az;
static uint16_t static_cycles;
static bool     frozen;

void health_monitor_init(void)
{
    last_ax = last_ay = last_az = 0;
    static_cycles = 0;
    frozen = false;
}

static bool within(int16_t v, int32_t lo, int32_t hi)
{
    return (int32_t)v >= lo && (int32_t)v <= hi;
}

void health_monitor_update(const sensor_snapshot_t *s, bool flight_confirmed)
{
    if (s == NULL) return;

    /* Every check is guarded by the valid bit. An invalid reading has
       already raised its own sensor error; judging its contents as well
       would raise a second, misleading fault. */

    if (s->valid & SENSOR_VALID_PT100) {
        if (!within(s->pt100_temp, TEMP_MIN_RAW, TEMP_MAX_RAW)) {
            error_raise(ERR_BIT_THERMAL_FAULT);
        }
    }

    if (s->valid & SENSOR_VALID_INT_BME) {
        if (!within(s->int_bme_temp, TEMP_MIN_RAW, TEMP_MAX_RAW)) {
            error_raise(ERR_BIT_THERMAL_FAULT);
        }
        if (s->int_bme_press < PRESS_BREACH_BELOW_RAW) {
            error_raise(ERR_BIT_FLUIDIC_FAULT);   /* vessel integrity */
        }
    }

    /* Suppressed once flying: ambient pressure is SUPPOSED to collapse. */
    if (!flight_confirmed && (s->valid & SENSOR_VALID_EXT_BME)) {
        if (s->ext_bme_press < PRESS_BREACH_BELOW_RAW) {
            error_raise(ERR_BIT_EXT_BME);
        }
    }

    if (s->valid & SENSOR_VALID_SLF3S) {
        if (abs((int)s->slf3s_flow) > FLOW_ABS_MAX_RAW) {
            error_raise(ERR_BIT_FLUIDIC_FAULT);
        }
        if (s->slf3s_flags & SLF3S_FLAG_AIR_IN_LINE) {
            error_raise(ERR_BIT_FLUIDIC_FAULT);
        }
    }

    /* ---- frozen-IMU detection ---------------------------------------- */
    if (s->valid & SENSOR_VALID_IMU) {
        const bool saturated = abs((int)s->ax) >= IMU_SATURATED_RAW
                            || abs((int)s->ay) >= IMU_SATURATED_RAW
                            || abs((int)s->az) >= IMU_SATURATED_RAW;

        const bool still = abs((int)s->ax - (int)last_ax) < IMU_STATIC_LIMIT
                        && abs((int)s->ay - (int)last_ay) < IMU_STATIC_LIMIT
                        && abs((int)s->az - (int)last_az) < IMU_STATIC_LIMIT;

        if (saturated) {
            static_cycles = 0;
            frozen = false;
        } else if (still) {
            if (static_cycles < IMU_STATIC_CYCLES) static_cycles++;
            if (static_cycles >= IMU_STATIC_CYCLES && !frozen) {
                frozen = true;
                error_raise(ERR_BIT_LSM6DSM);
            }
        } else {
            static_cycles = 0;
            frozen = false;
        }

        last_ax = s->ax; last_ay = s->ay; last_az = s->az;
    }
}

bool imu_is_frozen(void) { return frozen; }