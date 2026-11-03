#include "state_machine.h"
#include "mode_manager.h"
#include "error_manager.h"
#include "rs422_interface.h"
#include "uart_tcu.h"
#include "health_monitor.h"
#include "sensors.h"
#include <stdlib.h>
#include <stdint.h>

/* Launch detection threshold, stated in m/s^2 and converted to raw IMU
 * counts at compile time so the physical number is the one in the file.
 *
 * Justification: the REXUS boost reaches roughly Mach 1 (~343 m/s) in
 * about 2 s, so mean boost acceleration is ~171 m/s^2, about 17.5 g.
 * 50 m/s^2 (~5.1 g) is under a third of that -- comfortably detected --
 * while being far above anything ground handling produces, especially
 * as it must hold for LAUNCH_CONFIRM_CYCLES consecutive cycles.
 *
 * NOTE: the accelerometer also reads the 1 g of gravity, and at +-16 g
 * full scale it SATURATES through the whole boost (17.5 + 1 g). That is
 * harmless for detection -- a pegged axis is unambiguous -- but it means
 * the boost profile itself cannot be measured with this configuration.
 *
 * TODO: the thrust axis is assumed to be Z. Confirm against the IMU's
 * mounting orientation; the magnitude test below is sign-independent but
 * not axis-independent.
 */
#define LAUNCH_ACCEL_MSS   50

/* m/s^2 -> milli-g -> raw counts. 1 g = 9.80665 m/s^2, so
   milli_g = m/s^2 * 1000 / 9.80665 = m/s^2 * 101971 / 1000.
   counts  = milli_g * 1000 / 488   (IMU_ACCEL_SCALE_MG is 0.488 mg/LSB) */
#define LAUNCH_ACCEL_MG    (((int32_t)LAUNCH_ACCEL_MSS * 101971) / 1000)
#define LAUNCH_ACCEL_RAW   ((int32_t)((LAUNCH_ACCEL_MG * 1000) / 488))

_Static_assert(LAUNCH_ACCEL_RAW < 32767,
               "launch threshold is outside the int16 the IMU reports");
_Static_assert(LAUNCH_ACCEL_RAW > 4098,
               "launch threshold is below 2 g: ground handling would trip it");

/* Consecutive cycles above the threshold before launch is believed.
   One sample is a knock; half a second is a rocket. */
#define LAUNCH_CONFIRM_CYCLES  5

static uint8_t launch_cycles;
static bool    launch_latched;

/* Launch detection exists so that a failed LO wire does not cost the
   whole experiment. LO remains the primary signal; this is the
   independent second opinion. */
static bool launch_detected(void)
{
    if (launch_latched) return true;

    sensor_snapshot_t s;
    sensors_get(&s);

    if (!(s.valid & SENSOR_VALID_IMU) || imu_is_frozen()) {
        launch_cycles = 0;
        return false;
    }

    /* Magnitude on the thrust axis, sign-independent until the mounting
       orientation is confirmed. */
    if ((int32_t)abs((int)s.az) > LAUNCH_ACCEL_RAW) {
        if (launch_cycles < LAUNCH_CONFIRM_CYCLES) launch_cycles++;
        if (launch_cycles >= LAUNCH_CONFIRM_CYCLES) {
            launch_latched = true;
            return true;
        }
    } else {
        launch_cycles = 0;
    }
    return false;
}

void mode_inputs_collect(mode_inputs_t *out)
{
    if (out == NULL) return;

    /* One sample of all three discretes per cycle. Everything below
       reads the debounced levels, which are side-effect free, so the
       order of these reads does not matter. */
    rs422_sample();

    out->soe              = rs422_SOE_get();
    out->flight_confirmed = rs422_LO_get() || launch_detected();
    out->test_requested   = uart_tcu_take_test_request();

    /* startup_complete and tests_passed are deliberately NOT touched
       here. They are not hardware reads -- they come from the POST and
       from the MODE_T sequence -- so the caller sets them before
       calling and this function leaves them alone. */

    /* SOE before LO is physically possible -- the operator can assert
       SOE manually at any time -- but it means the timeline is not
       running as expected, so it is worth telling the ground.
       This check lives here, not in rs422_interface, because it is a
       judgement about mission phase and that module only reports
       levels. */
    if (out->soe && !out->flight_confirmed) {
        error_raise(ERR_BIT_UNEXPECTED_SIGNAL);
    }
}

software_mode_t mode_next(software_mode_t current, const mode_inputs_t *in)
{
    if (in == NULL) return current;

    switch (current) {
    case MODE_SU:
        if (in->test_requested)   return MODE_T;
        if (in->startup_complete) return MODE_NG;
        return MODE_SU;

    case MODE_T:
        /* Testing only ends by passing. A failed self-test stays in
           MODE_T so the problem is in front of the operator rather
           than being carried into flight. */
        if (in->tests_passed) return MODE_NG;
        return MODE_T;

    case MODE_NG:
        if (in->flight_confirmed) return MODE_NF;
        return MODE_NG;

    case MODE_NF:
        if (in->soe) return MODE_NE;
        return MODE_NF;

    case MODE_NE:
        /* Terminal. The experiment runs to power-off at T+600 s; there
           is no path back, because there is no way to un-run it. */
        return MODE_NE;

    default:
        /* Unreachable unless memory is corrupt. Failing to the safe
           ground mode is better than continuing in an unknown one. */
        error_raise(ERR_BIT_MODE_TRANSITION);
        return MODE_SU;
    }
}