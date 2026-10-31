/* Mode transition table. Mirrors SED 4.8.2.1.2 (activation criteria) and
 * 4.8.2.1.3 (state machine). If this file and the SED ever disagree, one
 * of them is a bug - they are the same specification written twice.
 *
 * Split in two on purpose:
 *   mode_inputs_collect()  touches hardware, decides nothing
 *   mode_next()            decides everything, touches no hardware
 * so every transition can be exercised on a laptop by filling the struct.
 */

#include "state_machine.h"
#include "mode_manager.h"
#include "error_manager.h"
#include "rs422_interface.h"
#include "crc16.h"


/* --- input sources not yet implemented ------------------------------- */

/* TODO:(step 4): latched by the IMU once sustained launch acceleration is
   seen. Returning false gives today's behaviour - LO is the only way to
   confirm flight - so the stub fails safe rather than permissive. */
static bool launch_detected(void)
{
    return false;
}

/* TODO:(step 3/11): set by the TEST telecommand over the RS-422 uplink.
   No uplink parser exists yet, so MODE_T is currently unreachable. */
static bool test_telecommand_pending(void)
{
    return false;
}

/* TODO: the MODE_T health sequence (SED 4.8.2.1.3: hardware, sensors,
   power and clock, storage). Returns false until it exists, which is
   honest: no tests have passed because no tests have run. See the
   MODE_T case for why that does not strand the experiment. */
static bool health_tests_passed(void)
{
    return false;
}

/* Power-on self test. Grows as subsystems land; today the only POST that
   actually exists is the CRC check, and that one matters - a wrong CRC
   invalidates every frame and every telemetry packet. */
static bool startup_complete(void)
{
    /* TODO:(step 6/8): add sensor init and RS-422 configuration. */
    return crc16_self_test();
}


/* --- collect ---------------------------------------------------------- */

void mode_inputs_collect(mode_inputs_t *out)
{
    out->startup_complete = startup_complete();
    out->test_requested   = test_telecommand_pending();
    out->tests_passed     = health_tests_passed();

    /* Two independent routes to "we are flying". LO is one wire to one
       connector pin; launch_detected() is our own accelerometer. Either
       is sufficient, so a single broken wire no longer costs the flight.
       Both are level/latched, so this stays true once it becomes true. */
    out->flight_confirmed = rs422_LO_get() || launch_detected();

    out->soe = rs422_SOE_get();

    /* SODS deliberately absent: per SED 4.8.2.1.3 it starts data storage
       inside NG and is switched off near end of flight without changing
       mode. It is an action, not a transition input. */
}


/* --- decide ----------------------------------------------------------- */

software_mode_t mode_next(software_mode_t current, const mode_inputs_t *in)
{
    switch (current) {

    case MODE_SU:
        /* SED: "NG once the startup phase is complete". */
        if (in->startup_complete) {
            return MODE_NG;
        }
        return MODE_SU;

    case MODE_T:
        /* SED says what happens when tests pass and is silent on failure.
           Resolved as: leave T either way, and flag a failure.

           Staying in T on failure would guarantee mission loss, because
           NE is only reachable through NG and NF. NG is safe - no major
           operations - and ground holds the error word, so the GO/NO-GO
           decision belongs to the operators, not to the firmware. Never
           strand the vehicle in a non-operational state when an
           operational one is still safe. */
        if (!in->tests_passed) {
            /* TODO: allocate ERR_BIT_SELF_TEST_FAIL (bit 22 is reserved)
               in protocol.h and telemetry_flags.md, update the reserved
               -bits _Static_assert, and raise it here. Until then a
               failed self test leaves no trace on the downlink, which is
               the worst property of this file. */
        }
        return MODE_NG;

    case MODE_NG:
        /* Flight confirmation is checked here and only here. NE is
           reachable only through NF, so it inherits the guarantee - do
           not add a redundant check there. */
        if (in->flight_confirmed) {
            return MODE_NF;
        }
        /* Ground only. A health-test sequence during the experiment
           would be worse than useless, so the telecommand is refused
           once flight is confirmed - and the order of these two tests
           is what enforces that. */
        if (in->test_requested) {
            return MODE_T;
        }
        return MODE_NG;

    case MODE_NF:
        /* SED: "NE after the SOE signal is received". This is the
           transition that injects the sample. */
        if (in->soe) {
            return MODE_NE;
        }
        return MODE_NF;

    case MODE_NE:
        /* SED: "turns off when SOE is deactivated", then NF for the
           remainder of the flight and descent. */
        if (!in->soe) {
            return MODE_NF;
        }
        return MODE_NE;

    default:
        /* Unreachable in a correct program: reaching it means the mode
           variable has been corrupted. Return NG rather than the mode we
           think we are in - it performs no major operations, and because
           flight_confirmed latches, the NG -> NF -> NE path re-runs on
           the next tick and the state machine heals itself.

           The error_raise() here is the one impurity in this function.
           It is confined to a path that never executes in a correct
           program, and the alternative - losing the only evidence of
           corruption - is worse. */
        error_raise(ERR_BIT_MODE_TRANSITION);
        return MODE_NG;
    }
}