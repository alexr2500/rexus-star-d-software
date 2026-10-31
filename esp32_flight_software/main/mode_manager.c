/* Owns the current operational mode. Decides nothing - the transition
 * table lives in state_machine.c. Keeping storage and policy in separate
 * files means there is exactly one place that can change the mode, and
 * exactly one place that defines when it may change.
 */

#include "mode_manager.h"
#include "error_manager.h"

/* MODE_SU is 0, so this is also what zero-initialisation would give -
 * stated explicitly because relying on that is how you end up with a
 * default nobody chose. */
static software_mode_t mode = MODE_SU;


software_mode_t software_mode_get(void)
{
    return mode;
}


void software_mode_set(software_mode_t new_mode)
{
    /* Cast to unsigned so a negative value cannot slip past the check.
       An enum's underlying type is implementation-defined; do not assume
       it is unsigned just because every enumerator is. */
    if ((unsigned)new_mode > (unsigned)MODE_NE) {
        error_raise(ERR_BIT_MODE_TRANSITION);
        return;                      /* keep the mode we were in */
    }

    if (new_mode == mode) {
        return;                      /* no change: nothing to do or log */
    }

    mode = new_mode;

    /* TODO(step 2): telemetry_set_mode(mode);
       This function is the single choke point for mode changes, so it is
       the only correct place to push the mode into the downlink packet.
       Blocked until telemetry.c is rewritten. */
}