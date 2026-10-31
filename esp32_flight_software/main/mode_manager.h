#ifndef MODE_MANAGER_H
#define MODE_MANAGER_H

#include "protocol.h"   /* software_mode_t lives here; the Pi decodes the
                           same values, so this file must not redefine it */

/* Owns the current operational mode and nothing else.
 *
 * The variable is static in the .c, so software_mode_set() is the only
 * way the mode can change anywhere in the firmware. That single choke
 * point is where mode changes get logged and pushed to telemetry, and
 * it is what makes "which line changed the mode?" answerable.
 *
 * This module decides nothing. Which transitions are legal is
 * state_machine's business; mode_manager only stores the result. Keeping
 * the transition table in exactly one file is the point.
 */

software_mode_t software_mode_get(void);

/* Stores the new mode. Values outside the enum are rejected and raise
   ERR_BIT_MODE_TRANSITION - a mode that does not exist must never
   become the mode we are in. */
void software_mode_set(software_mode_t mode);

#endif /* MODE_MANAGER_H */