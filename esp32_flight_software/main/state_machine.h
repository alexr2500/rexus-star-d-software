#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdbool.h>
#include "protocol.h"

/* Everything the transition logic is allowed to look at.
 *
 * Gathering the inputs and deciding on them are separated deliberately:
 * mode_next() is a pure function of (current, inputs) with no hardware
 * underneath it, so every transition can be tested on a laptop by
 * filling this struct. The EAR verification table for "the experiment
 * started at the right moment" is literally a list of these.
 *
 * SODS is absent on purpose. Per SED 4.8.2.1.3 it starts data storage
 * inside NG and is switched off near the end of flight without changing
 * mode, so it is an action, not a transition input. It is read
 * elsewhere.
 */
typedef struct {
    bool startup_complete;   /* POST finished: see mode_next() SU case  */
    bool test_requested;     /* TEST telecommand from ground            */
    bool tests_passed;       /* result of the MODE_T health sequence    */
    bool flight_confirmed;   /* LO asserted OR launch detected by IMU   */
    bool soe;                /* SOE level, debounced                    */
} mode_inputs_t;

/* Reads the hardware and fills *out. Impure by design - this is the only
   part of the state machine that touches RS-422, the IMU or the uplink. */
void mode_inputs_collect(mode_inputs_t *out);

/* The transition table. Returns the mode to be in next; returns current
   when nothing applies. Never mutates anything, never acts - the caller
   applies the result with software_mode_set(). */
software_mode_t mode_next(software_mode_t current, const mode_inputs_t *in);

#endif /* STATE_MACHINE_H */