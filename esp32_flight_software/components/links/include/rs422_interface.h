#ifndef RS422_INTERFACE_H
#define RS422_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

/* The three REXUS service discretes: LO, SOE, SODS.
 *
 * The name is historical. These are NOT RS-422 -- they are open-collector
 * switch lines from the RXSM that sink current when active (REXUS ICD
 * table 2.2), wired straight to GPIO1, GPIO2 and GPIO14 on this board
 * revision. The RS-422 telemetry link is uart_tcu.h.
 *
 * LEVELS, NOT EVENTS. The ICD says the LO switch "stays active during the
 * flight", so LO is a condition that holds, not a pulse that arrives. The
 * getters below are pure reads with no side effects: calling one does not
 * consume anything, and any number of callers may read the same signal in
 * the same cycle. state_machine.c depends on that.
 */

/* Configures the three pins as inputs with internal pull-ups and primes
   the debounce history from the current pin states. */
void rs422_init(void);

/* Takes one sample of all three pins and advances the debounce filter.
   Call at a steady rate from the control loop. The getters report the
   filtered state, so they are only as fresh as the last call to this. */
void rs422_sample(void);

/* Debounced levels. True means the signal is ASSERTED. */
bool rs422_LO_get(void);
bool rs422_SOE_get(void);
bool rs422_SODS_get(void);

/* True once LO has been asserted at any point since boot, and never
   false again. Survives a dropout on the LO line; does not survive a
   reset, which is what ERR_BIT_WATCHDOG_RESET is for. */
bool rs422_LO_ever_asserted(void);

/* Raw, undebounced pin states, for the self-test and for ground
   diagnosis of a harness. Not for flight logic. */
uint8_t rs422_raw_levels(void);
#define RS422_RAW_LO   0x01u
#define RS422_RAW_SOE  0x02u
#define RS422_RAW_SODS 0x04u

#endif /* RS422_INTERFACE_H */