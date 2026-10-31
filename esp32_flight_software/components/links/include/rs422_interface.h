#ifndef RS422_INTERFACE_H
#define RS422_INTERFACE_H

#include <stdbool.h>

/* The three RXSM discrete signals.
 *
 * ALL THREE ARE LEVELS, NOT EVENTS. REXUS ICD 2.2: "LO switches are
 * directly linked to the umbilical LO signal and therefore switches
 * immediately with the event of umbilical extraction and stays active
 * during the flight."
 *
 * So the getters have NO side effects and may be called any number of
 * times. The previous version of this file read-and-cleared LO, which
 * meant a single stray debug read swallowed lift-off permanently. It
 * also made a watchdog reboot unrecoverable - with levels, the firmware
 * re-reads three pins after a reset and knows exactly where it is.
 *
 * If a caller needs an EDGE, it compares against its own previous
 * value. That is the caller's business, not this interface's.
 *
 * ELECTRICALLY ACTIVE LOW. They are open-collector switches that sink
 * current on activation (ICD 2.2), so asserted reads 0 at the GPIO.
 * This module inverts; nothing above it should ever think about it.
 */

void rs422_init(void);

/* Sample and debounce all three. Call at a steady rate. */
void rs422_receive(void);

bool rs422_LO_get(void);
bool rs422_SOE_get(void);
bool rs422_SODS_get(void);

#endif /* RS422_INTERFACE_H */