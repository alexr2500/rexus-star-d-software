#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdbool.h>

/* Task watchdog. If the control loop stops feeding it, the chip resets.
   error_manager reads the reset reason at boot and latches
   ERR_BIT_WATCHDOG_RESET, so a reset is visible on the ground rather
   than looking like a clean start. */

void watchdog_init(void);

/* Registers the calling task with the watchdog. Call once from each
   task that must be monitored. */
void watchdog_register_task(void);

/* Feeds the watchdog. Call from the bottom of the control loop, never
   from a timer: a timer would keep feeding it while the loop is stuck,
   which defeats the entire point. */
void watchdog_feed(void);

#endif /* WATCHDOG_H */