#ifndef WATCHDOG_H
#define WATCHDOG_H

/* Task watchdog ownership.
 *
 * Two watchdogs exist on this chip and they catch different faults:
 *   Task WDT       a task stopped feeding - a loop that never exits, a
 *                  blocking call that never returns
 *   Interrupt WDT  interrupts disabled too long; configured in
 *                  menuconfig, not here, and not feedable - a hard
 *                  backstop
 *
 * Either resets the chip. error_manager_init() reads the reset reason at
 * boot and latches ERR_BIT_WATCHDOG_RESET, so a reboot in flight shows
 * up on the downlink instead of passing unnoticed.
 *
 * A reset is survivable by design: the RXSM signals are levels held for
 * the whole flight, so after a reboot the firmware re-reads three pins
 * and knows where it is in the mission. There is no history to lose.
 */

void watchdog_init(void);        /* once, early in app_main */
void watchdog_subscribe(void);   /* from each task that will feed */
void watchdog_feed(void);        /* top of that task's loop */

#endif /* WATCHDOG_H */