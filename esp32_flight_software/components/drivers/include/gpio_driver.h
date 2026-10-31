#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include <stdbool.h>

/* Board digital outputs: heaters and stepper enables.
 *
 * Exists so no other module writes a raw pin number. temperature_control
 * decides WHEN the heaters are on; this decides WHICH pins that means.
 * One board revision then touches one file.
 *
 * The RXSM inputs are NOT here - they belong to rs422_interface, which
 * owns their debouncing and polarity.
 */

/* Configures every output and drives them all inactive. Call before
   anything else: a heater or a stepper that comes up enabled because a
   pin floated through boot is the worst possible start. */
void gpio_driver_init(void);

/* All three heater channels together. They share one HEATER_FAULT bit
   and there is no per-channel feedback, so treating them as one zone is
   the only thing the error word can currently express. */
void gpio_driver_heaters_set(bool on);
bool gpio_driver_heaters_get(void);

/* The A4988/TMC2209 enable is active LOW; this function takes the
   logical sense and inverts internally, so callers never think about it.

   De-energise when idle: the 28BYJ-48 runs hot, and this one sits in a
   temperature-controlled vessel next to PLA that softens around 60 degC.
   TODO: the de-energise-when-idle policy is still not in the CONOPS. */
void gpio_driver_steppers_enable(bool enable);
void gpio_driver_stepper_dir(bool forward);

#endif /* GPIO_DRIVER_H */