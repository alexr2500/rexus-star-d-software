#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

/* Drives everything on the board that is a plain output: the heaters and
   the two stepper motors. Holds no policy — temperature_control.c decides
   when heat is wanted, this decides which pins move. */

void gpio_driver_init(void);

/* ---- heaters: three enable lines, four physical channels, one zone ---- */
void gpio_driver_heaters_set(bool on);
bool gpio_driver_heaters_get(void);

/* ---- syringe pump, TMC2209 ------------------------------------------- */
void gpio_driver_pump_enable(bool enable);
void gpio_driver_pump_direction(bool forward);
/* One STEP pulse. Blocks for PUMP_PULSE_WIDTH_US only (~2 us), never for
   the step interval: the caller spaces the pulses, so a long move cannot
   stall the control loop. */
void gpio_driver_pump_pulse(void);

/* ---- microscope stage, ULN2003 --------------------------------------- *
 * Half-step sequence, 8 states. dir = +1 or -1; any other value is
 * ignored. Advances one half-step per call and returns immediately.    */
void gpio_driver_stage_step(int8_t dir);

/* De-energise all four coils. Call this whenever the stage is not moving:
   a ULN2003 has no enable pin, so a coil left high burns current and heat
   for as long as it stays high. */
void gpio_driver_stage_release(void);

/* Where the sequence currently sits (0-7). Position is not tracked —
   the stage has no encoder and no limit switch on this board. */
uint8_t gpio_driver_stage_phase(void);

#endif /* GPIO_DRIVER_H */