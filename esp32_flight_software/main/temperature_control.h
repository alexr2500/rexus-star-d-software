#ifndef TEMPERATURE_CONTROL_H
#define TEMPERATURE_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/* Hysteresis control of the three heater enables, driven from the PT100.
 *
 * Hysteresis, not PID. The SED mentions PID in the thermal chapter, but
 * the hardware is three on/off MOSFET channels with no current control,
 * so there is nothing for a PID to modulate -- the only reachable duty
 * cycles are 0 and 1. The SED thermal section should be corrected to say
 * bang-bang with a 2 K deadband. */

void temperature_control_init(void);

/* One control step. temp_raw is in 0.01 C; temp_valid comes from the
   PT100's valid bit. Heaters go OFF whenever the temperature is not
   trustworthy: heating blind is the one failure that cooks the biology. */
void temperature_control_update(int16_t temp_raw, bool temp_valid);

/* Ground override via the HEAT telecommand. duration_s is mandatory and
   must be non-zero: an override with no end is the one thing the ground
   must not be able to ask for. Returns false if rejected. */
bool temperature_control_override(bool heaters_on, uint32_t duration_s);

/* True while an override is in force. */
bool temperature_control_override_active(void);

#endif /* TEMPERATURE_CONTROL_H */