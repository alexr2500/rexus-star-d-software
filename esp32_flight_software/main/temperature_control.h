#ifndef TEMPERATURE_CONTROL_H
#define TEMPERATURE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "sensors.h"

/* Hysteresis control of the three heater channels.
 *
 * SED 4.6: "the loop holds 22 degC using the chip-adjacent sensor, with
 * the heaters below 21 degC and off above 23 degC." That is bang-bang
 * with a 2 degC deadband, not PID - the SED calls it PID in three other
 * places and the document needs correcting, not this file.
 *
 * Hysteresis is the right choice here: no tuning, no integrator windup,
 * a slow thermal mass, and a PTC element (FIT0845, 60 degC max) that is
 * self-limiting in hardware. If the ESP dies with the heaters on, the
 * physics bounds the outcome.
 */

void temperature_control_init(void);

/* Call at a steady rate with the latest snapshot. Does nothing useful
   if the control sensor's valid bit is clear - see the .c for why. */
void temperature_control_update(const sensor_snapshot_t *s);

/* HEAT telecommand (SED 4.9): forces the heaters on or off, bypassing
   the loop, for at most duration_s. The timeout is not optional - an
   override with no expiry on a heater inside a sealed vessel full of
   living cells is a safety hazard, not a convenience. */
void temperature_control_override(bool heater_on, uint32_t duration_s);

/* For telemetry and test. */
bool temperature_control_heater_is_on(void);

#endif /* TEMPERATURE_CONTROL_H */