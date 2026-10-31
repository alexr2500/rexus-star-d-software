#ifndef HEALTH_MONITOR_H
#define HEALTH_MONITOR_H

#include <stdbool.h>
#include "sensors.h"

/* Turns a sensor snapshot into error-word bits 0-5 and 10.
 *
 * Limits come from main/telemetry_flags.md. If the two ever disagree,
 * the markdown is the specification and this file is the bug.
 *
 * flight_confirmed suppresses the external pressure range check: above
 * roughly 9 km the external BME280 is legitimately below its 300 hPa
 * floor, and flagging that would mark a working sensor faulty for most
 * of the flight. telemetry_flags.md bit 2 says exactly this.
 */
void health_monitor_check(const sensor_snapshot_t *s, bool flight_confirmed);

#endif /* HEALTH_MONITOR_H */