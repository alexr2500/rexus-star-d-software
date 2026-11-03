#ifndef HEALTH_MONITOR_H
#define HEALTH_MONITOR_H

#include <stdbool.h>
#include "sensors.h"

/* Turns readings into judgements. Owns no hardware and stores no
   readings: it is handed a snapshot and decides what is wrong with it.

   Limits are in RAW units, matching sensor_snapshot_t, so no float
   arithmetic runs in the health path. */

void health_monitor_init(void);

/* Evaluates one snapshot and latches whatever error bits apply.
   flight_confirmed suppresses the checks that only make sense on the
   ground -- ambient pressure above all, which legitimately collapses
   once the rocket is above the atmosphere. */
void health_monitor_update(const sensor_snapshot_t *s, bool flight_confirmed);

/* True when the IMU has produced the same reading for too long. A
   sensor that has stopped updating still returns a valid-looking
   number; only comparing samples over time catches it. */
bool imu_is_frozen(void);

#endif /* HEALTH_MONITOR_H */