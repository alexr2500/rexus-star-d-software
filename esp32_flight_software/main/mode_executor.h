#ifndef MODE_EXECUTOR_H
#define MODE_EXECUTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

/* What each mode DOES, as opposed to when it changes.
 *
 * The split across three modules is deliberate:
 *   state_machine  decides which mode we should be in   (pure)
 *   mode_manager   stores which mode we are in          (no policy)
 *   mode_executor  carries out that mode                (this file)
 *
 * SODS is handled here and not in the state machine. It is a signal,
 * not a mode: it gates data storage and video from about T-180 s and is
 * switched off late in the flight, without ever changing the mode.
 */

void mode_executor_init(void);

/* One step of whatever the current mode is supposed to be doing.
   Call once per control cycle, after the mode has been updated.
     sods / soe / flight_confirmed are the debounced levels.
   Builds and loads the MSG_POLL state broadcast for the Pi as its last
   action, so the Pi always sees the state that goes with this cycle. */
void mode_executor_run(software_mode_t mode,
                       bool sods,
                       bool soe,
                       bool flight_confirmed);

/* ---- MODE_T self-test ------------------------------------------------ *
 * Entering MODE_T starts the sequence; it runs over several cycles so
 * the sensors have time to produce fresh readings, then latches a
 * verdict. state_machine reads tests_passed to leave MODE_T.
 */

/* True once the sequence has reached a verdict. */
bool mode_executor_tests_complete(void);

/* The verdict. Only meaningful once complete. */
bool mode_executor_tests_passed(void);

/* Bitmask of which individual checks failed, for telemetry and for the
   ground to see WHICH test failed rather than just that one did. */
uint8_t mode_executor_test_failures(void);

#define TEST_FAIL_CRC        0x01u  /* CRC routine itself is wrong       */
#define TEST_FAIL_SENSORS    0x02u  /* a sensor is absent or incoherent  */
#define TEST_FAIL_CLOCK      0x04u  /* the monotonic clock is not moving */
#define TEST_FAIL_PI_LINK    0x08u  /* no STATUS from the Pi             */
#define TEST_FAIL_STORAGE    0x10u  /* Pi reports too little free space  */

/* ---- NE dispense ----------------------------------------------------- */

/* Microsteps still owed on the syringe. Zero before NE and once the
   dispense is finished. */
uint32_t mode_executor_dispense_remaining(void);

/* Latched true once the stepper has been commanded at all; mirrors the
   stepper_motor_activation field in the telemetry packet. */
bool mode_executor_stepper_activated(void);

#endif /* MODE_EXECUTOR_H */