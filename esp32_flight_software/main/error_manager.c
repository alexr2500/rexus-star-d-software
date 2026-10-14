/* Central error/status word for the telemetry downlink.
 *
 * One uint32_t mirroring main/telemetry_flags.md. Written by the sensor
 * task, the thermal loop, the RS-422 handler and the Pi link; read by
 * telemetry. Latching is enforced here, not by callers.
 */

#include "error_manager.h"
#include "esp_system.h"      /* esp_reset_reason() */
#include <stdatomic.h>

static _Atomic uint32_t error_flags = 0;


void error_manager_init(void)
{
    /* Zero the word FIRST, then record the reset cause. Reversed, the
       zeroing would erase the one fact that survives a reboot. */
    error_flags = 0;

    /* Read once into a local: two calls can drift apart during a later
       edit, and this is the kind of condition nobody re-reads. */
    esp_reset_reason_t reason = esp_reset_reason();

    /* Three different watchdogs can reset this chip. TASK_WDT is a task
       that stopped feeding; INT_WDT is interrupts disabled for too long,
       the more serious of the two; WDT is the RTC watchdog. All three
       mean the same thing to ground, so they share bit 17.

       TODO: ESP_RST_PANIC (firmware crashed and rebooted) is at least as
       serious and has no bit of its own. Decide whether it joins this
       one or takes a reserved bit, and update telemetry_flags.md before
       changing anything here - the doc and this file must agree. */
    if (reason == ESP_RST_TASK_WDT ||
        reason == ESP_RST_INT_WDT  ||
        reason == ESP_RST_WDT) {
        error_raise(ERR_BIT_WATCHDOG_RESET);
    }
}


void error_raise(uint8_t bit)
{
    if (bit < 32) {
        error_flags |= (1u << bit);
    }
}


void error_clear(uint8_t bit)
{
    if (bit >= 32) {
        return;
    }

    /* Latching is structural here, not a rule callers have to remember.
       A bit in ERR_LATCH_MASK records that something happened, and a
       fault that occurred in flight must stay visible on the downlink
       after the condition itself goes away. */
    if (ERR_LATCH_MASK & (1u << bit)) {
        return;
    }

    error_flags &= ~(1u << bit);
}


bool error_is_set(uint8_t bit)
{
    if (bit >= 32) {
        return false;
    }
    return (error_flags & (1u << bit)) != 0;
}


uint32_t error_flags_get(void)
{
    /* One line. This is what telemetry_update() calls. */
    return error_flags;
}


void error_update_pi_status(camera_status_t cam,
                            uint8_t ssd_free_gb,
                            bool pre_LO)
{
    /* Bits 7, 8 and 9 mirror the Pi's last STATUS reply. All three are
       non-latching: they describe a current state the Pi can recover
       from. Called only when a reply actually arrived - after a missed
       poll the bits keep their last value and PI_DEGRADED (bit 6) tells
       ground they may be stale. */

    /* Bit 7 - camera. Anything other than CAM_OK raises it, CAM_UNKNOWN
       included: "we don't know whether the camera is recording" is a
       fault, not a neutral state. Deliberately handled before the SSD
       sentinel check below, so an unknown disk reading can never
       suppress a known camera fault. */
    if (cam != CAM_OK) {
        error_raise(ERR_BIT_CAM_STATUS);
    } else {
        error_clear(ERR_BIT_CAM_STATUS);
    }

    /* 0xFF is not "255 GB free", it is "no reading". It compares above
       both thresholds, so falling through to the tests below would clear
       bits 8 and 9 and report healthy storage at exactly the moment we
       know nothing about it. Leave both bits as they were. */
    if (ssd_free_gb == SSD_VALUE_UNKNOWN) {
        return;
    }

    /* Bit 8 - running low. Not latched: WIPE is expected to clear it. */
    if (ssd_free_gb < LOW_SSD_GB) {
        error_raise(ERR_BIT_LOW_SSD);
    } else {
        error_clear(ERR_BIT_LOW_SSD);
    }

    /* Bit 9 - not enough room for a full flight. A go/no-go indication
       that only means anything before lift-off; afterwards the question
       is settled and the bit clears. */
    if (pre_lo && ssd_free_gb < LOW_SSD_FOR_FLIGHT_GB) {
        error_raise(ERR_BIT_LOW_SSD_FOR_FLIGHT);
    } else {
        error_clear(ERR_BIT_LOW_SSD_FOR_FLIGHT);
    }
}