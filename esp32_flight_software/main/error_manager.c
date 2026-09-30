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
    /* 1. zero the word
       2. THEN consult esp_reset_reason() and raise WATCHDOG_RESET if it
          was a watchdog cause. Order matters: reversed, you erase the
          one fact that survives nothing else. */
}


void error_raise(uint8_t bit)
{
    /* Guard the index, then one atomic_fetch_or.
       No latch check — raising is always permitted. */
}


void error_clear(uint8_t bit)
{
    /* Guard the index.
       Consult ERR_LATCH_MASK FIRST and return early if the bit latches.
       Otherwise one atomic_fetch_and with the inverted bit.
       Do not log in here — see the ISR note below. */
}


bool error_is_set(uint8_t bit)
{
    /* One read, one shift, one mask. */
}


uint32_t error_flags_get(void)
{
    /* One line. This is what telemetry_update() calls. */
}


void error_update_pi_status(camera_status_t cam,
                            uint8_t ssd_free_gb,
                            bool pre_lo)
{
    /* Three raise-or-clear decisions, all three bits non-latching:
         bit 7  cam != CAM_OK
         bit 8  ssd below LOW_SSD_GB
         bit 9  ssd below LOW_SSD_FOR_FLIGHT_GB   AND pre_lo
       Watch the 0xFF sentinel — see the gotcha below. */
}