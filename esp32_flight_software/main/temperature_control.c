#include "temperature_control.h"
#include "protocol.h"
#include "error_manager.h"
#include "gpio_driver.h"
#include "esp_timer.h"

/* Setpoint band, 0.01 C per count. Heat below 21.00 C, stop above
   23.00 C. The 2 K gap is what stops the MOSFETs chattering at the
   setpoint; it sits inside the 18-25 C biological band with margin on
   both sides. */
#define HEAT_ON_BELOW_RAW   2100
#define HEAT_OFF_ABOVE_RAW  2300

/* If the heaters have been on this long without the temperature rising
   by at least HEATER_FAULT_MIN_RISE, something is wrong: a disconnected
   element, a failed MOSFET, or a PT100 reading a different place from
   the one being heated. */
#define HEATER_FAULT_WINDOW_US  (120 * 1000000LL)   /* 2 minutes */
#define HEATER_FAULT_MIN_RISE   50                  /* 0.50 C */

static bool     heaters_on;
static int64_t  heating_since_us;
static int16_t  temp_at_heat_start;
static bool     fault_latched;

static bool     override_active;
static bool     override_state;
static int64_t  override_until_us;

static void apply(bool on)
{
    if (on != heaters_on) {
        heaters_on = on;
        gpio_driver_heaters_set(on);
    }
}

void temperature_control_init(void)
{
    heaters_on         = true;      /* force apply(false) to actually write */
    override_active    = false;
    fault_latched      = false;
    heating_since_us   = 0;
    temp_at_heat_start = 0;
    apply(false);
}

bool temperature_control_override(bool heaters_on_req, uint32_t duration_s)
{
    if (duration_s == 0u || duration_s > 3600u) return false;

    override_active   = true;
    override_state    = heaters_on_req;
    override_until_us = esp_timer_get_time() + (int64_t)duration_s * 1000000LL;
    return true;
}

bool temperature_control_override_active(void) { return override_active; }

void temperature_control_update(int16_t temp_raw, bool temp_valid)
{
    const int64_t now = esp_timer_get_time();

    /* An expired override simply stops applying; the next lines put the
       hysteresis back in charge without any special case. */
    if (override_active && now >= override_until_us) {
        override_active = false;
    }

    /* Safety outranks the override. A ground operator asking for heat
       cannot overrule a dead temperature sensor. */
    if (!temp_valid) {
        apply(false);
        heating_since_us = 0;
        return;
    }

    if (override_active) {
        apply(override_state);
        heating_since_us = 0;      /* no fault detection while overridden */
        return;
    }

    if (temp_raw < HEAT_ON_BELOW_RAW) {
        if (!heaters_on) {
            heating_since_us   = now;
            temp_at_heat_start = temp_raw;
        }
        apply(true);
    } else if (temp_raw > HEAT_OFF_ABOVE_RAW) {
        apply(false);
        heating_since_us = 0;
    }
    /* Between the two thresholds: hold the current state. That gap is
       the hysteresis. */

    /* ---- heater effectiveness check ---------------------------------- */
    if (heaters_on && heating_since_us != 0
        && (now - heating_since_us) > HEATER_FAULT_WINDOW_US) {

        if ((int32_t)temp_raw - (int32_t)temp_at_heat_start < HEATER_FAULT_MIN_RISE) {
            if (!fault_latched) {
                error_raise(ERR_BIT_HEATER_FAULT);
                fault_latched = true;
            }
        }
        /* Restart the window either way, so a slow recovery is given
           another chance rather than being measured against a start
           point from several minutes ago. */
        heating_since_us   = now;
        temp_at_heat_start = temp_raw;
    }
}