#include "temperature_control.h"
#include "error_manager.h"
#include "protocol.h"
#include "driver/gpio.h"
#include "esp_timer.h"

/* --- board wiring ----------------------------------------------------
 * TODO: confirm against Main_PCB_STAR_D and move to a shared
 * board_pins.h. The schematic has THREE independent channels
 * (HEATER1_EN..HEATER3_EN, each with its own MOSFET Q_HEATERn and
 * connector J_HEATERn). Driven together here as one zone, because there
 * is only one HEATER_FAULT bit and no per-channel feedback. If they are
 * ever driven separately, the error word needs rethinking first. */
#define HEATER1_EN_GPIO  GPIO_NUM_NC   /* TODO */
#define HEATER2_EN_GPIO  GPIO_NUM_NC   /* TODO */
#define HEATER3_EN_GPIO  GPIO_NUM_NC   /* TODO */

/* --- control band, SED 4.6 ------------------------------------------- */
#define HEAT_ON_BELOW_RAW   2100   /* 21.00 degC */
#define HEAT_OFF_ABOVE_RAW  2300   /* 23.00 degC */

/* --- heater fault detection ------------------------------------------
 * telemetry_flags.md bit 11 still reads "commanded >= X% duty for >= Y s
 * with no measurable dT/dt". With on/off control there is no duty cycle,
 * so it reduces to: commanded ON continuously for Y seconds without the
 * temperature rising by at least Z.
 *
 * TODO: both numbers are placeholders. Your thermal model gives roughly
 * 0.4 degC/min, which would be ~0.8 degC in 120 s - so 0.5 degC leaves
 * margin against a slow but working heater. Replace with a measured
 * heater response before flight and update telemetry_flags.md. */
#define HEATER_FAULT_WINDOW_US  (120 * 1000000LL)
#define HEATER_FAULT_MIN_RISE   50     /* 0.50 degC in raw units */

static bool     heater_on          = false;
static bool     override_active    = false;
static bool     override_state     = false;
static int64_t  override_expires_us = 0;

/* Fault-detection state: when the heater was last switched on, and what
   the temperature was at that moment. */
static int64_t  on_since_us   = 0;
static int16_t  temp_at_on    = 0;
static bool     fault_window_armed = false;


static void heaters_write(bool on)
{
    /* TODO: enable once the GPIOs are known. Writing to GPIO_NUM_NC
       would be an error, so this is deliberately inert rather than
       wrong - the control logic above it is still exercised and
       testable. */
#if 0
    gpio_set_level(HEATER1_EN_GPIO, on ? 1 : 0);
    gpio_set_level(HEATER2_EN_GPIO, on ? 1 : 0);
    gpio_set_level(HEATER3_EN_GPIO, on ? 1 : 0);
#else
    (void)on;
#endif
}


void temperature_control_init(void)
{
    /* TODO: gpio_config() the three enables as outputs, then drive them
       LOW before anything else. A heater that comes up on because a pin
       floated during boot is the worst possible start. */
    heater_on       = false;
    override_active = false;
    heaters_write(false);
}


void temperature_control_override(bool on, uint32_t duration_s)
{
    override_active     = true;
    override_state      = on;
    override_expires_us = esp_timer_get_time() + (int64_t)duration_s * 1000000LL;
}


bool temperature_control_heater_is_on(void)
{
    return heater_on;
}


void temperature_control_update(const sensor_snapshot_t *s)
{
    const int64_t now = esp_timer_get_time();

    /* --- override, and its expiry ------------------------------------ */
    if (override_active) {
        if (now >= override_expires_us) {
            override_active = false;     /* fall through to normal control */
        } else {
            heater_on = override_state;
            heaters_write(heater_on);
            return;                      /* fault detection paused */
        }
    }

    /* --- no trustworthy reading -------------------------------------- */
    /* Heaters OFF. Controlling on an unknown temperature is how you cook
       the samples: the PTC bounds the outcome at 60 degC, but 60 degC is
       already far above the 25 degC biological ceiling. The PT100's own
       error bit reports why we stopped controlling. */
    if (!(s->valid & SENSOR_VALID_PT100)) {
        heater_on = false;
        heaters_write(false);
        fault_window_armed = false;
        return;
    }

    const int16_t t = s->pt100_temp;

    /* --- hysteresis --------------------------------------------------
     * Only the two thresholds act. Between 21 and 23 degC nothing
     * changes, which is the whole point: a single setpoint would chatter
     * the MOSFETs at the switching rate of the sensor noise. */
    if (!heater_on && t < HEAT_ON_BELOW_RAW) {
        heater_on          = true;
        on_since_us        = now;
        temp_at_on         = t;
        fault_window_armed = true;
    } else if (heater_on && t > HEAT_OFF_ABOVE_RAW) {
        heater_on          = false;
        fault_window_armed = false;
    }

    heaters_write(heater_on);

    /* --- heater fault ------------------------------------------------ */
    /* Latched (bit 11), so it is raised once and stays. Checked only
       while the heater has been commanded on continuously - a heater
       that keeps reaching its threshold and switching off is working. */
    if (heater_on && fault_window_armed
        && (now - on_since_us) >= HEATER_FAULT_WINDOW_US) {

        if ((t - temp_at_on) < HEATER_FAULT_MIN_RISE) {
            error_raise(ERR_BIT_HEATER_FAULT);
        }
        fault_window_armed = false;   /* judge once per on-period */
    }
}