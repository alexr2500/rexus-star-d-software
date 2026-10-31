#include "rs422_interface.h"
#include "board_pins.h"
#include "error_manager.h"
#include "driver/gpio.h"

/* Consecutive agreeing samples before a change is believed. These are
   switch contacts on a long cable in a vibrating rocket, so a single
   sample will chatter. The ICD gives ~1 ms timeline accuracy, so a few
   milliseconds of debounce costs nothing meaningful - and the cost of
   a false SOE is a mode transition that should not have happened.
   TODO: tune against the actual rs422_receive() call rate. */
#define DEBOUNCE_SAMPLES 5

typedef struct {
    bool    stable;
    bool    candidate;
    uint8_t agree;
} debounced_t;

static debounced_t lo, soe, sods;

/* Asserted = current sinking = pin reads LOW. */
static bool read_asserted(int pin)
{
    return gpio_get_level(pin) == 0;
}

static void debounce(debounced_t *d, bool raw)
{
    if (raw == d->stable) {
        d->agree = 0;                  /* no pending change */
        return;
    }
    if (raw == d->candidate) {
        if (d->agree < DEBOUNCE_SAMPLES) { d->agree++; }
    } else {
        d->candidate = raw;
        d->agree = 1;
    }
    if (d->agree >= DEBOUNCE_SAMPLES) {
        d->stable = raw;
        d->agree  = 0;
    }
}


void rs422_init(void)
{
    const gpio_config_t in = {
        .pin_bit_mask = (1ULL << PIN_RXSM_LO)
                      | (1ULL << PIN_RXSM_SOE)
                      | (1ULL << PIN_RXSM_SODS),
        .mode         = GPIO_MODE_INPUT,
        /* Open-collector switches need a pull-up to define the
           unasserted level. The schematic has R_PULL_LO and
           R_PULL_SODS; enable the internal ones too so a missing or
           wrong-value external resistor cannot leave a pin floating. */
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
#if 0   /* TODO: enable once board_pins.h is filled */
    gpio_config(&in);
#else
    (void)in;
#endif

    lo   = (debounced_t){ false, false, 0 };
    soe  = (debounced_t){ false, false, 0 };
    sods = (debounced_t){ false, false, 0 };
}


void rs422_receive(void)
{
#if 0   /* TODO: enable with board_pins.h */
    debounce(&lo,   read_asserted(PIN_RXSM_LO));
    debounce(&soe,  read_asserted(PIN_RXSM_SOE));
    debounce(&sods, read_asserted(PIN_RXSM_SODS));
#else
    (void)read_asserted;
#endif

    /* telemetry_flags.md bit 15: "Example: SOE received before LO.
       Indicates wiring fault or noise." Latched - we want to know it
       happened even if the order later looks sane. */
    if (soe.stable && !lo.stable) {
        error_raise(ERR_BIT_UNEXPECTED_SIGNAL);
    }
}


bool rs422_LO_get(void)   { return lo.stable; }
bool rs422_SOE_get(void)  { return soe.stable; }
bool rs422_SODS_get(void) { return sods.stable; }