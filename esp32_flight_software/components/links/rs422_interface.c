#include "rs422_interface.h"
#include "board_pins.h"
#include "driver/gpio.h"

/* Number of consecutive identical samples needed before the filtered
   level changes. At a 10 ms control-loop period this is a 50 ms filter:
   long enough to reject harness noise, short enough that LO is seen well
   inside the SED's timing budget. */
#define DEBOUNCE_SAMPLES 5

typedef struct {
    gpio_num_t pin;
    bool       level;     /* debounced, true = asserted */
    uint8_t    streak;    /* consecutive raw samples disagreeing with level */
} flag_t;

static flag_t flags[3];   /* [0] LO, [1] SOE, [2] SODS */

#define LO   0
#define SOE  1
#define SODS 2

static bool lo_ever = false;

/* Reads one pin and converts the electrical level into "asserted".
   The flags sink current when active, so asserted is LOW. The internal
   pull-up is what makes the inactive state read as HIGH -- on this board
   revision there is no external pull-up and no optocoupler, so if the
   pull-up is ever disabled these lines float and read as noise. */
static bool pin_asserted(gpio_num_t pin)
{
    return gpio_get_level(pin) == REXUS_FLAG_ACTIVE_LEVEL;
}

void rs422_init(void)
{
    flags[LO].pin   = PIN_LO_IN;
    flags[SOE].pin  = PIN_SOE_IN;
    flags[SODS].pin = PIN_SODS_IN;

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_LO_IN)
                      | (1ULL << PIN_SOE_IN)
                      | (1ULL << PIN_SODS_IN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    /* Prime the filter from the real pin states rather than from false.
       If the experiment is reset after lift-off -- which the watchdog can
       do -- LO is already asserted, and starting from false would spend
       DEBOUNCE_SAMPLES cycles claiming the rocket is on the pad. */
    for (int i = 0; i < 3; i++) {
        flags[i].level  = pin_asserted(flags[i].pin);
        flags[i].streak = 0;
    }
    lo_ever = flags[LO].level;
}

void rs422_sample(void)
{
    for (int i = 0; i < 3; i++) {
        const bool raw = pin_asserted(flags[i].pin);

        if (raw == flags[i].level) {
            flags[i].streak = 0;              /* agrees: nothing pending */
        } else if (++flags[i].streak >= DEBOUNCE_SAMPLES) {
            flags[i].level  = raw;            /* held long enough: accept */
            flags[i].streak = 0;
        }
    }

    if (flags[LO].level) lo_ever = true;
}

bool rs422_LO_get(void)   { return flags[LO].level;   }
bool rs422_SOE_get(void)  { return flags[SOE].level;  }
bool rs422_SODS_get(void) { return flags[SODS].level; }

bool rs422_LO_ever_asserted(void) { return lo_ever; }

uint8_t rs422_raw_levels(void)
{
    uint8_t m = 0;
    if (pin_asserted(PIN_LO_IN))   m |= RS422_RAW_LO;
    if (pin_asserted(PIN_SOE_IN))  m |= RS422_RAW_SOE;
    if (pin_asserted(PIN_SODS_IN)) m |= RS422_RAW_SODS;
    return m;
}