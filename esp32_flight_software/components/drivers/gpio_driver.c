#include "gpio_driver.h"
#include "board_pins.h"
#include "driver/gpio.h"

static bool heaters_on = false;

/* Driver enable pins on A4988/TMC2209 are active LOW. */
#define STEPPER_EN_ACTIVE_LEVEL   0
#define STEPPER_EN_INACTIVE_LEVEL 1


void gpio_driver_init(void)
{
    const gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_HEATER1_EN)
                      | (1ULL << PIN_HEATER2_EN)
                      | (1ULL << PIN_HEATER3_EN)
                      | (1ULL << PIN_EN_STEPPERS)
                      | (1ULL << PIN_DIR_M1),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    /* TODO: enable once board_pins.h is filled. With GPIO_NUM_NC the
       bit mask is meaningless and gpio_config would fail, so this is
       deliberately inert rather than wrong. */
#if 0
    gpio_config(&out);
#else
    (void)out;
#endif

    gpio_driver_heaters_set(false);
    gpio_driver_steppers_enable(false);
    gpio_driver_stepper_dir(true);
}


void gpio_driver_heaters_set(bool on)
{
    heaters_on = on;
#if 0
    const int level = on ? 1 : 0;
    gpio_set_level(PIN_HEATER1_EN, level);
    gpio_set_level(PIN_HEATER2_EN, level);
    gpio_set_level(PIN_HEATER3_EN, level);
#endif
}

bool gpio_driver_heaters_get(void) { return heaters_on; }


void gpio_driver_steppers_enable(bool enable)
{
#if 0
    gpio_set_level(PIN_EN_STEPPERS,
                   enable ? STEPPER_EN_ACTIVE_LEVEL
                          : STEPPER_EN_INACTIVE_LEVEL);
#else
    (void)enable;
#endif
}


void gpio_driver_stepper_dir(bool forward)
{
#if 0
    gpio_set_level(PIN_DIR_M1, forward ? 1 : 0);
#else
    (void)forward;
#endif
}