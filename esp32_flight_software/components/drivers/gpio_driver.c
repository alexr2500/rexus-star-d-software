#include "gpio_driver.h"
#include "board_pins.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"       /* esp_rom_delay_us */

#define PUMP_PULSE_WIDTH_US 3   /* TMC2209 needs >=100 ns; 3 us is generous
                                   and still far below any step interval. */

static bool    heaters_on = false;
static uint8_t stage_phase = 0;

/* Half-step sequence. One bit per coil, LSB = coil A.
   Single coils and pairs alternate, which is what makes it "half": the
   rotor settles between two full-step positions on the even entries. */
static const uint8_t STAGE_SEQ[8] = {
    0x1,  /* A        */
    0x3,  /* A + B    */
    0x2,  /*     B    */
    0x6,  /*     B + C*/
    0x4,  /*         C*/
    0xC,  /*         C + D */
    0x8,  /*             D */
    0x9,  /* A +         D */
};

static void stage_write(uint8_t mask)
{
    gpio_set_level(PIN_STAGE_IN1, (mask >> 0) & 1u);
    gpio_set_level(PIN_STAGE_IN2, (mask >> 1) & 1u);
    gpio_set_level(PIN_STAGE_IN3, (mask >> 2) & 1u);
    gpio_set_level(PIN_STAGE_IN4, (mask >> 3) & 1u);
}

void gpio_driver_init(void)
{
    const uint64_t out_mask =
          (1ULL << PIN_HEATER1_EN) | (1ULL << PIN_HEATER2_EN)
        | (1ULL << PIN_HEATER3_EN)
        | (1ULL << PIN_PUMP_STEP)  | (1ULL << PIN_PUMP_DIR)
        | (1ULL << PIN_PUMP_ENN)
        | (1ULL << PIN_STAGE_IN1)  | (1ULL << PIN_STAGE_IN2)
        | (1ULL << PIN_STAGE_IN3)  | (1ULL << PIN_STAGE_IN4);

    gpio_config_t cfg = {
        .pin_bit_mask = out_mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    /* Safe state first, before anything else can run. Order matters: the
       heaters go off before the motors get power, not after. */
    gpio_driver_heaters_set(false);
    gpio_driver_stage_release();
    gpio_set_level(PIN_PUMP_STEP, 0);
    gpio_set_level(PIN_PUMP_DIR,  0);
    gpio_driver_pump_enable(false);
}

/* ---------------------------------------------------------------- heaters */

void gpio_driver_heaters_set(bool on)
{
    const uint32_t level = on ? HEATER_ACTIVE_LEVEL : !HEATER_ACTIVE_LEVEL;
    gpio_set_level(PIN_HEATER1_EN, level);
    gpio_set_level(PIN_HEATER2_EN, level);
    gpio_set_level(PIN_HEATER3_EN, level);   /* two MOSFETs on this one */
    heaters_on = on;
}

bool gpio_driver_heaters_get(void) { return heaters_on; }

/* ------------------------------------------------------------ syringe pump */

void gpio_driver_pump_enable(bool enable)
{
    gpio_set_level(PIN_PUMP_ENN, enable ? PUMP_ENABLE_LEVEL
                                        : !PUMP_ENABLE_LEVEL);
}

void gpio_driver_pump_direction(bool forward)
{
    gpio_set_level(PIN_PUMP_DIR, forward ? 1 : 0);
}

void gpio_driver_pump_pulse(void)
{
    gpio_set_level(PIN_PUMP_STEP, 1);
    esp_rom_delay_us(PUMP_PULSE_WIDTH_US);
    gpio_set_level(PIN_PUMP_STEP, 0);
}

/* -------------------------------------------------------- microscope stage */

void gpio_driver_stage_step(int8_t dir)
{
    if (dir > 0)      stage_phase = (uint8_t)((stage_phase + 1u) & 7u);
    else if (dir < 0) stage_phase = (uint8_t)((stage_phase + 7u) & 7u);
    else              return;                  /* dir == 0: do nothing */

    stage_write(STAGE_SEQ[stage_phase]);
}

void gpio_driver_stage_release(void)
{
    stage_write(0x0);
    /* stage_phase is kept: the next step continues the sequence from
       where it stopped, which avoids a jolt on restart. */
}

uint8_t gpio_driver_stage_phase(void) { return stage_phase; }