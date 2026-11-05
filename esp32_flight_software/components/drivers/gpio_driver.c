#include "gpio_driver.h"
#include "board_pins.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"       /* esp_rom_delay_us */
#include "esp_timer.h"
#include "driver/ledc.h"
#include <stddef.h>

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

/* ---------------------------------------------- pump hardware pulse train */

/* LEDC generates a square wave on the STEP pin. Duty is fixed at 50%,
   which gives the TMC2209 a far wider pulse than its 100 ns minimum at
   any rate we use. 4-bit resolution is plenty for a 50% duty and keeps
   the maximum achievable frequency high. */
#define PUMP_LEDC_MODE       LEDC_LOW_SPEED_MODE
#define PUMP_LEDC_TIMER      LEDC_TIMER_0
#define PUMP_LEDC_CHANNEL    LEDC_CHANNEL_0
#define PUMP_LEDC_RES_BITS   LEDC_TIMER_4_BIT
#define PUMP_LEDC_DUTY_HALF  8u            /* 8/16 = 50% at 4-bit */

static esp_timer_handle_t pump_stop_timer;
static volatile bool      pump_running;
static uint32_t           pump_target_pulses;
static uint32_t           pump_rate_hz;
static int64_t            pump_started_us;

static void pump_stop_now(void)
{
    /* Idle level 0: STEP rests low between trains, which is what the
       driver expects. */
    (void)ledc_stop(PUMP_LEDC_MODE, PUMP_LEDC_CHANNEL, 0);
    pump_running = false;
}

static void pump_timer_cb(void *arg)
{
    (void)arg;
    pump_stop_now();
}

esp_err_t gpio_driver_pump_run(uint32_t pulses, uint32_t pulses_per_s)
{
    if (pulses == 0u || pulses_per_s == 0u) return ESP_ERR_INVALID_ARG;
    if (pump_running)                       return ESP_ERR_INVALID_STATE;

    ledc_timer_config_t tcfg = {
        .speed_mode      = PUMP_LEDC_MODE,
        .duty_resolution = PUMP_LEDC_RES_BITS,
        .timer_num       = PUMP_LEDC_TIMER,
        .freq_hz         = pulses_per_s,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&tcfg);
    if (err != ESP_OK) return err;

    ledc_channel_config_t ccfg = {
        .gpio_num   = PIN_PUMP_STEP,
        .speed_mode = PUMP_LEDC_MODE,
        .channel    = PUMP_LEDC_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = PUMP_LEDC_TIMER,
        .duty       = PUMP_LEDC_DUTY_HALF,
        .hpoint     = 0,
    };
    err = ledc_channel_config(&ccfg);
    if (err != ESP_OK) return err;

    if (pump_stop_timer == NULL) {
        const esp_timer_create_args_t targs = {
            .callback = pump_timer_cb,
            .arg      = NULL,
            .name     = "pump_stop",
        };
        err = esp_timer_create(&targs, &pump_stop_timer);
        if (err != ESP_OK) { pump_stop_now(); return err; }
    }

    /* Duration for exactly `pulses` cycles. Rounded up by half a period
       so the last pulse completes rather than being cut in half. */
    const uint64_t us = ((uint64_t)pulses * 1000000ULL) / pulses_per_s
                      + (500000ULL / pulses_per_s);

    pump_target_pulses = pulses;
    pump_rate_hz       = pulses_per_s;
    pump_started_us    = esp_timer_get_time();
    pump_running       = true;

    err = esp_timer_start_once(pump_stop_timer, us);
    if (err != ESP_OK) { pump_stop_now(); return err; }

    return ESP_OK;
}

bool gpio_driver_pump_busy(void) { return pump_running; }

void gpio_driver_pump_abort(void)
{
    if (pump_stop_timer != NULL) (void)esp_timer_stop(pump_stop_timer);
    pump_stop_now();
}

uint32_t gpio_driver_pump_pulses_done(void)
{
    if (pump_rate_hz == 0u) return 0u;

    const int64_t elapsed_us = esp_timer_get_time() - pump_started_us;
    if (elapsed_us <= 0) return 0u;

    const uint64_t done = ((uint64_t)elapsed_us * pump_rate_hz) / 1000000ULL;
    return (done > pump_target_pulses) ? pump_target_pulses : (uint32_t)done;
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