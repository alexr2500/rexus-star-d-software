#include "mode_executor.h"
#include "protocol.h"
#include "sensors.h"
#include "error_manager.h"
#include "gpio_driver.h"
#include "spi_pi_link.h"
#include "crc16.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include <string.h>

/* ===================================================================== *
 * MODE_T self test (SED 4.8.2.1.3)
 *
 * The SED lists four checks: hardware and interfaces, sensors active and
 * coherent, power and clock, and available storage. Everything below is
 * something the ESP can actually observe; nothing is asserted that we
 * cannot measure.
 *
 * It runs over several cycles rather than in one shot, because a single
 * cycle can catch a sensor mid-conversion and fail a healthy board.
 * ===================================================================== */
#define TEST_CYCLES  20      /* 2 s at the 10 Hz control rate */

static uint8_t  test_cycles_left;
static bool     test_running;
static bool     test_complete;
static bool     test_passed;
static uint8_t  test_failures;
static uint32_t test_start_ms;

/* ===================================================================== *
 * NE dispense
 * FIXME: check injection speeds and quantity
 * ===================================================================== */
#define PUMP_NL_PER_PULSE       15u      /* 0.015 uL, 1/16 microstep */ //FIXME:

/* FIXME: confirm the dispense volume and duration with the biology team.
   P.3 asks for 2 mL of phagocytic target to be mixed; whether that is
   the full NE dispense, and over how long, is not stated in the SED. */
#define NE_DISPENSE_UL          2000u    /* P.3: mix 2 mL              */
#define NE_DISPENSE_SECONDS     u      /* TODO: confirm speed/rate of injection ?             */

#define NE_DISPENSE_PULSES      ((NE_DISPENSE_UL * 1000u) / PUMP_NL_PER_PULSE)
#define PUMP_PULSES_PER_CYCLE   ((NE_DISPENSE_PULSES * SPI_TRANSACTION_PERIOD_MS) \
                                 / (NE_DISPENSE_SECONDS * 1000u))

/* Spacing inside a burst. 20 us is 50 kHz at the driver, far inside the
   TMC2209's limit, and the resulting shaft speed is under one turn per
   second. The whole burst blocks for PULSES_PER_CYCLE * 20 us, which is
   a few milliseconds of a 100 ms cycle. */
#define PUMP_PULSE_INTERVAL_US  20u

_Static_assert(PUMP_PULSES_PER_CYCLE > 0u,
               "dispense is too slow to emit a pulse each cycle");
_Static_assert((PUMP_PULSES_PER_CYCLE * PUMP_PULSE_INTERVAL_US) < 20000u,
               "the pump burst would block more than 20 ms of a cycle");

static uint32_t dispense_remaining;
static bool     stepper_activated;

/* ===================================================================== */

static software_mode_t last_mode = MODE_SU;

void mode_executor_init(void)
{
    test_cycles_left   = 0;
    test_running       = false;
    test_complete      = false;
    test_passed        = false;
    test_failures      = 0;
    test_start_ms      = 0;
    dispense_remaining = 0;
    stepper_activated  = false;
    last_mode          = MODE_SU;
}

/* ---- the self test ---------------------------------------------------- */

static void test_begin(void)
{
    test_running     = true;
    test_complete    = false;
    test_passed      = false;
    test_failures    = 0;
    test_cycles_left = TEST_CYCLES;
    test_start_ms    = (uint32_t)(esp_timer_get_time() / 1000);
}

static void test_step(void)
{
    sensor_snapshot_t s;
    sensors_get(&s);

    /* Hardware and interfaces: if the CRC routine is wrong, nothing that
       leaves this board can be trusted, so it is checked every pass. */
    if (!crc16_self_test()) test_failures |= TEST_FAIL_CRC;

    /* Sensors active and coherent. Every sensor that has a driver must
       be reporting valid data for the whole test window -- one dropout
       is a failure, because the point of the test is to catch the
       intermittent ones. */
    const uint16_t required = SENSOR_VALID_PT100 | SENSOR_VALID_INT_BME
                            | SENSOR_VALID_EXT_BME | SENSOR_VALID_SLF3S
                            | SENSOR_VALID_IMU | SENSOR_VALID_ABP;
    if ((s.valid & required) != required) test_failures |= TEST_FAIL_SENSORS;

    /* Clock. Mission timestamps depend on it, and a stopped monotonic
       clock is otherwise invisible. */
    if (s.timestamp_ms <= test_start_ms) test_failures |= TEST_FAIL_CLOCK;

    /* The Pi: both that it is talking, and that it has somewhere to
       write. Storage is the Pi's to report, so a degraded link fails
       the link check rather than silently passing the storage one. */
    if (spi_pi_link_is_degraded()) {
        test_failures |= TEST_FAIL_PI_LINK;
    } else if (error_is_set(ERR_BIT_LOW_SSD_FOR_FLIGHT)) {
        test_failures |= TEST_FAIL_STORAGE;
    }

    if (test_cycles_left > 0u) test_cycles_left--;

    if (test_cycles_left == 0u) {
        test_running  = false;
        test_complete = true;
        test_passed   = (test_failures == 0u);
    }
}

bool mode_executor_tests_complete(void)  { return test_complete; }
bool mode_executor_tests_passed(void)    { return test_passed; }
uint8_t mode_executor_test_failures(void){ return test_failures; }

/* ---- the pump --------------------------------------------------------- */

uint32_t mode_executor_dispense_remaining(void) { return dispense_remaining; }
bool     mode_executor_stepper_activated(void)  { return stepper_activated; }

static void dispense_step(void)
{
    if (dispense_remaining == 0u) {
        /* Finished: de-energise so the driver is not holding current
           against a stopped screw for the rest of the flight. */
        gpio_driver_pump_enable(false);
        return;
    }

    uint32_t n = PUMP_PULSES_PER_CYCLE;
    if (n > dispense_remaining) n = dispense_remaining;

    for (uint32_t i = 0; i < n; i++) {
        gpio_driver_pump_pulse();
        esp_rom_delay_us(PUMP_PULSE_INTERVAL_US);
    }
    dispense_remaining -= n;
}

/* ---- the state broadcast ---------------------------------------------- */

static void broadcast(software_mode_t mode, camera_rate_t cam, log_rate_t log,
                      bool sods, bool soe, bool flight, uint32_t t_ms)
{
    uint8_t p[LEN_POLL];
    memset(p, 0, sizeof p);

    p[POLL_OFF_MODE]     = (uint8_t)mode;
    p[POLL_OFF_CAM_RATE] = (uint8_t)cam;
    p[POLL_OFF_LOG_RATE] = (uint8_t)log;

    uint8_t flags = 0;
    if (sods)   flags |= POLL_FLAG_SODS;
    if (soe)    flags |= POLL_FLAG_SOE;
    if (flight) flags |= POLL_FLAG_FLIGHT;
    p[POLL_OFF_FLAGS] = flags;

    p[POLL_OFF_TIME_MS + 0] = (uint8_t)(t_ms & 0xFFu);
    p[POLL_OFF_TIME_MS + 1] = (uint8_t)((t_ms >> 8) & 0xFFu);
    p[POLL_OFF_TIME_MS + 2] = (uint8_t)((t_ms >> 16) & 0xFFu);
    p[POLL_OFF_TIME_MS + 3] = (uint8_t)((t_ms >> 24) & 0xFFu);

    (void)spi_pi_link_load(MSG_POLL, p, (uint8_t)LEN_POLL);
}

/* ---- the executor itself ---------------------------------------------- */

void mode_executor_run(software_mode_t mode, bool sods, bool soe,
                       bool flight_confirmed)
{
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    const bool entered = (mode != last_mode);

    /* Rates default to the safe end and are raised by the mode below.
       Writing them this way round means a mode that forgets to set them
       records nothing, rather than recording at the wrong rate. */
    camera_rate_t cam = CAM_RATE_OFF;
    log_rate_t    log = LOG_RATE_OFF;

    switch (mode) {

    case MODE_SU:
        /* Startup. The POST has already run in main before the loop
           started; nothing here moves any hardware, because the mode
           the board wakes up in must not actuate anything. */
        break;

    case MODE_T:
        /* Entered only by TEST telecommand. Runs the sequence, then
           state_machine takes us straight back to NG on the verdict.
           The camera and logging stay as they are in NG so a test does
           not interrupt a recording that SODS has already started. */
        if (entered) test_begin();
        if (test_running) test_step();

        cam = CAM_RATE_SLOW;
        log = sods ? LOG_RATE_NOMINAL : LOG_RATE_OFF;
        break;

    case MODE_NG:
        /* Normal ground. No major operations; temperature control is
           running in the main loop. Uplinks (WIPE, TEST, HEAT) are
           accepted -- that is handled by uart_tcu, which is polled
           every cycle regardless of mode.
           SODS, at about T-180 s, is what starts storage and video. */
        cam = sods ? CAM_RATE_SLOW : CAM_RATE_OFF;
        log = sods ? LOG_RATE_NOMINAL : LOG_RATE_OFF;
        break;

    case MODE_NF:
        /* In flight. SED 4.8.2.1.3 says the logging frequency is
           increased here; the camera stays at the slow rate until the
           experiment itself begins at SOE. */
        cam = sods ? CAM_RATE_SLOW : CAM_RATE_OFF;
        log = LOG_RATE_HIGH;
        break;

    case MODE_NE:
        /* The experiment. Camera to the high rate, logging high, and
           the syringe advances to push liquid through the chamber. */
        if (entered) {
            dispense_remaining = NE_DISPENSE_PULSES;
            gpio_driver_pump_direction(true);     /* forward = dispense */
            gpio_driver_pump_enable(true);
            stepper_activated = true;
        }
        dispense_step();

        cam = CAM_RATE_HIGH;
        log = LOG_RATE_HIGH;
        break;

    default:
        error_raise(ERR_BIT_MODE_TRANSITION);
        break;
    }

    /* Leaving NE for any reason stops the pump. There is no legal
       transition out of NE, so reaching this is already a fault -- but
       a fault must not leave the motor driving. */
    if (last_mode == MODE_NE && mode != MODE_NE) {
        gpio_driver_pump_enable(false);
        dispense_remaining = 0;
    }

    broadcast(mode, cam, log, sods, soe, flight_confirmed, now_ms);
    last_mode = mode;
}