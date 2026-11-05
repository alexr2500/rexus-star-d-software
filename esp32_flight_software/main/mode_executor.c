#include "mode_executor.h"
#include "protocol.h"
#include "sensors.h"
#include "error_manager.h"
#include "gpio_driver.h"
#include "spi_pi_link.h"
#include "rs422_interface.h"
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
#define PUMP_NL_PER_PULSE       15u      /* 0.015 uL, 1/16 microstep */

/* ---- injection profile ------------------------------------------------
 * Volume and duration are a pair: changing one without checking the
 * other against the hardware is how the syringe stalls mid-flight. The
 * three asserts below check the pair against the three real limits.
 *
 * SED CONFLICT, recorded here rather than resolved in code:
 *   P.3   "shall be able to mix 2mL of the phagocytic target"
 *   P.17  "shall mix the cells with up to 500uL of phagocytic target
 *          (a non-zero amount must be injected of at least 100 uL)"
 * FV-T02's pass criterion follows P.17 ("0-500 uL ... FAIL: any
 * parameter outside range"), so a 2 mL injection fails its own
 * verification as written. P.17 and FV-T02 need correcting to match
 * whichever volume is nominal.
 *
 * Candidate profiles, all at 1/16 microstepping on a 10 mm bore and a
 * 0.6 mm screw:
 *
 *   volume   time   flow        motor     verdict
 *   2000 uL  3 s    48 ml/min   1019 rpm  flow sensor is +-40 ml/min,
 *                                         and the motor will not turn
 *                                         that fast under load
 *   2000 uL  60 s   2 ml/min    42 rpm    fine, but too slow for the
 *                                         biology per the team
 *   500 uL   3 s    10 ml/min   212 rpm   fine on every count  <= ACTIVE
 *
 * The active pair is the only one that meets the 2-3 s injection the
 * biology needs AND stays inside the hardware. Switching to 2 mL at
 * this speed needs a wider-range flow sensor and a faster or
 * better-geared pump; the asserts will refuse to build until then.
 */
#define NE_DISPENSE_UL          500u     /* P.17; see the conflict above */
#define NE_DISPENSE_SECONDS     3u       /* biology: 2-3 s               */

#define NE_DISPENSE_PULSES      ((NE_DISPENSE_UL * 1000u) / PUMP_NL_PER_PULSE)
#define NE_DISPENSE_RATE_HZ     (NE_DISPENSE_PULSES / NE_DISPENSE_SECONDS)

/* Derived checks, in the units the limits are actually quoted in. */
#define NE_FLOW_ML_PER_MIN      ((NE_DISPENSE_UL * 60u) / (NE_DISPENSE_SECONDS * 1000u))
#define NE_MOTOR_RPM            ((NE_DISPENSE_RATE_HZ * 60u) / 3200u)  /* 3200 ustep/rev */

_Static_assert(NE_DISPENSE_UL <= 2500u,
               "dispense volume exceeds the 2.5 mL syringe capacity");

_Static_assert(NE_FLOW_ML_PER_MIN <= 40u,
               "injection exceeds the SLF3S-1300F +-40 ml/min full scale: "
               "flow would saturate during the one event it must measure");

_Static_assert(NE_MOTOR_RPM <= 400u,
               "injection needs a shaft speed a NEMA 8 will not hold under "
               "load; a stall loses steps silently and there is no encoder");

static uint32_t dispense_remaining;
static bool     stepper_activated;

/* ===================================================================== */

static software_mode_t last_mode = MODE_SU;
static uint8_t         poll_divider;

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
    poll_divider       = 0;
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
    /* The train runs in hardware. All this does each cycle is notice
       when it has finished and drop the driver current, so the motor is
       not held against a stopped screw for the rest of the flight. */
    if (gpio_driver_pump_busy()) {
        dispense_remaining = NE_DISPENSE_PULSES
                           - gpio_driver_pump_pulses_done();
        return;
    }

    if (dispense_remaining != 0u) {
        dispense_remaining = 0u;
        gpio_driver_pump_enable(false);
    }
}

/* ---- what goes to the Pi each cycle ----------------------------------- *
 * SED table 4.13: sensor transfer at 10 Hz, status exchange at 1 Hz. One
 * transaction carries one frame, so SENSOR_DATA goes out every cycle and
 * POLL takes its place once a second. The Pi needs both: the sensor rows
 * are the science CSV, the poll is how it learns the mode, the rates and
 * the mission clock.
 * ===================================================================== */

static uint8_t poll_divider;

/* Little-endian writers. Done by hand rather than memcpy of a struct so
   the wire layout is visible here and cannot be changed by a compiler
   padding decision. */
static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static void put_i16(uint8_t *p, int16_t v) { put_u16(p, (uint16_t)v); }

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

/* Mission time is SIGNED on the wire. Countdown time is negative --
   T-180 s is -180000 -- and the Pi takes abs() of it to name its CSV
   files. An unsigned field would turn every countdown stamp into a
   number near 4.29 billion. */
static void put_i32(uint8_t *p, int32_t v) { put_u32(p, (uint32_t)v); }

/* Fields the Pi unpacks as unsigned cannot carry SENSOR_RAW_INVALID:
   INT16_MIN would arrive as 32768, a perfectly plausible pressure. Zero
   is used instead, which no real reading produces. Per-sample validity
   is NOT in this frame -- the Pi uses status_error_flag. That is an ICD
   limitation worth recording, not a bug here. */
static uint16_t as_u16(int16_t v)
{
    return (v == SENSOR_RAW_INVALID) ? 0u : (uint16_t)v;
}

static void send_sensor_frame(const sensor_snapshot_t *s)
{
    uint8_t p[LEN_SENSOR_DATA];

    /* Order is fixed by SENSOR_DATA_FORMAT "<hHHhHHHhhhhhhhhI" in
       raspberry_pi_software/src/stard/links/messages.py. The two must be
       changed together or every row the Pi logs is nonsense. */
    put_i16(&p[0],  s->ext_bme_temp);
    put_u16(&p[2],  as_u16(s->ext_bme_press));
    put_u16(&p[4],  as_u16(s->ext_bme_hum));
    put_i16(&p[6],  s->int_bme_temp);
    put_u16(&p[8],  as_u16(s->int_bme_press));
    put_u16(&p[10], as_u16(s->int_bme_hum));
    put_u16(&p[12], as_u16(s->abp_pressure));
    put_i16(&p[14], s->slf3s_flow);
    put_i16(&p[16], s->pt100_temp);
    put_i16(&p[18], s->ax);
    put_i16(&p[20], s->ay);
    put_i16(&p[22], s->az);
    put_i16(&p[24], s->gx);
    put_i16(&p[26], s->gy);
    put_i16(&p[28], s->gz);
    put_u32(&p[30], error_flags_get());

    (void)spi_pi_link_load(MSG_SENSOR_DATA, p, (uint8_t)LEN_SENSOR_DATA);
}

/* The time reference the Pi stamps its files with. Derived, not stored:
   there is exactly one place it can be wrong this way. */
static time_ref_t current_time_ref(bool sods, bool flight)
{
    if (flight) return TIME_MISSION;
    if (sods)   return TIME_COUNTDOWN;
    return TIME_UNSYNCED;
}

static void send_poll(software_mode_t mode, camera_rate_t cam, log_rate_t log,
                      bool sods, bool soe, bool flight, int32_t t_ms)
{
    uint8_t p[LEN_POLL];
    memset(p, 0, sizeof p);

    p[POLL_OFF_MODE]     = (uint8_t)mode;
    p[POLL_OFF_CAM_RATE] = (uint8_t)cam;
    p[POLL_OFF_LOG_RATE] = (uint8_t)log;

    uint8_t flags = 0;
    if (sods)                        flags |= POLL_FLAG_SODS;
    if (soe)                         flags |= POLL_FLAG_SOE;
    if (rs422_LO_get())              flags |= POLL_FLAG_LO;
    if (flight)                      flags |= POLL_FLAG_FLIGHT;
    if (spi_pi_link_is_degraded())   flags |= POLL_FLAG_DEGRADED;

    flags |= (uint8_t)(((uint8_t)current_time_ref(sods, flight)
                        << POLL_TIMEREF_SHIFT) & POLL_TIMEREF_MASK);
    p[POLL_OFF_FLAGS] = flags;

    put_i32(&p[POLL_OFF_TIME_MS], t_ms);

    (void)spi_pi_link_load(MSG_POLL, p, (uint8_t)LEN_POLL);
}

static void broadcast(software_mode_t mode, camera_rate_t cam, log_rate_t log,
                      bool sods, bool soe, bool flight, int32_t t_ms)
{
    /* POLL once per second, SENSOR_DATA on the other nine cycles. */
    if (poll_divider == 0u) {
        send_poll(mode, cam, log, sods, soe, flight, t_ms);
    } else {
        sensor_snapshot_t s;
        sensors_get(&s);
        send_sensor_frame(&s);
    }

    poll_divider++;
    if ((uint32_t)poll_divider * SPI_TRANSACTION_PERIOD_MS >= POLL_PERIOD_MS) {
        poll_divider = 0u;
    }
}

/* ---- the executor itself ---------------------------------------------- */

void mode_executor_run(software_mode_t mode, bool sods, bool soe,
                       bool flight_confirmed)
{
    /* TODO: this is uptime, not mission time. Anchoring it so that the
       countdown runs negative and LO is t=0 is not implemented -- the
       ESP has no countdown reference yet. The wire field is signed and
       ready for it; the Pi already handles negatives. */
    const int32_t now_ms = (int32_t)(esp_timer_get_time() / 1000);
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
            gpio_driver_pump_direction(true);     /* forward = dispense */
            gpio_driver_pump_enable(true);
            if (gpio_driver_pump_run(NE_DISPENSE_PULSES,
                                     NE_DISPENSE_RATE_HZ) == ESP_OK) {
                dispense_remaining = NE_DISPENSE_PULSES;
                stepper_activated  = true;
            } else {
                /* The injection is the experiment. If the train will not
                   start, the ground has to know immediately. */
                gpio_driver_pump_enable(false);
                error_raise(ERR_BIT_FLUIDIC_FAULT);
            }
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
        gpio_driver_pump_abort();
        gpio_driver_pump_enable(false);
        dispense_remaining = 0;
    }

    broadcast(mode, cam, log, sods, soe, flight_confirmed, now_ms);
    last_mode = mode;
}