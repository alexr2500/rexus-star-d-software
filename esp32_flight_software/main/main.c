#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "protocol.h"
#include "error_manager.h"
#include "mode_manager.h"
#include "state_machine.h"
#include "sensors.h"
#include "health_monitor.h"
#include "temperature_control.h"
#include "telemetry.h"
#include "watchdog.h"
#include "mode_executor.h"

#include "crc16.h"
#include "spi_interface.h"
#include "spi_pi_link.h"
#include "rs422_interface.h"
#include "uart_tcu.h"
#include "gpio_driver.h"

static const char *TAG = "stard";

/* Control-loop period. Everything else is derived from it:
     rs422 debounce  = 5 cycles  = 50 ms
     sensor read     = every cycle (10 Hz, matches the Pi link)
     telemetry       = self-limited to 1 Hz inside telemetry_task()  */
#define LOOP_PERIOD_MS  (SPI_TRANSACTION_PERIOD_MS)

_Static_assert(LOOP_PERIOD_MS <= SPI_TRANSACTION_PERIOD_MS,
               "the control loop must run at least as fast as the Pi "
               "clocks, or armed frames go stale before they are read");

static bool post_passed;    /* power-on self test result */

/* Power-on self test. Runs before anything is allowed to move.
   Returns false only for faults that make the firmware itself
   untrustworthy; a dead sensor is reported, not fatal. */
static bool power_on_self_test(void)
{
    bool ok = true;

    /* If this fails, every frame and every telemetry packet is
       mis-checksummed, so nothing downstream can be believed. */
    if (!crc16_self_test()) {
        ESP_LOGE(TAG, "CRC self test FAILED");
        ok = false;
    }

    /* Sensors are reported, not fatal: flying with one dead sensor is
       better than not flying. sensors_init() has already latched the
       relevant error bits. */
    if (!sensors_init()) {
        ESP_LOGW(TAG, "one or more sensors did not initialise");
    }

    return ok;
}

void app_main(void)
{
    /* ---- order matters from here down ------------------------------- */

    /* 1. Error manager first: it reads the reset reason, which must be
          read before anything else can obscure why we rebooted. */
    error_manager_init();

    /* 2. Outputs to a safe state before any bus is touched. Heaters off,
          motors disabled. If the firmware dies after this line, it dies
          with the hardware safe. */
    gpio_driver_init();

    /* 3. Watchdog early, so a hang during the remaining init is caught
          rather than leaving the experiment wedged on the pad. */
    watchdog_init();
    watchdog_register_task();

    /* 4. Buses before the devices that sit on them. */
    if (spi_interface_init() != ESP_OK) {
        ESP_LOGE(TAG, "sensor SPI bus failed to initialise");
    }
    rs422_init();
    if (uart_tcu_init() != ESP_OK) {
        ESP_LOGE(TAG, "RS-422 link failed to initialise");
    }
    if (spi_pi_link_init() != ESP_OK) {
        ESP_LOGE(TAG, "Pi SPI link failed to initialise");
    }

    /* 5. Devices and the modules that judge them. */
    post_passed = power_on_self_test();
    health_monitor_init();
    temperature_control_init();
    mode_executor_init();
    telemetry_init();

    /* 6. Only now is a mode meaningful. */
    software_mode_set(MODE_SU);

    ESP_LOGI(TAG, "init complete, POST %s", post_passed ? "passed" : "FAILED");

    mode_inputs_t inputs = {0};
    inputs.startup_complete = post_passed;
    inputs.tests_passed     = false;

    for (;;) {
        /* --- inputs --------------------------------------------------- */
        sensors_read();
        uart_tcu_poll();
        spi_pi_link_service();

        /* mode_inputs_collect samples the discretes and leaves
           startup_complete and tests_passed as we set them. */
        mode_inputs_collect(&inputs);

        /* MODE_T latches its verdict in mode_executor; state_machine
           needs it to leave MODE_T and go back to NG. */
        inputs.tests_passed = mode_executor_tests_complete()
                           && mode_executor_tests_passed();

        /* --- decide --------------------------------------------------- */
        const software_mode_t current = software_mode_get();
        const software_mode_t next    = mode_next(current, &inputs);
        if (next != current) {
            software_mode_set(next);
            ESP_LOGI(TAG, "mode %d -> %d", (int)current, (int)next);
        }

        /* --- act ------------------------------------------------------ */
        sensor_snapshot_t snap;
        sensors_get(&snap);

        health_monitor_update(&snap, inputs.flight_confirmed);

        temperature_control_update(snap.pt100_temp,
                                   (snap.valid & SENSOR_VALID_PT100) != 0u);

        /* Ground HEAT override, if one arrived this cycle. */
        bool heat_on = false;
        uint32_t heat_secs = 0;
        if (uart_tcu_take_heat_request(&heat_on, &heat_secs)) {
            (void)temperature_control_override(heat_on, heat_secs);
        }

        /* What the mode actually DOES. SODS is read here rather than
           through mode_inputs_t: it gates data storage and video, and
           never changes the mode. */
        mode_executor_run(next,
                          rs422_SODS_get(),
                          inputs.soe,
                          inputs.flight_confirmed);

        /* --- outputs -------------------------------------------------- */
        camera_status_t  cam  = CAM_UNKNOWN;
        uint8_t          ssd  = SSD_VALUE_UNKNOWN;
        bool             prelo = false;
        command_result_t res  = RESULT_NONE;
        if (spi_pi_link_take_status(&cam, &ssd, &prelo, &res)) {
            error_update_pi_status(cam, ssd, prelo);
            telemetry_set_camera_status(cam);
            telemetry_set_ssd_free_gb(ssd);
        }
        telemetry_task();

        /* --- housekeeping --------------------------------------------- */
        watchdog_feed();
        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}