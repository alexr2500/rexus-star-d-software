#include "watchdog.h"
#include "esp_task_wdt.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "wdt";

/* Longer than the slowest subscriber's worst case, short enough that a
   hang costs only a few telemetry packets. The 1 Hz telemetry loop is
   the slowest, so 5 s is 5x margin.
   TODO(step 6): revisit once the real task periods exist. */
#define WDT_TIMEOUT_MS 5000


void watchdog_init(void)
{
    esp_task_wdt_config_t cfg = {
        .timeout_ms = WDT_TIMEOUT_MS,
        /* Watch the idle task on every core. An idle task that never
           runs means something is spinning without yielding, which
           starves everything below it in priority. */
        .idle_core_mask = (1u << portNUM_PROCESSORS) - 1u,
        /* Panic rather than just logging: we want the reset, and we want
           esp_reset_reason() to say why it happened. */
        .trigger_panic = true,
    };

    /* sdkconfig may already have started the TWDT (CONFIG_ESP_TASK_WDT_INIT).
       Then init returns INVALID_STATE and reconfigure is correct. Handling
       both means this file does not silently depend on a menuconfig
       setting someone may change later. */
    esp_err_t err = esp_task_wdt_init(&cfg);
    if (err == ESP_ERR_INVALID_STATE) {
        err = esp_task_wdt_reconfigure(&cfg);
    }

    /* Deliberately not ESP_ERROR_CHECK. That aborts, and aborting at boot
       because a watchdog would not configure is a worse outcome than
       flying without one. Log and continue. */
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "task WDT setup failed: %s", esp_err_to_name(err));
    }
}


void watchdog_subscribe(void)
{
    esp_err_t err = esp_task_wdt_add(NULL);   /* NULL = calling task */
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WDT subscribe failed: %s", esp_err_to_name(err));
    }
}


void watchdog_feed(void)
{
    /* Call this from the top of the task loop, never from inside a
       sub-function. The point is to prove the loop is turning, and a
       feed three calls down can be reached by a task that is otherwise
       stuck. */
    esp_task_wdt_reset();
}