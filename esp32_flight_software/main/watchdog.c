#include "watchdog.h"
#include "esp_task_wdt.h"
#include <stddef.h>

/* Long enough that a slow sensor cycle never trips it, short enough
   that a hung loop is caught well inside one microgravity phase. */
#define WDT_TIMEOUT_MS  8000

void watchdog_init(void)
{
    esp_task_wdt_config_t cfg = {
        .timeout_ms     = WDT_TIMEOUT_MS,
        .idle_core_mask = 0,         /* the idle tasks are not monitored:
                                        they are allowed to be idle */
        .trigger_panic  = true,      /* reset, do not just log */
    };

    /* Deliberately NOT ESP_ERROR_CHECK. Depending on sdkconfig the IDF
       may already have initialised the task watchdog at startup, in
       which case init returns ESP_ERR_INVALID_STATE and the correct
       response is to reconfigure it, not to abort the boot. */
    esp_err_t err = esp_task_wdt_init(&cfg);
    if (err == ESP_ERR_INVALID_STATE) {
        (void)esp_task_wdt_reconfigure(&cfg);
    }
}

void watchdog_register_task(void)
{
    (void)esp_task_wdt_add(NULL);    /* NULL means the calling task */
}

void watchdog_feed(void)
{
    (void)esp_task_wdt_reset();
}