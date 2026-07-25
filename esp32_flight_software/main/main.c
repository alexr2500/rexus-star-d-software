#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "telemetry.h"
#include "mode_manager.h"
#include "error_manager.h"
#include "rs422_interface.h"
#include "state_machine.h"

void app_main(void)
{
    while (1)
    {
        rs422_receive();
        software_mode_t mode = software_mode_get();
        mode_execute(mode);
    }
}