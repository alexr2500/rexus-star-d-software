#include "mode_manager.h"
#include "telemetry.h"
#include "rs422_interface.h"
//TODO: add the includes that will handle ALL tasks that are not yet coded, ie sensors & temp

//ENTRY POINT FOR ALL ACTIONS THROUGHOUT MISSION. Executes at each iteration from the main.c loop

void mode_execute(software_mode_t mode)
{
    //TODO: add the functions that are run in each mode
    switch(mode) {
        case MODE_SU:           //SOFTWARE STARTUP
            telemetry_init();
            break;
        case MODE_T:
            break;
        case MODE_NG:
            telemetry_task();
            if (rs442_LO_get() == true)
            {
                mode = MODE_NF;
            }
            break;
        case MODE_NF:
            telemetry_task();
            if (rs422_SOE_get() == true)
            {
                mode = MODE_NE;
            }
            break;
        case MODE_NE:
            telemetry_task();
            if (rs422_SOE_get() == false)
            {
                mode = MODE_NF;
            }
            break;
        default:
            mode = MODE_NF;
            break;
    }
}