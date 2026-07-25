#include <stdio.h>
#include "error_manager.h"

static uint32_t error_flag = 0;

const char* error_to_string(error_flag_t error)
{
    switch(error){
        case ERROR_NONE: return "OK"; break;
        case ERROR_CAMERA_REC: return "Camera recording error"; break;
        case ERROR_SENSOR_FAIL: return "Sensor read failed"; break;
        case ERROR_SENSOR_INVALID: return "Sensor value invalid/incoherent"; break;
        case ERROR_GENTEMP_SENSOR_FAIL: return "EM Temperature sensor failed"; break;
        case ERROR_GENTEMP_CRITICAL: return "EM Temperature is critical"; break;
        case ERROR_PVTEMP_SENSOR_FAIL: return "PV Temperature sensor failed"; break;
        case ERROR_PVTEMP_CONTROL_FAIL: return "PV Temperature control failed"; break;
        case ERROR_PVTEMP_CRITICAL: return "PV Temperature is critical"; break;
        case ERROR_INTERFACE_RS422: return "RS422 Interface issue"; break;
        case ERROR_INTERFACE_SPI: return "SPI Interface issue"; break;
        case ERROR_INTERFACE_UART: return "UART Interface issue"; break;
        case ERROR_TELEM_FORMAT: return "Telemetry packet formatting issue"; break;
        case ERROR_TELEM_BUFFER: return "Telemetry buffer issue"; break;
        case ERROR_MEMORY: return "Memory card issue"; break;
        case ERROR_DATA_CORRUPTION: return "Previously stored data has been overwritten"; break;
        case ERROR_CPU: return "CPU error"; break;
        default: "Undefined error type"; break;
    }
}

void raise_error(error_flag_t error)
{
    error_flag |= (1 << error);
}

void remove_error(error_flag_t error)
{
    error_flag &= ~(1 << error);
}

uint32_t get_error_flags(void)
{
    return error_flag;
}