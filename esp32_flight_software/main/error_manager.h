#ifndef ERROR_MANAGER_H
#define ERROR_MANAGER_H

#include <stdint.h>
                                                                        //ERROR FLAG BIT POSITION (defined as enum)
typedef enum {
    ERROR_NONE = 0,             //no error                              1
    ERROR_CAMERA_REC,           //camera                                2
    ERROR_SENSOR_FAIL,          //sensor issues                         3
    ERROR_SENSOR_INVALID,       //                                      4
    ERROR_GENTEMP_SENSOR_FAIL,  //temperature read and control          5
    ERROR_GENTEMP_CRITICAL,     //                                      6    
    ERROR_MFCTEMP_SENSOR_FAIL,  //                                      7
    ERROR_MFCTEMP_CONTROL_FAIL, //                                      8
    ERROR_MFCTEMP_CRITICAL,     //                                      9
    ERROR_SLF35_SENSOR_FAIL,    //Fluidic                               10
    ERROR_ABP_SENSOR_FAIL,      //                                      11
    ERROR_INTERFACE_RS422,      //interface issues                      12
    ERROR_INTERFACE_SPI,        //                                      13
    ERROR_INTERFACE_UART,       //                                      14
    ERROR_TELEM_FORMAT,         //telemetry issues                      15
    ERROR_TELEM_BUFFER,         //                                      16
    ERROR_MEMORY,               //memory                                17
    ERROR_DATA_CORRUPTION,      //                                      18
    ERROR_CPU                   //flight computer issues                19
} error_flag_t;

const char* error_to_string(error_flag_t error);
void raise_error(error_flag_t error);
void remove_error(error_flag_t error);
uint32_t get_error_flags(void);

#endif