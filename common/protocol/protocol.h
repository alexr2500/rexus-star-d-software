#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

typedef enum {
    MODE_SU = 0,
    MODE_T = 1,
    MODE_NG = 2,
    MODE_NF = 3,
    MODE_NE = 4
} software_mode_t;

typedef enum {
    CAM_OK = 0,
    CAM_STALL = 1,
    CAM_WRITE_FAIL = 2
} camera_status_t;

typedef enum {
    CMD_NONE = 0,
    CMD_WIPE = 1,
    CMD_TEST = 2,
} command_t;

typedef enum {
    RESULT_NONE    = 0,
    RESULT_PENDING = 1,
    RESULT_OK      = 2,
    RESULT_FAILED  = 3
} command_result_t;

#endif PROTOCOL_H