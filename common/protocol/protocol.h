#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

// 1. Framing constants

#define PROTO_SYNC0          0xAA
#define PROTO_SYNC1          0x55   //alternating byte patterns to sync wired payloads
#define PROTO_OVERHEAD_BYTES 6      // 2 sync + 1 ID + 1 length + 2 CRC
#define PROTO_MAX_PAYLOAD    64     // largest payload, serves as buffer size


// 2. Message IDs         

typedef enum {
    MSG_POLL        = 0x01,   // ESP32 -> Pi
    MSG_COMMAND     = 0x02,   // ESP32 -> Pi
    MSG_STATUS      = 0x03,   // Pi -> ESP32
    MSG_SENSOR_DATA = 0x04    // ESP32 -> Pi
} message_id_t;


// 3. Payload lengths

#define LEN_POLL 6
#define LEN_COMMAND 2
#define LEN_STATUS 4
#define LEN_SENSOR_DATA 30


// 4. Enum catalogue

typedef enum {
    MODE_SU = 0,
    MODE_T = 1,
    MODE_NG = 2,
    MODE_NF = 3,
    MODE_NE = 4
} software_mode_t;

typedef enum {
    CAM_UNKOWN = 0,
    CAM_OK = 1,
    CAM_STALL = 2,
    CAM_WRITE_FAIL = 3
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
} command_result_t;         //fixed length constants so that receiver checks if payload is corrupted


// 5. Sentinel values

#define SSD_VALUE_UNKOWN 0xFF
#define ERR_BIT_PI_DEGRADED     // bit position for degraded Pi flag in `status_error_t` //TODO: choose bit position


// 6. Sensor scale factors  

#define IMU_ACCEL_SCALE_MG 0.488f
#define IMU_GYRO_SCALE_MDPS 70.0f


// 7. Timing constants


#endif PROTOCOL_H