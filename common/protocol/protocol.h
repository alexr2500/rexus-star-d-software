#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

// 1. Framing constants

#define PROTO_SYNC0          0xAA   //alternating byte patterns to sync wired payloads
#define PROTO_SYNC1          0x55
#define PROTO_OVERHEAD_BYTES 6      // 2 sync + 1 ID + 1 length + 2 CRC
#define PROTO_MAX_PAYLOAD    64     // Receive buffer size. Largest defined payload is 34 bytes


// 2. Message IDs         

typedef enum {
    MSG_POLL        = 0x01,   // ESP32 -> Pi
    MSG_COMMAND     = 0x02,   // ESP32 -> Pi
    MSG_STATUS      = 0x03,   // Pi -> ESP32
    MSG_SENSOR_DATA = 0x04    // ESP32 -> Pi
} message_id_t;


// 3. Payload lengths

#define LEN_POLL 7
#define LEN_COMMAND 2
#define LEN_STATUS 4
#define LEN_SENSOR_DATA 34      //fixed length constants so that receiver checks if payload is corrupted


// 4. Enum catalogue

typedef enum {
    TIME_UNSYNCED  = 0,
    TIME_COUNTDOWN = 1,
    TIME_MISSION   = 2
} time_ref_t;

typedef enum {
    MODE_SU = 0,
    MODE_T = 1,
    MODE_NG = 2,
    MODE_NF = 3,
    MODE_NE = 4
} software_mode_t;

typedef enum {
    CAM_UNKNOWN = 0,
    CAM_OK = 1,
    CAM_STALL = 2,
    CAM_WRITE_FAIL = 3
} camera_status_t;

typedef enum {
    CMD_NONE = 0,
    CMD_WIPE = 1,
    CMD_TEST = 2
} command_t;

typedef enum {
    RESULT_NONE    = 0,
    RESULT_PENDING = 1,
    RESULT_OK      = 2,
    RESULT_FAILED  = 3
} command_result_t;


// 5. Sentinel values

#define SSD_VALUE_UNKNOWN 0xFF

// See telemetry_flags.md for descriptions and latching behaviour

#define ERR_BIT_PT100                       0
#define ERR_BIT_INT_BME                     1
#define ERR_BIT_EXT_BME                     2
#define ERR_BIT_SLF3S                       3
#define ERR_BIT_ABP                         4
#define ERR_BIT_LSM6DSM                     5
#define ERR_BIT_PI_DEGRADED                 6
#define ERR_BIT_CAM_STATUS                  7
#define ERR_BIT_LOW_SSD                     8
#define ERR_BIT_LOW_SSD_FOR_FLIGHT          9
#define ERR_BIT_THERMAL_FAULT               10
#define ERR_BIT_HEATER_FAULT                11
#define ERR_BIT_FLUIDIC_FAULT               12
#define ERR_BIT_TCU_UART_LINK               13
#define ERR_BIT_RXSM_LINK                   14
#define ERR_BIT_UNEXPECTED_SIGNAL           15
#define ERR_BIT_MODE_TRANSITION             16
#define ERR_BIT_WATCHDOG_RESET              17
#define ERR_BIT_CSV_WRITE                   18
#define ERR_BIT_CONFIG_LOAD                 19
#define ERR_BIT_VIDEO_SHUTDOWN_INCOMPLETE   20
// Bits 21-31 reserved, transmitted as zero


// 6. Sensor scale factors (multiply raw by scale)

#define IMU_ACCEL_SCALE_MG 0.488f
#define IMU_GYRO_SCALE_MDPS 70.0f
#define BME_TEMP_SCALE_C 0.01f
#define BME_PRESSURE_SCALE_PA 2.0f
#define BME_HUMIDITY_SCALE_RH 0.01f
#define PT100_TEMP_SCALE_C 0.01f
#define SLF3S_FLOW_SCALE 0.002f


// 7. Timing constants

#define BAUD_RATE 115200
#define SENSOR_DATA_PERIOD_MS 100
#define POLL_PERIOD_MS 1000
#define REPLY_TIMEOUT_MS 50
#define MISS_COUNT_DEGRADED 3
#define SUCCESS_COUNT_CLEAR_DEGRADED 5


#endif //PROTOCOL_H