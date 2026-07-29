# All changes made to `protocol.py` must be correlated to the contents of `protocol.h` for the ESP32.

from enum import IntEnum

# 1. Framing constants

PROTO_SYNC0 = 0xAA
PROTO_SYNC1 = 0x55
PROTO_OVERHEAD_BYTES = 6
PROTO_MAX_PAYLOAD = 64


# 2. Message IDs

class MessageId(IntEnum):
    MSG_POLL = 0x01
    MSG_COMMAND = 0x02
    MSG_STATUS = 0x03
    MSG_SENSOR_DATA = 0x04


# 3. Payload lengths

LEN_POLL = 6
LEN_COMMAND = 2
LEN_STATUS = 4
LEN_SENSOR_DATA = 34


# 4. Enum catalogue

class SoftwareMode(IntEnum):
    MODE_SU = 0
    MODE_T = 1
    MODE_NG = 2
    MODE_NF = 3
    MODE_NE = 4

class CameraStatus(IntEnum):
    CAM_UNKNOWN = 0
    CAM_OK = 1
    CAM_STALL = 2
    CAM_WRITE_FAIL = 3

class Command(IntEnum):
    CMD_NONE = 0
    CMD_WIPE = 1
    CMD_TEST = 2

class CommandResult(IntEnum):
    RESULT_NONE = 0
    RESULT_PENDING = 1
    RESULT_OK = 2
    RESULT_FAILED = 3


# 5. Sentinel values

ERR_BIT_PT100               = 0
ERR_BIT_INT_BME             = 1
ERR_BIT_EXT_BME             = 2
ERR_BIT_SLF3S               = 3
ERR_BIT_ABP                 = 4
ERR_BIT_LSM6DSM             = 5
ERR_BIT_PI_DEGRADED         = 6
ERR_BIT_CAM_STATUS          = 7
ERR_BIT_LOW_SSD             = 8
ERR_BIT_LOW_SSD_FOR_FLIGHT  = 9
ERR_BIT_THERMAL_FAULT       = 10
ERR_BIT_HEATER_FAULT        = 11
ERR_BIT_FLUIDIC_FAULT       = 12
ERR_BIT_TCU_UART_LINK       = 13
ERR_BIT_RXSM_LINK           = 14
ERR_BIT_UNEXPECTED_SIGNAL   = 15
ERR_BIT_MODE_TRANSITION     = 16
ERR_BIT_WATCHDOG_RESET      = 17
ERR_BIT_CSV_WRITE           = 18
ERR_BIT_CONFIG_LOAD         = 19
# Bits 20-31 reserved, = 0


# 6. Sensor scale factors (multiply raw by scale)

IMU_ACCEL_SCALE_MG              = 0.488
IMU_GYRO_SCALE_MDPS             = 70
BME_TEMP_SCALE_C                = 0.01
BME_PRESSURE_SCALE_PA           = 2
BME_HUMIDITY_SCALE_RH           = 0.01
PT100_TEMP_SCALE_C              = 0.01
SLF3S_FLOW_SCALE                = 0.002


# 7. Timing constants

BAUD_RATE                       = 115200
SENSOR_DATA_PERIOD_MS           = 100
POLL_PERIOD_MS                  = 1000
REPLY_TIMEOUT_MS                = 50
MISS_COUNT_DEGRADED             = 3
SUCCESS_COUNT_CLEAR_DEGRADED    = 5