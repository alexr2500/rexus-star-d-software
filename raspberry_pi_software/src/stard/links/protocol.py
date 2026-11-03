"""Shared ESP32 <-> Raspberry Pi contract.

Hand-mirrored from common/protocol/protocol.h. The two files must
change in the SAME COMMIT, always. There is no generator and no check
-- only this docstring and discipline.

TRANSPORT: SPI, Pi as master, ESP32 as slave.
The Pi owns SCLK and CS because Linux spidev is master-only. That is
electrical, not architectural: the Pi clocks the bus at a fixed rate
and decides nothing; the ESP32 chooses the content of every one of its
own frames. The message catalogue is unchanged from the UART design
for exactly that reason.
"""

from enum import IntEnum

# ---------------------------------------------------------------- 1. Framing

PROTO_SYNC0 = 0xAA          # alternating patterns: unlikely to occur
PROTO_SYNC1 = 0x55          # in sensor data
PROTO_OVERHEAD_BYTES = 6    # 2 sync + 1 ID + 1 length + 2 CRC
PROTO_MAX_PAYLOAD = 42      # SPI_FRAME_LEN - PROTO_OVERHEAD_BYTES

# CRC-16/CCITT-FALSE over [ID, length, payload]. Sync bytes are a
# framing marker, not data, and are excluded. Poly 0x1021, init 0xFFFF,
# no reflection, no final XOR. Check value "123456789" -> 0x29B1.

# ------------------------------------------------------- 2. SPI transport

# Every transaction is this many bytes in BOTH directions, always.
# Full duplex: the Pi shifts out STATUS while the ESP32 shifts out its
# frame. A short frame is zero-padded; FrameParser scans for sync bytes
# and discards the padding, so no length negotiation is needed.
SPI_FRAME_LEN = 48

assert 34 + PROTO_OVERHEAD_BYTES <= SPI_FRAME_LEN, \
    "SENSOR_DATA no longer fits one SPI transaction"

SPI_BUS = 0                 # /dev/spidev0.0
SPI_DEVICE = 0
SPI_MODE = 0                # CPOL=0, CPHA=0 -- must match the ESP32 exactly
SPI_CLOCK_HZ = 1_000_000

# ESP32 -> Pi handshake line. High means "my TX buffer is armed, you
# may clock". The Pi MUST wait on this before every transaction:
# clocking while the slave has nothing loaded shifts out whatever was
# last in its DMA buffer -- stale data with a valid CRC, which is worse
# than noise.
# TODO: BCM pin number, once the harness is defined.
SPI_HANDSHAKE_GPIO = None

# ------------------------------------------------------- 3. Message IDs


class MessageId(IntEnum):
    MSG_POLL = 0x01         # ESP32 -> Pi
    MSG_COMMAND = 0x02      # ESP32 -> Pi
    MSG_STATUS = 0x03       # Pi -> ESP32
    MSG_SENSOR_DATA = 0x04  # ESP32 -> Pi
    MSG_IDLE = 0x05         # either way: "alive, nothing to say"


# MSG_IDLE exists so "the ESP had nothing queued this slot" is
# distinguishable from "the ESP never loaded its buffer". Zero-padding
# would make those identical, and they are different faults.

# --------------------------------------------------- 4. Payload lengths

LEN_POLL = 8
LEN_COMMAND = 2
LEN_STATUS = 4
LEN_SENSOR_DATA = 34        # fixed, so a truncated payload is rejected
LEN_IDLE = 0

# ------------------------------------------------------ 5. Enum catalogue


class TimeRef(IntEnum):
    TIME_UNSYNCED = 0
    TIME_COUNTDOWN = 1
    TIME_MISSION = 2


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


# ------------------------------------------- 6. Sentinels and thresholds

SSD_VALUE_UNKNOWN = 0xFF    # NOT 255 GB. A third state.
LOW_SSD_GB = 20
LOW_SSD_FOR_FLIGHT_GB = 100

# See esp32_flight_software/main/telemetry_flags.md for descriptions
# and latching behaviour.

ERR_BIT_PT100 = 0
ERR_BIT_INT_BME = 1
ERR_BIT_EXT_BME = 2
ERR_BIT_SLF3S = 3
ERR_BIT_ABP = 4
ERR_BIT_LSM6DSM = 5
ERR_BIT_PI_DEGRADED = 6
ERR_BIT_CAM_STATUS = 7
ERR_BIT_LOW_SSD = 8
ERR_BIT_LOW_SSD_FOR_FLIGHT = 9
ERR_BIT_THERMAL_FAULT = 10
ERR_BIT_HEATER_FAULT = 11
ERR_BIT_FLUIDIC_FAULT = 12
ERR_BIT_TCU_UART_LINK = 13
ERR_BIT_RXSM_LINK = 14
ERR_BIT_UNEXPECTED_SIGNAL = 15
ERR_BIT_MODE_TRANSITION = 16
ERR_BIT_WATCHDOG_RESET = 17
ERR_BIT_CSV_WRITE = 18
ERR_BIT_CONFIG_LOAD = 19
ERR_BIT_VIDEO_SHUTDOWN_INCOMPLETE = 20
# Bits 21-31 reserved, transmitted as zero.

# --------------------------------------------- 7. Sensor scale factors

IMU_ACCEL_SCALE_MG = 0.488      # LSM6DSM at +/-16 g
IMU_GYRO_SCALE_MDPS = 70.0      # LSM6DSM at +/-2000 dps
BME_TEMP_SCALE_C = 0.01
BME_PRESSURE_SCALE_PA = 2.0
BME_HUMIDITY_SCALE_RH = 0.01
PT100_TEMP_SCALE_C = 0.01
SLF3S_FLOW_SCALE = 0.002

# ------------------------------------------------------------ 8. Timing

# The Pi clocks one transaction this often. It is the Pi's job to hold
# this rate; the ESP32 cannot enforce it as a slave.
SPI_TRANSACTION_PERIOD_MS = 100     # 10 Hz

# Slot allocation, decided by the ESP32: SENSOR_DATA in nine slots out
# of ten, POLL in the tenth. Same cadences the UART design had.
SENSOR_DATA_PERIOD_MS = 100
POLL_PERIOD_MS = 1000

# How long the ESP32 waits for a transaction before calling it a miss.
# Replaces REPLY_TIMEOUT_MS, which is meaningless on a full-duplex bus
# where the reply arrives in the same transaction as the request.
PI_LINK_TIMEOUT_MS = 300

# Link health, counted per TRANSACTION (10 Hz) not per poll (1 Hz).
# Counts raised to preserve the original time constants:
#     30 misses    = 3 s of silence   -> PI_DEGRADED
#     50 successes = 5 s of good link -> clear it
MISS_COUNT_DEGRADED = 30
SUCCESS_COUNT_CLEAR_DEGRADED = 50