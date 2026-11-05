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

# ESP32 -> Pi handshake line: THERE ISN'T ONE.
#
# J3 on the flight board carries four wires -- CS, CLK, MOSI, MISO -- and
# no fifth pin for the ESP32 to say "my TX buffer is armed". The link
# therefore runs one cycle behind: the ESP loads a frame and the Pi
# reads it on the NEXT transaction, so what arrives is up to
# SPI_TRANSACTION_PERIOD_MS old.
#
# That is acceptable because nothing on the Pi closes a control loop --
# it logs and records. It is recorded here because it is a property of
# the board, not a choice, and because the sequence number in every
# frame is what detects a repeat rather than a handshake.
#
# Leave this None unless a handshake wire is actually added.
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

# Link health is defined in TIME, not in frame counts, and the counts
# are derived -- exactly as protocol.h does it. Changing the link rate
# then keeps the seconds correct on both sides automatically.
#
# SED v3-1 table 4.13 still quotes 3 and 5, which were the counts when
# the link ran at 1 Hz. At 10 Hz those would be 300 ms and 500 ms, and a
# Linux machine flushing video exceeds 300 ms while perfectly healthy.
PI_DEGRADED_SET_MS = 3000
PI_DEGRADED_CLEAR_MS = 5000

MISS_COUNT_DEGRADED = PI_DEGRADED_SET_MS // SPI_TRANSACTION_PERIOD_MS
SUCCESS_COUNT_CLEAR_DEGRADED = PI_DEGRADED_CLEAR_MS // SPI_TRANSACTION_PERIOD_MS

assert MISS_COUNT_DEGRADED >= 1 and SUCCESS_COUNT_CLEAR_DEGRADED >= 1, \
    "degraded thresholds rounded to zero: SPI period longer than the windows"
assert SUCCESS_COUNT_CLEAR_DEGRADED > MISS_COUNT_DEGRADED, \
    "clearing the degraded flag must be harder than setting it"


# ------------------------------------------- 9. Mode broadcast (POLL)
#
# Mirrors protocol.h section 11. The ESP32 tells the Pi what STATE it is
# in every second, rather than sending edge commands like "start
# recording": on a link with no handshake and no retransmission a missed
# command is lost forever, while a missed state broadcast is corrected
# on the next poll. The Pi reacts idempotently -- if the broadcast says
# the camera should be at the high rate and it is not, it changes it.


class CameraRate(IntEnum):
    OFF = 0         # not recording
    SLOW = 1        # ground / standby cadence
    HIGH = 2        # experiment cadence


class LogRate(IntEnum):
    OFF = 0         # not logging
    NOMINAL = 1     # SED table 4.15 unified cadence
    HIGH = 2        # increased in flight, SED 4.8.2.1.3


# MSG_POLL payload layout, LEN_POLL = 8 bytes:
#   [0]     SoftwareMode
#   [1]     flag bits, below
#   [2]     CameraRate
#   [3]     LogRate
#   [4..7]  mission time, milliseconds, little-endian uint32
POLL_OFF_MODE = 0
POLL_OFF_FLAGS = 1
POLL_OFF_CAM_RATE = 2
POLL_OFF_LOG_RATE = 3
POLL_OFF_TIME_MS = 4

POLL_FLAG_SODS = 0x01       # data storage gate, T-180 s
POLL_FLAG_SOE = 0x02
POLL_FLAG_LO = 0x04
POLL_FLAG_FLIGHT = 0x08     # LO or launch detected by the IMU
POLL_FLAG_DEGRADED = 0x10   # the ESP32's own view of the link

# Bits 6-7 carry the TimeRef, so a scrubbed countdown and a real flight
# can be told apart in the CSV filenames. There is no spare byte: the
# payload is 8 and the timestamp needs 4.
POLL_TIMEREF_SHIFT = 6
POLL_TIMEREF_MASK = 0xC0

assert POLL_OFF_TIME_MS + 4 == LEN_POLL, \
    "POLL payload layout does not fill LEN_POLL exactly"
