#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/* Shared ESP32 <-> Raspberry Pi <-> ground contract.
 *
 * This file has a twin: raspberry_pi_software/src/stard/links/protocol.py.
 * Every constant below that the Pi also uses must be changed in both files
 * in the same commit. There is no code generation between them; the only
 * thing keeping them in step is discipline.
 */

/* ===================================================================== *
 * 1. Framing
 * ===================================================================== */

#define PROTO_SYNC0          0xAA   /* alternating byte patterns, so a receiver
                                       that joins mid-stream can find a frame
                                       boundary without a length prefix */
#define PROTO_SYNC1          0x55
#define PROTO_OVERHEAD_BYTES 6      /* 2 sync + 1 ID + 1 length + 2 CRC */
#define PROTO_MAX_PAYLOAD    42     /* largest defined payload is 34 bytes;
                                       the ceiling is SPI_FRAME_LEN minus
                                       overhead, asserted at the bottom */

/* ===================================================================== *
 * 2. Message IDs
 * ===================================================================== */

typedef enum {
    MSG_POLL        = 0x01,   /* ESP32 -> Pi */
    MSG_COMMAND     = 0x02,   /* ESP32 -> Pi */
    MSG_STATUS      = 0x03,   /* Pi -> ESP32 */
    MSG_SENSOR_DATA = 0x04,   /* ESP32 -> Pi */
    MSG_IDLE        = 0x05    /* ESP32 -> Pi, nothing to say this cycle.
                                 The Pi clocks every cycle whether or not the
                                 ESP has new data, so there has to be a frame
                                 meaning "no content" that still carries a
                                 sequence number and a valid CRC. Without it
                                 a quiet cycle is indistinguishable from a
                                 dead link. */
} message_id_t;

/* ===================================================================== *
 * 3. Payload lengths
 *
 * Fixed per message type, so a receiver can reject a frame whose declared
 * length does not match its ID before parsing any of it.
 * ===================================================================== */

#define LEN_POLL         8
#define LEN_COMMAND      2
#define LEN_STATUS       4
#define LEN_SENSOR_DATA  34
#define LEN_IDLE         0

/* ===================================================================== *
 * 4. Enum catalogue
 *
 * Zero means "safe" or "unknown" in every enum here, so a zeroed buffer
 * or a dropped field never decodes as a confident wrong answer.
 * ===================================================================== */

typedef enum {
    TIME_UNSYNCED  = 0,
    TIME_COUNTDOWN = 1,
    TIME_MISSION   = 2
} time_ref_t;

typedef enum {
    MODE_SU = 0,
    MODE_T  = 1,
    MODE_NG = 2,
    MODE_NF = 3,
    MODE_NE = 4
} software_mode_t;

typedef enum {
    CAM_UNKNOWN    = 0,
    CAM_OK         = 1,
    CAM_STALL      = 2,
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

/* ===================================================================== *
 * 5. Sentinel values and error bits
 * ===================================================================== */

#define SSD_VALUE_UNKNOWN      0xFF
#define LOW_SSD_GB             20
#define LOW_SSD_FOR_FLIGHT_GB  100

/* Bit positions in the 32-bit error word. Descriptions and latching
   behaviour: main/telemetry_flags.md. The macro names here are the
   authoritative spelling -- the document must match these, not the
   other way round. */

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
/* Bits 21-31 reserved, transmitted as zero. error_manager.h asserts that
   nothing latches there. */

/* ===================================================================== *
 * 6. Sensor scale factors (multiply raw by scale to get engineering units)
 * ===================================================================== */

#define IMU_ACCEL_SCALE_MG     0.488f   /* +-16 g full scale  */
#define IMU_GYRO_SCALE_MDPS    70.0f    /* +-2000 dps         */
#define BME_TEMP_SCALE_C       0.01f
#define BME_PRESSURE_SCALE_PA  2.0f
#define BME_HUMIDITY_SCALE_RH  0.01f
#define PT100_TEMP_SCALE_C     0.01f
#define SLF3S_FLOW_SCALE       0.002f   /* 1/500, water calibration */

/* ===================================================================== *
 * 7. ESP32 <-> Pi SPI link
 *
 * The Pi is the SPI MASTER and the ESP32 the SLAVE (SED table 4.13),
 * because Linux spidev is master-only. The ESP32 is still the LOGICAL
 * master: it decides the content of every frame; the Pi only turns the
 * crank at a fixed rate.
 *
 * J3 carries four wires -- CS, CLK, MOSI, MISO -- and no handshake line,
 * so the ESP cannot tell the Pi "my buffer is armed". The link therefore
 * runs one cycle behind: what the ESP loads now is read on the NEXT
 * transaction. See spi_pi_link.h.
 * ===================================================================== */

#define SPI_FRAME_LEN             48        /* every transaction is exactly this
                                               long in both directions, whatever
                                               the payload, so neither side has
                                               to agree a length beforehand */
#define SPI_MODE                  0         /* CPOL = 0, CPHA = 0 */
#define SPI_CLOCK_HZ              1000000
#define SPI_TRANSACTION_PERIOD_MS 100       /* 10 Hz, set by the Pi */

/* ===================================================================== *
 * 8. TCU / RXSM serial link
 * ===================================================================== */

#define TCU_BAUD_RATE  115200

/* ===================================================================== *
 * 9. Link health
 *
 * The degraded flag is defined in TIME, not in frame counts. The original
 * design intent was "about 3 seconds of silence sets it, about 5 seconds
 * of good traffic clears it", chosen when the link ran at 1 Hz and the
 * counts therefore happened to be 3 and 5.
 *
 * The link now runs at 10 Hz. Keeping the counts at 3 and 5 would mean
 * 300 ms and 500 ms, and a Linux machine flushing video to disk will
 * exceed 300 ms while perfectly healthy -- so the old numbers would raise
 * a false degraded flag in normal operation.
 *
 * Deriving the counts from the period keeps the intent intact and makes
 * the link rate the only thing anyone has to change.
 *
 * ACTION: SED v3-1 table 4.13 still states "3 consecutive invalid or
 * stale frames, cleared after 5 valid frames". It must be updated to
 * quote the times (3 s / 5 s) rather than the counts, before EAR.
 * ===================================================================== */

#define PI_DEGRADED_SET_MS    3000   /* silence before the flag is raised  */
#define PI_DEGRADED_CLEAR_MS  5000   /* good traffic before it is lowered  */

#define MISS_COUNT_DEGRADED \
    (PI_DEGRADED_SET_MS / SPI_TRANSACTION_PERIOD_MS)            /* 30 at 10 Hz */
#define SUCCESS_COUNT_CLEAR_DEGRADED \
    (PI_DEGRADED_CLEAR_MS / SPI_TRANSACTION_PERIOD_MS)          /* 50 at 10 Hz */

/* How long a single transaction may be late before it counts as missing.
   Three periods: tolerant of one skipped cycle, not of two. */
#define PI_LINK_TIMEOUT_MS  (3 * SPI_TRANSACTION_PERIOD_MS)

#define SENSOR_DATA_PERIOD_MS  100   /* 10 Hz, matches the SPI rate */
#define POLL_PERIOD_MS         1000  /* 1 Hz status exchange */

/* ===================================================================== *
 * 10. Mode broadcast (MSG_POLL payload) and rate selectors
 *
 * The ESP tells the Pi what STATE it is in, every cycle, rather than
 * sending edge commands like "start recording". On a link with no
 * handshake and no retransmission, a missed command is lost forever,
 * whereas a missed state broadcast is corrected 100 ms later. Same
 * principle as the REXUS discretes: levels over edges on an unreliable
 * link.
 *
 * The Pi reacts to the broadcast idempotently: if the mode says the
 * camera should be at the high rate and it is not, it changes it.
 * MSG_COMMAND stays for genuine one-shot ground actions (WIPE, TEST).
 * ===================================================================== */

typedef enum {
    CAM_RATE_OFF  = 0,   /* not recording */
    CAM_RATE_SLOW = 1,   /* ground / standby cadence */
    CAM_RATE_HIGH = 2    /* experiment cadence */
} camera_rate_t;

typedef enum {
    LOG_RATE_OFF     = 0,   /* not logging */
    LOG_RATE_NOMINAL = 1,   /* SED table 4.15 unified cadence */
    LOG_RATE_HIGH    = 2    /* increased in flight, SED 4.8.2.1.3 */
} log_rate_t;

/* MSG_POLL payload layout, LEN_POLL = 8 bytes.
     [0]     software_mode_t
     [1]     flag bits, below
     [2]     camera_rate_t
     [3]     log_rate_t
     [4..7]  mission time, milliseconds, little-endian uint32
   The timestamp is the periodic sync of SED 4.8.2.2: the Pi has no
   independent mission reference and computes its own timestamps from
   this, so imaging stays correctly stamped through a link outage. */
#define POLL_OFF_MODE      0
#define POLL_OFF_FLAGS     1
#define POLL_OFF_CAM_RATE  2
#define POLL_OFF_LOG_RATE  3
#define POLL_OFF_TIME_MS   4

#define POLL_FLAG_SODS     0x01u   /* data storage gate, T-180 s */
#define POLL_FLAG_SOE      0x02u
#define POLL_FLAG_LO       0x04u
#define POLL_FLAG_FLIGHT   0x08u   /* LO or launch detected by the IMU */

_Static_assert(POLL_OFF_TIME_MS + 4 == LEN_POLL,
               "MSG_POLL payload layout does not fill LEN_POLL exactly");

/* ===================================================================== *
 * 11. Compile-time consistency checks
 *
 * Each of these has failed in a real edit at some point, or would have.
 * They cost nothing at run time.
 * ===================================================================== */

_Static_assert(LEN_SENSOR_DATA + PROTO_OVERHEAD_BYTES <= SPI_FRAME_LEN,
               "SENSOR_DATA no longer fits in one SPI transaction");

_Static_assert(PROTO_MAX_PAYLOAD + PROTO_OVERHEAD_BYTES <= SPI_FRAME_LEN,
               "PROTO_MAX_PAYLOAD exceeds what one SPI transaction can carry");

_Static_assert(LEN_SENSOR_DATA <= PROTO_MAX_PAYLOAD,
               "SENSOR_DATA is larger than the declared maximum payload");

_Static_assert(MISS_COUNT_DEGRADED >= 1 && SUCCESS_COUNT_CLEAR_DEGRADED >= 1,
               "degraded thresholds rounded down to zero: the SPI period is "
               "longer than the time windows, so the flag could never work");

_Static_assert(SUCCESS_COUNT_CLEAR_DEGRADED > MISS_COUNT_DEGRADED,
               "clearing the degraded flag must be harder than setting it, "
               "or the link will oscillate in and out of degraded");

_Static_assert(SENSOR_DATA_PERIOD_MS >= SPI_TRANSACTION_PERIOD_MS,
               "sensor frames are produced faster than the Pi can collect "
               "them: samples would be overwritten before being read");

#endif /* PROTOCOL_H */