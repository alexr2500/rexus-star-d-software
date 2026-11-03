#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/* Shared ESP32 <-> Raspberry Pi contract.
 *
 * Hand-mirrored by raspberry_pi_software/src/stard/links/protocol.py.
 * The two files must change in the SAME COMMIT, always. There is no
 * generator and no check - only this comment and discipline.
 *
 * TRANSPORT: SPI, ESP32 as slave, Pi as master.
 * The Pi owns SCLK and CS because Linux spidev is master-only. That is
 * electrical, not architectural: the Pi clocks the bus at a fixed rate
 * and decides nothing; the ESP32 chooses the content of every one of
 * its own frames. The message catalogue below is unchanged from the
 * UART design for exactly that reason.
 */

/* ===================================================================
 * 1. Framing
 * =================================================================== */

#define PROTO_SYNC0          0xAA   /* alternating patterns: unlikely  */
#define PROTO_SYNC1          0x55   /* to occur in sensor data         */
#define PROTO_OVERHEAD_BYTES 6      /* 2 sync + 1 ID + 1 length + 2 CRC */
#define PROTO_MAX_PAYLOAD    42     /* SPI_FRAME_LEN - PROTO_OVERHEAD_BYTES */

/* CRC-16/CCITT-FALSE over [ID, length, payload]. The sync bytes are a
   framing marker, not data, and are excluded. Poly 0x1021, init 0xFFFF,
   no reflection, no final XOR. Check value "123456789" -> 0x29B1. */

/* ===================================================================
 * 2. SPI transport                                            [NEW]
 * =================================================================== */

/* Every transaction is this many bytes in BOTH directions, always.
   Full duplex: the ESP32 shifts out its frame while the Pi shifts out
   STATUS. A frame shorter than this is zero-padded; the receiver's
   parser scans for sync bytes and discards the padding, so no length
   negotiation is needed.

   48 fits the largest real frame (SENSOR_DATA: 34 + 6 = 40) with
   headroom, and is 4-byte aligned for DMA. */
#define SPI_FRAME_LEN        48

/* A frame that cannot fit in one transaction cannot be sent at all.
   Better to break the build than to discover it on the bench. */
_Static_assert(LEN_SENSOR_DATA + PROTO_OVERHEAD_BYTES <= SPI_FRAME_LEN,
               "SENSOR_DATA no longer fits one SPI transaction");

/* SPI mode 0 (CPOL=0, CPHA=0). Both ends must agree exactly: a mode
   mismatch shifts every bit by one clock and produces garbage that
   occasionally still contains a valid-looking sync pair. */
#define SPI_MODE             0

/* Conservative for a first bring-up over whatever cable length the
   harness ends up being. Raise only after the link is proven.
   TODO: measure margin at the flight cable length. */
#define SPI_CLOCK_HZ         1000000

/* ESP32 -> Pi. High means "my TX buffer is armed, you may clock".
   Without it the Pi can clock while the slave has nothing loaded, and
   the ESP32 shifts out whatever was last in its DMA buffer: stale data
   with a valid CRC, which is worse than noise. The Pi MUST wait on
   this line before every transaction. */
/* (the GPIO number itself is board truth, not protocol) */

/* ===================================================================
 * 3. Message IDs
 * =================================================================== */

typedef enum {
    MSG_POLL        = 0x01,   /* ESP32 -> Pi */
    MSG_COMMAND     = 0x02,   /* ESP32 -> Pi */
    MSG_STATUS      = 0x03,   /* Pi -> ESP32 */
    MSG_SENSOR_DATA = 0x04,   /* ESP32 -> Pi */
    MSG_IDLE        = 0x05    /* either way: "alive, nothing to say" */
} message_id_t;

/* MSG_IDLE exists so "the ESP is running but had nothing queued this
   slot" is distinguishable from "the ESP never loaded its buffer".
   Zero-padding would make those two look identical, and the whole
   point of the handshake line is that they are different faults. */

/* ===================================================================
 * 4. Payload lengths
 * =================================================================== */

#define LEN_POLL        8
#define LEN_COMMAND     2
#define LEN_STATUS      4
#define LEN_SENSOR_DATA 34    /* fixed, so a receiver can reject a
                                 truncated or corrupted payload */
#define LEN_IDLE        0

/* ===================================================================
 * 5. Enum catalogue
 * =================================================================== */

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

/* ===================================================================
 * 6. Sentinels and thresholds
 * =================================================================== */

#define SSD_VALUE_UNKNOWN     0xFF   /* NOT 255 GB. A third state. */
#define LOW_SSD_GB            20
#define LOW_SSD_FOR_FLIGHT_GB 100

/* See telemetry_flags.md for descriptions and latching behaviour. */

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
/* Bits 21-31 reserved, transmitted as zero. */

/* ===================================================================
 * 7. Sensor scale factors (multiply raw by scale)
 * =================================================================== */

#define IMU_ACCEL_SCALE_MG    0.488f   /* LSM6DSM at +/-16 g    */
#define IMU_GYRO_SCALE_MDPS   70.0f    /* LSM6DSM at +/-2000 dps */
#define BME_TEMP_SCALE_C      0.01f
#define BME_PRESSURE_SCALE_PA 2.0f
#define BME_HUMIDITY_SCALE_RH 0.01f
#define PT100_TEMP_SCALE_C    0.01f
#define SLF3S_FLOW_SCALE      0.002f

/* ===================================================================
 * 8. Timing
 * =================================================================== */

/* --- RS-422 to the RXSM (unrelated to the Pi link) ------------------ */
#define TCU_BAUD_RATE 115200   /* REXUS ICD default, 8N1 */

/* --- Pi link -------------------------------------------------------- */

/* The Pi clocks one transaction this often. It is the Pi's job to hold
   this rate; the ESP32 cannot enforce it as a slave. */
#define SPI_TRANSACTION_PERIOD_MS 100   /* 10 Hz */

/* Slot allocation. The ESP32 sends SENSOR_DATA in nine slots out of
   ten and POLL in the tenth, giving the same 10 Hz / 1 Hz cadences the
   UART design had. COMMAND displaces a SENSOR_DATA slot when one is
   pending. */
#define SENSOR_DATA_PERIOD_MS 100
#define POLL_PERIOD_MS        1000

/* How long the ESP32 waits for a transaction before calling it a miss.
   Replaces REPLY_TIMEOUT_MS, which is meaningless on a full-duplex bus
   where the reply arrives in the same transaction as the request. */
#define PI_LINK_TIMEOUT_MS 300

/* Link health, now counted per TRANSACTION (10 Hz) rather than per
   poll (1 Hz). The counts were raised to preserve the original time
   constants - 3 missed transactions is 300 ms, which a Linux box
   writing video to an SSD will hit without being faulty.
       30 misses    = 3 s of silence   -> PI_DEGRADED
       50 successes = 5 s of good link -> clear it                    */
#define MISS_COUNT_DEGRADED          30
#define SUCCESS_COUNT_CLEAR_DEGRADED 50

#endif /* PROTOCOL_H */