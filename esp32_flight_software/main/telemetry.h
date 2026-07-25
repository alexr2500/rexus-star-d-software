#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "mode_manager.h"    /* software_mode_t — owned there, not modified here */
#include "camera_manager.h"  /* camera_status_t — TODO: adjust path to your actual camera module */
#include "storage_manager.h" /* ssd_status_t — TODO: adjust path to your actual storage module */

/* =====================================================================
 * STAR-D TELEMETRY PACKET DEFINITION
 * =====================================================================
 * Plain, byte-aligned packet — no bitfields, no bit-shifting anywhere.
 * Every field is one clearly-named variable, in the order it's sent.
 * This mirrors the framing style the REXUS User Manual itself suggests
 * (§7.7.2, "Downlink Protocol Example": SYNC / ID / COUNT / DATA /
 * CHECKSUM) — sync bytes, a message ID, a running counter, the
 * payload, then a checksum. Not CCSDS, and doesn't need to be: the
 * RS-422 link to the RXSM is fully transparent (REXUS User Manual
 * §7.7.2 — "the experiment teams are responsible for the formatting"),
 * so there's no compliance requirement pulling toward CCSDS, and a
 * plain layout is much easier to read, debug, and explain than
 * bit-packed fields would be.
 *
 * On the wire, one TelemetryPacket = one transmission, sent as raw
 * bytes over UART (nominally 1 Hz). The struct is packed, so its byte
 * layout IS the wire layout — no compiler padding to worry about.
 *
 * Byte order: native to the ESP32 target (little-endian). The
 * ground-station decoder MUST unpack using the same byte order
 * (e.g. Python struct format string starting with '<'). Record this
 * in the ICD.
 * ===================================================================== */

/* Sync bytes: a fixed, recognisable 2-byte marker at the start of
 * every packet, so the receiver can find where a packet begins in the
 * UART stream. 0xAA/0x55 (alternating bit pattern 10101010/01010101)
 * is a standard, widely-used choice in embedded serial protocols —
 * chosen because that bit pattern is very unlikely to occur by chance
 * in ordinary sensor data. */
#define TLM_SYNC1 0xAAu
#define TLM_SYNC2 0x55u

/* Message ID: identifies what kind of packet this is. Only one type
 * exists today, but the field means you're not locked in if you ever
 * want to add e.g. a separate event/log message later. */
#define TLM_MSG_ID_TELEMETRY 0x01u

/* Number of header bytes before the payload starts: sync1 + sync2 +
 * msg_id + seq_count + payload_length. Used only to document/compute
 * payload_length below — not needed anywhere else. */
#define TLM_HEADER_LEN 6u

/* ---------------------------------------------------------------------
 * Status types (camera_status_t, ssd_status_t) are NOT defined here.
 * They live in the modules that actually produce them — camera_manager.h
 * and storage_manager.h — same pattern as software_mode_t in
 * mode_manager.h. telemetry.h only needs the type names (for the
 * setter prototypes below); it never owns their meaning. This keeps
 * telemetry.h a pure wire-format definition, not a dumping ground for
 * every subsystem's enums as the project grows.
 *
 * Expected shape, to define in camera_manager.h:
 *   typedef enum {
 *       CAM_STATUS_NOT_DETECTED = 0,  // not enumerated by libcamera at boot/test
 *       CAM_STATUS_OK            = 1,  // SensorTimestamp advancing + last write confirmed
 *       CAM_STATUS_STALLED       = 2,  // detected, but SensorTimestamp not advancing
 *       CAM_STATUS_WRITE_FAIL    = 3   // frames captured, but storage write not confirmed
 *   } camera_status_t;
 *
 * Expected shape, to define in storage_manager.h:
 *   typedef enum {
 *       SSD_STATUS_OK        = 0,
 *       SSD_STATUS_LOW_SPACE = 1,
 *       SSD_STATUS_FULL      = 2,
 *       SSD_STATUS_ERROR     = 3
 *   } ssd_status_t;
 * ------------------------------------------------------------------- */

/* ---------------------------------------------------------------------
 * The packet itself. Packed (no padding) so the struct's byte layout
 * IS the wire layout. Fields are listed in the exact order they're
 * transmitted, top to bottom — read this struct top to bottom and
 * you're reading the packet.
 *
 * Analog sensor fields use fixed-point scaled integers instead of
 * float: sensor accuracy doesn't need float precision, and it halves
 * the byte cost of every value (4 bytes -> 2 bytes). Scale factors
 * marked TODO must be confirmed against each sensor's actual datasheet
 * resolution/range before flight.
 * ------------------------------------------------------------------- */
typedef struct __attribute__((packed)) {
    /* --- frame header (6 bytes) --- */
    uint8_t  sync1;                 /* fixed: TLM_SYNC1 */ 
    uint8_t  sync2;                 /* fixed: TLM_SYNC2 */
    uint8_t  msg_id;                /* TLM_MSG_ID_TELEMETRY */
    uint16_t seq_count;             /* increments every packet; wraps naturally at 65536 */
    uint8_t  payload_length;        /* bytes from timestamp_ms to error_flag, inclusive */

    /* --- payload: timing + mode --- */
    uint32_t timestamp_ms;          /* ms since onboard clock start (LO) */
    uint8_t  mode;                  /* cast from software_mode_t; kept 1 byte on the wire */

    /* --- payload: temperature (signed: can read below 0) --- */
    int16_t  pt100_temp;      /* MFC fluid temperature */
    int16_t  internal_temp;   /* BME280, internal */
    int16_t  external_temp;   /* BME280, external */

    /* --- payload: fluidic */
    int16_t  slf35_flow;
    int16_t  abp_pressure;

    /* --- payload: system health --- */
    uint16_t voltage_mV;                 /* main PCB voltage, millivolts (unsigned: never negative) */
    uint8_t  stepper_motor_activation;   /* 0/1, latched once confirmed after SOE */
    uint8_t  ssd_status;                 /* ssd_status_t */
    uint8_t  camera_status;              /* camera_status_t */
    uint32_t error_flag;                 /* bitmask, see error_manager.h enum */

    /* --- trailer --- */
    uint16_t crc;                        /* CRC-16/CCITT, see TLM_CRC_COVERAGE_LEN */
} TelemetryPacket;

/* Number of bytes to feed the CRC function: everything in the struct
 * EXCLUDING the trailing crc field. Computed from sizeof() so it can
 * never drift out of sync if fields are added/removed later. Always
 * use this instead of hand-deriving the length at the call site. */
#define TLM_CRC_COVERAGE_LEN ((uint16_t)(sizeof(TelemetryPacket) - sizeof(((TelemetryPacket*)0)->crc)))

/* Value for payload_length: everything after the 6-byte header, minus
 * the trailing crc field. Computed from sizeof() for the same reason. */
#define TLM_PAYLOAD_LENGTH ((uint8_t)(sizeof(TelemetryPacket) - TLM_HEADER_LEN - sizeof(((TelemetryPacket*)0)->crc)))

void telemetry_init(void);
void telemetry_task(void);

/* Controlled setters: keep each assignment type-safe at the call site
 * (enum in, uint8_t stored) without exposing the packet itself outside
 * this module — consistent with the existing "packet is only handled
 * by this file" encapsulation. Each takes its type from the module
 * that actually owns that status's meaning. */
void telemetry_set_mode(software_mode_t mode);
void telemetry_set_camera_status(camera_status_t status);
void telemetry_set_ssd_status(ssd_status_t status);

#endif