# UART Interface — ESP32 ↔ Raspberry Pi CM5

## Message catalogue

**Timing model:** the ESP32 is master. It initiates every exchange; the Pi
only ever replies. This gives deterministic link timing, removes any
possibility of contention, and makes a missing reply unambiguous evidence
that the Pi is unresponsive — which is the trigger for the degraded flag.
The cost is that Pi-originated information is bounded by the poll interval
(≤ 1 s), which is acceptable for all data carried on this link.

### ESP32 → Raspberry Pi

| Name | Purpose | When sent |
|---|---|---|
| `POLL` | Carries current mode, degraded flag and mission time; requests a status reply | 1 Hz, and immediately on mode transition |
| `COMMAND` | Forwards an uplink telecommand (WIPE / TEST) with a sequence number | On receipt of an uplink command |

### Raspberry Pi → ESP32

| Name | Purpose | When sent |
|---|---|---|
| `STATUS` | Camera health, SSD free space, and the sequence number + result of the last executed command | In reply to every `POLL` |
| Logging error | Shows if a logging error occured | In case of error |

### Message contents

**`POLL`**
| Field | Notes |
|---|---|
| Mode | SU / T / NG / NF / NE |
| Degraded flag | Boolean |
| Mission time | Milliseconds since LO; negative before LO |

**`COMMAND`**
| Field | Notes |
|---|---|
| Command ID | WIPE / TEST |
| Sequence number | Increments per command; makes execution idempotent |

**`STATUS`**
| Field | Notes |
|---|---|
| Camera status | OK / stalled / write-fail (feeds the telemetry camera byte) |
| SSD free space | Feeds the telemetry SSD field |
| Last command sequence | Echoes the sequence number the Pi last executed |
| Last command result | Success / failure / none |

## Frame format

| Offset | Field | Size | Notes |
|---|---|---|---|
| 0 | Sync | 2 B | `0xAA 0x55` — start-of-frame marker |
| 2 | Message ID | 1 B | See message ID table |
| 3 | Payload length | 1 B | Bytes of payload only |
| 4 | Payload | 0–255 B | Per-message, see layouts below |
| 4+N | CRC-16 | 2 B | CCITT, computed over ID + length + payload |

Multi-byte integers are **little-endian** throughout.

### 0x01 — POLL (ESP32 → Pi)

| Offset | Field | Type | Size | Notes |
|---|---|---|---|---|
| 0 | Mode | uint8 | 1 B | `software_mode_t` |
| 1 | Degraded flag | uint8 | 1 B | 0 = nominal, 1 = degraded |
| 2 | Mission time | int32 | 4 B | ms since LO, negative before LO |

*Payload length : 6 bytes*

### 0x02 — COMMAND (ESP32 → Pi)

| Offset | Field | Type | Size | Notes |
|---|---|---|---|---|
| 0 | Command flag | uint8 | 1 B | `command_t`, 1 = WIPE, 2 = TEST |
| 1 | Seq number | uint8 | 1 B | |

*Payload length : 2 bytes*

### 0x03 — STATUS (Pi → ESP32)

//TODO: SSD storage

| Offset | Field | Type | Size | Notes |
|---|---|---|---|---|
| 0 | Camera status | uint8 | 1 B | `camera_status_t` |
| 1 | SSD storage remaining | uint8 | 1 B | Shown in GB remaining |
| 2 | Command seq echo | uint8 | 1 B | |
| 3 | Command result | uint8 | 1 B |  |

*Payload length : 4 bytes*