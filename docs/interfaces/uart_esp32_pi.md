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
| Time reference | Tells the Pi if the ESP's clock is a legitimate mission timeline or unsynced |

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
| 6 | Time reference | uint8 | 1 B | `time_ref_t` |
| 7 | SODS active | uint8 | 1 B | `sods_active` |

*Payload length : 8 bytes*

### 0x02 — COMMAND (ESP32 → Pi)

| Offset | Field | Type | Size | Notes |
|---|---|---|---|---|
| 0 | Command flag | uint8 | 1 B | `command_t`, 1 = WIPE, 2 = TEST |
| 1 | Seq number | uint8 | 1 B | |

*Payload length : 2 bytes*

### 0x03 — STATUS (Pi → ESP32)

| Offset | Field | Type | Size | Notes |
|---|---|---|---|---|
| 0 | Camera status | uint8 | 1 B | `camera_status_t` |
| 1 | SSD storage remaining | uint8 | 1 B | Shown in GB remaining (rounded down) |
| 2 | Command seq echo | uint8 | 1 B | |
| 3 | Command result | uint8 | 1 B |  |

*Payload length : 4 bytes*

### 0x04 — SENSOR_DATA (ESP32 → Pi)

| Offset | Field | Type | Size | Scale | Notes |
|---|---|---|---|---|---|
| 0 | `EXT_BME_temp` | int16 | 2 B | 0.01 °C | −40…+85 °C |
| 2 | `EXT_BME_pressure` | uint16 | 2 B | 2 Pa | Below sensor range above ~9 km |
| 4 | `EXT_BME_humidity` | uint16 | 2 B | 0.01 %RH | 0…100 %RH |
| 6 | `INT_BME_temp` | int16 | 2 B | 0.01 °C | |
| 8 | `INT_BME_pressure` | uint16 | 2 B | 2 Pa | Nominal ~1013 hPa; decline indicates PV leak |
| 10 | `INT_BME_humidity` | uint16 | 2 B | 0.01 %RH | |
| 12 | `ABP_pressure` | uint16 | 2 B | TBD | **Part number needed** |
| 14 | `SLF3S_flow` | int16 | 2 B | 1/500 ml/min | Raw sensor word |
| 16 | `PT100_temp` | int16 | 2 B | 0.01 °C | MFC control temperature |
| 18 | `IMU_accel_x` | int16 | 2 B | 0.488 mg/LSB | ±16 g full scale |
| 20 | `IMU_accel_y` | int16 | 2 B | 0.488 mg/LSB | ±16 g full scale |
| 22 | `IMU_accel_z` | int16 | 2 B | 0.488 mg/LSB | ±16 g full scale |
| 24 | `IMU_gyro_x` | int16 | 2 B | 70 mdps/LSB (datasheet value) | ±2000 dps full scale |
| 26 | `IMU_gyro_y` | int16 | 2 B | 70 mdps/LSB | ±2000 dps full scale |
| 28 | `IMU_gyro_z` | int16 | 2 B | 70 mdps/LSB | ±2000 dps full scale |
| 30 | `status_error_flag` | uint32 | 4B | - | Bitfield; see [error flag catalogue](../../esp32_flight_software/main/telemetry_flags.md) |

Payload length: 34 bytes

## Timing and error handling

| Parameter | Value | Rationale |
|---|---|---|
| Baud rate | 115200 | 8N1. Matches the REXUS TCU link rate. |
| `SENSOR_DATA` period | 100 ms | 10 Hz, matches CSV logging cadence |
| `POLL` period | 1000 ms | 1 Hz, matches telemetry downlink cadence |
| Reply timeout | 50 ms | ~25× nominal round trip; absorbs Linux scheduling jitter |
| Misses to set degraded | 3 consecutive | Tolerates transient jitter; detects a dead Pi within 3 s |
| Successes to clear degraded | 5 consecutive | Hysteresis — prevents flag flapping on a marginal link |

## CRC algorithm

CRC-16/CCITT-FALSE
Polynomial:      0x1021
Initial value:   0xFFFF
Input reflected: no
Output reflected: no
Final XOR:       none