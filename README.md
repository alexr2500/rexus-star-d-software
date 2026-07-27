# STAR-D — REXUS 38 Flight Software

This repository contains the source code that runs on the STAR-D experiment. It handles sensor acquisition, imaging, metadata logging and telemetry downlink output. The code is split in two subsections, one for the ESP32 that runs on C, and the other for the RaspberryPi CM5 that runs on Python.

## Repository layout
| Folder | Contents |
|---|---|
| `esp32_flight_software/` | ESP32-S3 firmware (C, ESP-IDF) |
| `raspberry_pi_software/` | Pi CM5 imaging and storage (Python) |
| `common/` | Shared protocol definitions |
| `docs/` | CONOPS, interfaces, architecture |

## Characteristics

The software functions using a state machine corresponding to flight mission phases. States are activated by specific signals sent by the RXSM.