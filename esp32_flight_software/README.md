# star-d-digital-twin

Software-in-the-Loop (SIL) / Digital Twin for STAR-D (RX38 REXUS)

Mission logic · Thermal control · Power budget · Telemetry · Fault injection

---

# ESP32 ↔ Raspberry Pi SPI Communication – Validation

## Overview

We successfully established a stable SPI communication link between:

- **Raspberry Pi (SPI Master)**
- **ESP32 (SPI Slave)**

The Raspberry Pi initiates transactions and sends command bytes.  
The ESP32 receives the command and returns a response byte.

This validates:

- Electrical wiring
- SPI mode configuration
- Chip Select handling
- Full-duplex transaction behavior
- Master/slave architecture consistency

This is the first verified hardware communication layer of the STAR-D digital twin.

---

# Hardware Architecture

## SPI Signal Mapping

| Signal | Raspberry Pi (Master) | ESP32 (Slave) |
|--------|-----------------------|---------------|
| MOSI   | GPIO10                | GPIO23        |
| MISO   | GPIO9                 | GPIO19        |
| SCLK   | GPIO11                | GPIO18        |
| CS     | GPIO8 (CE0)           | GPIO5         |
| GND    | GND                   | GND           |

SPI Mode: **Mode 0**  
Clock Speed: **100 kHz** (stable on breadboard)

---

# Communication Test

## Transaction Behavior

When the Raspberry Pi sends:

<img width="501" height="77" alt="Capture d’écran 2026-02-23 à 10 25 59" src="https://github.com/user-attachments/assets/5ce2a6ac-41b7-439e-86c9-7b339df065e3" />


<img width="413" height="319" alt="Capture d’écran 2026-02-23 à 10 25 44" src="https://github.com/user-attachments/assets/b4acd06a-6d41-4175-a122-f006ade18bc0" />

