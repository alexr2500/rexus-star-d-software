# STAR-D — REXUS 38 flight software

ESP32-S3 + Raspberry Pi CM5 (Python, picamera2).

## Constraints from CDR — do not violate
- ESP32 <-> Pi link is UART. Never SPI. Legacy SPI naming is a known bug.
- There is no FB/fallback mode. Degraded operation is a boolean flag inside NF/NE.
- Modes are SU, T, NG, NF, NE only.
- Camera health = SensorTimestamp advancing + file size growing.
- Read CONOPs v7 for concept of operations

## Rules
- Flight code must be fault-recoverable. No unhandled exceptions in the main loop.
- Do not invent protocol details. If docs/interfaces/uart_esp32_pi.md doesn't
  specify it, ask.
- Pi code must run on Windows against simulation/ fakes (no picamera2 locally).