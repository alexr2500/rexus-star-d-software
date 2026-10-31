# Telemetry Packet Contents

This catalogue shows what data is sent through telemetry downlink to the ground station.

**Main characteristics :**
- Initial value `0xFFFF`
- CRC Polynomial `0x1021`
- Expected packet length : 32 bytes

| Data | Size | Section | Description |
| --- | --- | --- | --- |
| `SYNC1` | 1B | Header | First packet sync identifier |
| `SYNC2` | 1B | | Second packet sync identifier |
| `MSG_ID` | 1B | | Telemetry message type (only 1 exists) |
| `SEQ_COUNT` | 2B | | Packet counter, wraps at 65536 |
| `PAYLOAD_LENGTH` | 1B | | Length of packet contents, header and CRC aside |
| 