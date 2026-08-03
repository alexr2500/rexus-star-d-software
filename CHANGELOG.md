# Changelog

## [SED v2-1] — 2026-07-15
### Changed
- Inter-processor link changed from SPI to UART
### Removed
- Fallback (FB) mode, replaced by degraded-operation flag

## [SED v3-0] — 2026-08-24
### *Upcoming*
### Added
- UART protocol definition (docs/interfaces/uart_esp32_pi.md)
- Shared protocol constants in C and Python
- Frame layer: CRC-16/CCITT, frame builder, stream parser (Python)