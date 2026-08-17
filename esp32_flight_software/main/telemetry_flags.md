# Telemetry downlink status/error flags catalogue

This catalogue shows what each bit of the `status_error_flag` *(uint32)* variable represents for real-time ground station flight systems analysis.

//TODO: fill in X/Y fluidic values

**Latching:** a latched bit stays set once triggered and clears only on power cycle or reset. A non-latched bit reflects the current condition and clears when it resolves.

**Pi-sourced bits:** bits 7, 8, 9 and 18 originate on the Raspberry Pi and cross the UART link. They are meaningless while bit 6 (Pi degraded) is set — the ESP32 reports last-known values, which may be stale.

| Bit | Field | Latched | Description |
| --- | --- | --- | --- |
| 0 | PT100_ERROR | Yes | I²C/SPI read failed, or value outside −40…+85 °C. *Serious enough to set for ground station to analyze* |
| 1 | INT_BME_ERROR | Yes | I²C read failed, or value outside 300–1100 hPa / −40…+85 °C |
| 2 | EXT_BME_ERROR | Yes | I²C read failed, or value outside sensor range. *Expect out-of-range pressure above ~9 km; do not flag on altitude alone* |
| 3 | SLF3S_ERROR | Yes | I²C read failed or CRC mismatch on the sensor word |
| 4 | ABP_ERROR | Yes | Read failed, or value outside the part's rated pressure range |
| 5 | LSM6DSM_ERROR | Yes | WHO_AM_I mismatch at init, or reads returning static values |
| 6 | PI_DEGRADED | No | Pi unresponsive. Set after 3 consecutive missed replies, cleared after 5 consecutive successes |
| 7 | CAM_STATUS | No | Camera recording status is not OK *(different from the enum of various statuses)* |
| 8 | LOW_SSD | No | Flagged if SSD storage is below 20 GB. *Not latched since **WIPE** clears this* |
| 9 | LOW_SSD_FOR_FLIGHT | No | Flagged if SSD storage is below 100 GB (before LO only) |
| 10 | THERMAL_FAULT | No | Sample temperature is outside of safe boundaries |
| 11 | HEATER_FAULT | Yes | Heater is unresponsive to PID/commands: commanded ≥ X% duty for ≥ Y s with no measurable dT/dt |
| 12 | FLUIDIC_FAULT | Yes | Pump commanded on, SLF3S reads below X ml/min for ≥ Y s |
| 13 | TCU_UART_LINK_FAULT | No | UART write failure or transmit buffer overrun on the TCU downlink. *One-way link: cannot detect loss of reception at the TCU* |
| 14 | RXSM_LINK_FAULT | Yes | Mission signals not received over RS-422 |
| 15 | UNEXPECTED_SIGNAL_ORDER | Yes | *Example: SOE received before LO*. Indicates wiring fault or noise |
| 16 | MODE_TRANSITION_FAULT | Yes | A transition not permitted by the state machine was requested and rejected |
| 17 | WATCHDOG_RESET | Yes | Last reset was watchdog-initiated; set at boot |
| 18 | CSV_WRITE_FAULT | Yes | Pi reported a metadata write failure |
| 19 | CONFIG_LOAD_FAULT | Yes | Configuration could not be read at startup; running on compiled defaults |
| 20 | VIDEO_SHUTDOWN_INCOMPLETE | No | Video recording did not have time to shutdown during join period |
| 21–31 | *Reserved* | — | Available for future use; transmitted as zero |