#ifndef UART_TCU_H
#define UART_TCU_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

/* The RS-422 serial link to the RXSM, through the ISL83491 transceiver
   (IC3), and onward to the TCU and the ground station.
   115200 8N1. The transceiver's DE and ~RE are hard-tied on the board, so
   it is always driving and always receiving: there is no direction line
   for software to manage.

   Downlink: telemetry packets, framed the same way as the Pi link.
   Uplink:   the three telecommands of SED 4.9.

   Telecommands ARE events, unlike the REXUS discretes. The take_* calls
   below are therefore read-and-clear by design: one uplinked TEST runs
   one self-test, and a caller that reads it consumes it. */

/* Uplink command IDs. The ground station must mirror these. */
#define TC_ID_TEST  0x10
#define TC_ID_WIPE  0x11
#define TC_ID_HEAT  0x12

esp_err_t uart_tcu_init(void);

/* Queues bytes for downlink. Non-blocking; returns the number written,
   which is less than len if the driver's TX buffer is full. */
int uart_tcu_send(const void *data, size_t len);

/* Drains the RX buffer and parses any complete uplink frames. Call at a
   steady rate. Frames with a bad CRC are discarded silently; the
   resynchronisation is handled by scanning for the sync pattern. */
void uart_tcu_poll(void);

/* Read-and-clear. Each returns true at most once per received command. */
bool uart_tcu_take_test_request(void);
bool uart_tcu_take_wipe_request(void);
/* On true, *on and *duration_s carry the requested override. A duration
   of 0 is rejected at parse time, so *duration_s is always non-zero:
   temperature_control_override() must never be handed an open-ended
   request from the ground. */
bool uart_tcu_take_heat_request(bool *on, uint32_t *duration_s);

/* Milliseconds since the last well-formed uplink frame, saturating.
   UINT32_MAX until the first one arrives. health_monitor.c turns this
   into ERR_BIT_TCU_UART_LINK; this module sets no error bits itself. */
uint32_t uart_tcu_ms_since_last_frame(void);

#endif /* UART_TCU_H */