#ifndef UART_TCU_H
#define UART_TCU_H

#include <stdbool.h>
#include <stdint.h>
#include "protocol.h"

/* RS-422 link to the RXSM service module. Two separate pairs on the
 * D-sub (ICD Table 2.2): "Telemetry to SM" out, "Telecommand to Exp"
 * in. One UART peripheral serves both.
 *
 * 8N1, 115200 by default (ICD 2.2). MORABA provides a transparent byte
 * stream - no message protocol is imposed, but checksums and sync are
 * recommended, which is what the telemetry packet already does.
 *
 * The downlink is ONE-WAY in the sense that matters: a successful write
 * proves we transmitted, never that the SM received. ERR_BIT_TCU_UART_LINK
 * can only ever report a local failure.
 */

void uart_tcu_init(void);

/* Blocking write of one telemetry packet. ~2.8 ms for 32 bytes at
   115200, so it is survivable in a task and not in an ISR. Returns
   false and raises ERR_BIT_TCU_UART_LINK on failure. */
bool uart_tcu_send(const uint8_t *data, size_t len);

/* Non-blocking: drains the uplink and acts on any complete command.
   Call at a steady rate. */
void uart_tcu_poll_uplink(void);

/* Latched request from a TEST telecommand; reading it clears it. The
   one place in this codebase where read-and-clear is correct, because
   this IS an event and not a level. */
bool uart_tcu_take_test_request(void);

#endif /* UART_TCU_H */