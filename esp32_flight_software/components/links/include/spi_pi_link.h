#ifndef SPI_PI_LINK_H
#define SPI_PI_LINK_H

#include <stdbool.h>
#include <stdint.h>
#include "protocol.h"

/* ESP32 <-> Raspberry Pi link. ESP32 is the SPI SLAVE.
 *
 * WHY THE ESP IS THE SLAVE
 * Linux spidev is master-only, so the Pi must own SCLK and CS. That is
 * an electrical fact, not a design choice, and it does not change the
 * logical hierarchy: the Pi clocks the bus at a fixed rate and decides
 * nothing; the ESP decides what every one of its own transactions
 * contains.
 *
 * ONE-CYCLE PIPELINE
 * A slave's TX buffer must be loaded before the master clocks. So the
 * frame the ESP sends in transaction N was chosen during N-1. At 10 Hz
 * that is 100 ms of command latency. Nothing in the protocol depends on
 * sub-100 ms response, but do not assume a command takes effect in the
 * same cycle it was decided.
 *
 * HANDSHAKE
 * A GPIO from ESP to Pi goes high when a TX buffer is armed. The Pi
 * waits for it before clocking. Without it the Pi can clock while the
 * slave has no buffer loaded, and the ESP32 shifts out whatever was
 * last in the DMA buffer - stale data with a valid CRC, which is worse
 * than noise.
 *
 * FRAMING
 * Unchanged from the UART design: sync, id, length, payload, CRC-16,
 * exactly as framing.py parses today. Each transaction is a fixed
 * SPI_FRAME_LEN bytes; a short frame is zero-padded and the Pi's
 * parser discards the padding because it is scanning for sync bytes.
 * Keeping the framing means the Pi side needs a new transport, not a
 * new protocol.
 */

/* Fixed transaction size, both directions. Largest real frame is
   SENSOR_DATA: 34 payload + 6 overhead = 40. Rounded up for DMA
   alignment, with headroom. */
#define SPI_FRAME_LEN  48

void spi_pi_link_init(void);

/* Queue one frame for the NEXT transaction. Returns false if a frame is
   already waiting - the caller decides whether to drop or retry, since
   only it knows whether its message is stale. */
bool spi_pi_link_send(message_id_t id, const uint8_t *payload, uint8_t len);

/* Run the link. Blocks until the Pi clocks a transaction, then decodes
   whatever arrived. Call from a dedicated task. */
void spi_pi_link_service(void);

/* Last values from the Pi's STATUS reply. Meaningless while
   ERR_BIT_PI_DEGRADED is set - the ESP keeps reporting the last known
   values, which may be stale. */
camera_status_t  spi_pi_link_camera_status(void);
uint8_t          spi_pi_link_ssd_free_gb(void);
command_result_t spi_pi_link_command_result(void);
bool             spi_pi_link_is_degraded(void);

#endif /* SPI_PI_LINK_H */