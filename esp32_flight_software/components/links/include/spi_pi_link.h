#ifndef SPI_PI_LINK_H
#define SPI_PI_LINK_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "protocol.h"

/* The ESP32 <-> Raspberry Pi link, on its own SPI bus (J3).
 *
 * The Pi is the SPI MASTER because Linux spidev is master-only; the ESP32
 * is the electrical slave. The ESP32 is still the LOGICAL master: it
 * chooses the content of every frame, and the Pi only provides the clock
 * at 10 Hz. SED table 4.13.
 *
 * NO HANDSHAKE LINE. J3 has four pins -- CS, CLK, MOSI, MISO -- and no
 * fifth wire for the ESP to say "my transmit buffer is armed". The link
 * therefore runs one cycle behind: whatever this module loads now is read
 * by the Pi on the NEXT transaction, so the Pi's data is up to
 * SPI_TRANSACTION_PERIOD_MS (100 ms) old. That is acceptable because the
 * Pi only logs and records; nothing on the Pi closes a control loop. It
 * is recorded here because it is a property of the board, not a choice.
 *
 * Consequence worth knowing: if the Pi stops clocking, this link dies
 * completely. A slave cannot initiate. The degraded flag below is the
 * only warning the ESP gets.
 */

esp_err_t spi_pi_link_init(void);

/* Collects any completed transaction, parses it, and arms the next one.
   Call at a steady rate at least as fast as the Pi clocks. Cheap when
   nothing has completed. */
void spi_pi_link_service(void);

/* Loads the payload the Pi will receive on the next transaction. id is a
   message_id_t; len must be <= PROTO_MAX_PAYLOAD. Replaces whatever was
   loaded before, so calling it faster than the Pi clocks drops frames --
   which is correct: the Pi wants the freshest sample, not a backlog. */
esp_err_t spi_pi_link_load(uint8_t id, const uint8_t *payload, uint8_t len);

/* Read-and-clear: true once per STATUS frame received from the Pi.
   Any pointer may be NULL. */
bool spi_pi_link_take_status(camera_status_t *cam,
                             uint8_t *ssd_free_gb,
                             bool *pre_LO,
                             command_result_t *last_result);

/* True while the link is considered unhealthy. Set after
   MISS_COUNT_DEGRADED consecutive service() calls with no completed
   transaction, cleared after SUCCESS_COUNT_CLEAR_DEGRADED good frames.
   error_manager owns ERR_BIT_PI_DEGRADED; this module only reports. */
bool spi_pi_link_is_degraded(void);

/* Milliseconds since the last valid frame, saturating. UINT32_MAX until
   the first one. */
uint32_t spi_pi_link_ms_since_last_frame(void);

#endif /* SPI_PI_LINK_H */