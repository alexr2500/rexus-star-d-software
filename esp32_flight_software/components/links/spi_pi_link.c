#include "spi_pi_link.h"
#include "board_pins.h"
#include "crc16.h"
#include "error_manager.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

/* DMA needs 4-byte aligned buffers in DMA-capable memory. WORD_ALIGNED_ATTR
   on statics is the simplest way to guarantee both on ESP32. */
static WORD_ALIGNED_ATTR uint8_t tx_buf[SPI_FRAME_LEN];
static WORD_ALIGNED_ATTR uint8_t rx_buf[SPI_FRAME_LEN];

static bool tx_armed = false;          /* a real frame is loaded */

static camera_status_t  pi_camera   = CAM_UNKNOWN;
static uint8_t          pi_ssd_gb   = SSD_VALUE_UNKNOWN;
static command_result_t pi_cmd_res  = RESULT_NONE;

static uint8_t  miss_count    = 0;
static uint8_t  success_count = 0;
static bool     degraded      = true;  /* until proven otherwise */


/* --- ISR callbacks ---------------------------------------------------
 * These run in interrupt context. IRAM_ATTR because they must not fault
 * on a flash cache miss, and nothing in here may log or allocate -
 * ESP_LOGx is not ISR-safe. */

static void IRAM_ATTR on_setup(spi_slave_transaction_t *t)
{
    (void)t;
    gpio_set_level(PIN_PI_HANDSHAKE, 1);   /* buffer armed, clock away */
}

static void IRAM_ATTR on_trans(spi_slave_transaction_t *t)
{
    (void)t;
    gpio_set_level(PIN_PI_HANDSHAKE, 0);
}


void spi_pi_link_init(void)
{
    /* TODO: PIN_PI_* must go in board_pins.h. The Pi's SPI pins are
       fixed by the CM5 pin map, so these are dictated by hardware, not
       chosen. Confirm against the CM5 header and the schematic. */
    const spi_bus_config_t bus = {
        .mosi_io_num = PIN_PI_MOSI,
        .miso_io_num = PIN_PI_MISO,
        .sclk_io_num = PIN_PI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    const spi_slave_interface_config_t slv = {
        .mode          = 0,            /* TODO: must match the Pi's
                                          spidev mode exactly. Mode
                                          mismatch reads bit-shifted
                                          garbage that still sometimes
                                          passes a sync-byte scan. */
        .spics_io_num  = PIN_PI_CS,
        .queue_size    = 2,
        .flags         = 0,
        .post_setup_cb = on_setup,
        .post_trans_cb = on_trans,
    };

    gpio_set_direction(PIN_PI_HANDSHAKE, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_PI_HANDSHAKE, 0);

    spi_slave_initialize(PI_SPI_HOST, &bus, &slv, SPI_DMA_CH_AUTO);

    memset(tx_buf, 0, sizeof(tx_buf));
    tx_armed = false;
}


bool spi_pi_link_send(message_id_t id, const uint8_t *payload, uint8_t len)
{
    if (tx_armed) {
        return false;                  /* previous frame not sent yet */
    }
    if ((size_t)len + PROTO_OVERHEAD_BYTES > SPI_FRAME_LEN) {
        return false;
    }

    memset(tx_buf, 0, sizeof(tx_buf));
    tx_buf[0] = PROTO_SYNC0;
    tx_buf[1] = PROTO_SYNC1;
    tx_buf[2] = (uint8_t)id;
    tx_buf[3] = len;
    memcpy(&tx_buf[4], payload, len);

    /* CRC covers id, length and payload - not the sync bytes, which are
       a framing marker rather than data. Same coverage as framing.py. */
    const uint16_t crc = crc16_ccitt(&tx_buf[2], (size_t)len + 2u);
    tx_buf[4 + len] = (uint8_t)(crc & 0xFFu);
    tx_buf[5 + len] = (uint8_t)(crc >> 8);   /* little-endian, as the
                                                rest of the protocol */
    tx_armed = true;
    return true;
}


static void handle_status(const uint8_t *p, uint8_t len)
{
    if (len != LEN_STATUS) {
        return;                        /* malformed; counted by caller */
    }
    pi_camera  = (camera_status_t)p[0];
    pi_ssd_gb  = p[1];
    /* p[2] is command_seq_echo - checked by whoever issued the command */
    pi_cmd_res = (command_result_t)p[3];

    error_update_pi_status(pi_camera, pi_ssd_gb, /* pre_lo */ false);
    /* TODO: pre_lo must come from the state machine, not be hardcoded.
       Bit 9 (LOW_SSD_FOR_FLIGHT) is wrong until this is wired. */
}


void spi_pi_link_service(void)
{
    spi_slave_transaction_t t = {
        .length    = SPI_FRAME_LEN * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    /* Blocks until the Pi clocks. If the Pi is dead this never returns,
       which is why this belongs in its own task and not in the main
       loop - a dead Pi must not stop the mission.
       TODO: use spi_slave_queue_trans + get_trans_result with a timeout
       so a silent Pi is detected rather than waited on forever. */
    if (spi_slave_transmit(PI_SPI_HOST, &t, portMAX_DELAY) != ESP_OK) {
        tx_armed = false;
        return;
    }
    tx_armed = false;

    /* --- validate what came back ------------------------------------ */
    bool ok = false;

    if (rx_buf[0] == PROTO_SYNC0 && rx_buf[1] == PROTO_SYNC1) {
        const uint8_t id  = rx_buf[2];
        const uint8_t len = rx_buf[3];

        if ((size_t)len + PROTO_OVERHEAD_BYTES <= SPI_FRAME_LEN) {
            const uint16_t want = (uint16_t)rx_buf[4 + len]
                                | ((uint16_t)rx_buf[5 + len] << 8);
            if (crc16_ccitt(&rx_buf[2], (size_t)len + 2u) == want) {
                ok = true;
                if (id == MSG_STATUS) {
                    handle_status(&rx_buf[4], len);
                }
            }
        }
    }

    /* --- link health, per protocol.h --------------------------------- */
    /* An all-zero RX is the Pi clocking with nothing loaded, which is
       indistinguishable from a dead Pi at this layer - both are "no
       valid STATUS" and both count as a miss. */
    if (ok) {
        miss_count = 0;
        if (success_count < SUCCESS_COUNT_CLEAR_DEGRADED) {
            success_count++;
        }
        if (degraded && success_count >= SUCCESS_COUNT_CLEAR_DEGRADED) {
            degraded = false;
            error_clear(ERR_BIT_PI_DEGRADED);
        }
    } else {
        success_count = 0;
        if (miss_count < MISS_COUNT_DEGRADED) {
            miss_count++;
        }
        if (!degraded && miss_count >= MISS_COUNT_DEGRADED) {
            degraded = true;
            error_raise(ERR_BIT_PI_DEGRADED);
        }
    }
}


camera_status_t  spi_pi_link_camera_status(void) { return pi_camera; }
uint8_t          spi_pi_link_ssd_free_gb(void)   { return pi_ssd_gb; }
command_result_t spi_pi_link_command_result(void){ return pi_cmd_res; }
bool             spi_pi_link_is_degraded(void)   { return degraded; }