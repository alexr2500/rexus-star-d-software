#include "spi_pi_link.h"
#include "board_pins.h"
#include "crc16.h"
#include "driver/spi_slave.h"
#include "driver/spi_common.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

/* SPI3_HOST: SPI2 carries the sensors, SPI0/1 the in-package flash. */
#define PI_SPI_HOST   SPI3_HOST

/* DMA requires the buffers to be word-aligned, and the slave driver
   rejects them otherwise. Both are exactly SPI_FRAME_LEN: every
   transaction is the same size in both directions, whatever the payload,
   so neither side ever has to agree on a length up front. */
static WORD_ALIGNED_ATTR uint8_t tx_buf[SPI_FRAME_LEN];
static WORD_ALIGNED_ATTR uint8_t rx_buf[SPI_FRAME_LEN];

static spi_slave_transaction_t trans;
static bool     trans_queued;

static uint8_t  miss_count;
static uint16_t success_count;
static bool     degraded;
static int64_t  last_frame_us = INT64_MIN;

static uint8_t  tx_seq;

/* Latched STATUS contents. */
static bool             status_pending;
static camera_status_t  st_cam;
static uint8_t          st_ssd;
static bool             st_pre_lo;
static command_result_t st_result;

/* ---- framing ---------------------------------------------------------- */

/* Lays out one frame in buf:
     [0] 0xAA  [1] 0x55  [2] id  [3] len  [4] seq  [5..] payload
     then CRC16 little-endian, then zero padding to SPI_FRAME_LEN.
   The CRC covers id, len, seq and payload. */
static void build_frame(uint8_t *buf, uint8_t id, const uint8_t *payload, uint8_t len)
{
    memset(buf, 0, SPI_FRAME_LEN);
    buf[0] = PROTO_SYNC0;
    buf[1] = PROTO_SYNC1;
    buf[2] = id;
    buf[3] = len;
    buf[4] = tx_seq++;
    if (len && payload) memcpy(&buf[5], payload, len);

    const uint16_t crc = crc16_ccitt(&buf[2], (size_t)len + 3u);
    buf[5 + len]     = (uint8_t)(crc & 0xFFu);
    buf[5 + len + 1] = (uint8_t)(crc >> 8);
}

/* Validates a received frame in place. Returns the payload length, or -1. */
static int parse_frame(const uint8_t *buf, uint8_t *id_out, const uint8_t **payload)
{
    if (buf[0] != PROTO_SYNC0 || buf[1] != PROTO_SYNC1) return -1;

    const uint8_t id  = buf[2];
    const uint8_t len = buf[3];
    if ((size_t)len + 7u > SPI_FRAME_LEN) return -1;

    const uint16_t want = (uint16_t)((uint16_t)buf[5 + len + 1] << 8 | buf[5 + len]);
    if (crc16_ccitt(&buf[2], (size_t)len + 3u) != want) return -1;

    *id_out  = id;
    *payload = &buf[5];
    return (int)len;
}

/* ---- transaction plumbing --------------------------------------------- */

static void arm_next(void)
{
    if (trans_queued) return;

    memset(&trans, 0, sizeof trans);
    trans.length    = SPI_FRAME_LEN * 8u;   /* in bits */
    trans.tx_buffer = tx_buf;
    trans.rx_buffer = rx_buf;

    /* Queued without blocking. If the queue is somehow full the link just
       misses this cycle and the miss counter notices. */
    if (spi_slave_queue_trans(PI_SPI_HOST, &trans, 0) == ESP_OK) {
        trans_queued = true;
    }
}

esp_err_t spi_pi_link_init(void)
{
    spi_bus_config_t bus;
    memset(&bus, 0, sizeof bus);
    bus.mosi_io_num   = PIN_RASP_MOSI;
    bus.miso_io_num   = PIN_RASP_MISO;
    bus.sclk_io_num   = PIN_RASP_CLK;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;

    spi_slave_interface_config_t slv;
    memset(&slv, 0, sizeof slv);
    slv.spics_io_num = PIN_RASP_CS;
    slv.queue_size   = 1;      /* one armed transaction is the whole design */
    slv.mode         = SPI_MODE;

    esp_err_t err = spi_slave_initialize(PI_SPI_HOST, &bus, &slv, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) return err;

    /* Arm with an IDLE frame so the very first transaction the Pi clocks
       returns something well-formed rather than whatever the DMA buffer
       happened to contain. */
    build_frame(tx_buf, MSG_IDLE, NULL, 0);
    trans_queued = false;
    arm_next();
    return ESP_OK;
}

esp_err_t spi_pi_link_load(uint8_t id, const uint8_t *payload, uint8_t len)
{
    if (len > PROTO_MAX_PAYLOAD) return ESP_ERR_INVALID_SIZE;

    /* Written straight into the armed DMA buffer. The window in which the
       Pi could clock a half-updated frame is real but harmless: the CRC
       is written last, so a torn frame fails its check at the Pi and is
       counted as a miss rather than acted on. */
    build_frame(tx_buf, id, payload, len);
    return ESP_OK;
}

void spi_pi_link_service(void)
{
    spi_slave_transaction_t *done = NULL;

    /* Non-blocking: zero ticks. Nothing completed means the Pi has not
       clocked since the last call, which is normal most of the time. */
    const esp_err_t err = spi_slave_get_trans_result(PI_SPI_HOST, &done, 0);

    if (err != ESP_OK || done == NULL) {
        if (miss_count < MISS_COUNT_DEGRADED) {
            miss_count++;
            if (miss_count >= MISS_COUNT_DEGRADED) {
                degraded      = true;
                success_count = 0;
            }
        }
        arm_next();          /* make sure one is always armed */
        return;
    }

    trans_queued = false;

    uint8_t        id      = 0;
    const uint8_t *payload = NULL;
    const int      len     = parse_frame(rx_buf, &id, &payload);

    if (len < 0) {
        /* A transaction happened but the content was not a valid frame.
           Counted as a miss: the wire moved, but no information arrived. */
        if (miss_count < MISS_COUNT_DEGRADED) {
            miss_count++;
            if (miss_count >= MISS_COUNT_DEGRADED) {
                degraded      = true;
                success_count = 0;
            }
        }
        arm_next();
        return;
    }

    last_frame_us = esp_timer_get_time();
    miss_count    = 0;

    if (degraded) {
        if (++success_count >= SUCCESS_COUNT_CLEAR_DEGRADED) {
            degraded      = false;
            success_count = 0;
        }
    }

    if (id == MSG_STATUS && len >= LEN_STATUS) {
        st_cam         = (camera_status_t)payload[0];
        st_ssd         = payload[1];
        st_pre_lo      = (payload[2] != 0u);
        st_result      = (command_result_t)payload[3];
        status_pending = true;
    }

    arm_next();
}

bool spi_pi_link_take_status(camera_status_t *cam,
                             uint8_t *ssd_free_gb,
                             bool *pre_LO,
                             command_result_t *last_result)
{
    if (!status_pending) return false;
    status_pending = false;
    if (cam)         *cam         = st_cam;
    if (ssd_free_gb) *ssd_free_gb = st_ssd;
    if (pre_LO)      *pre_LO      = st_pre_lo;
    if (last_result) *last_result = st_result;
    return true;
}

bool spi_pi_link_is_degraded(void) { return degraded; }

uint32_t spi_pi_link_ms_since_last_frame(void)
{
    if (last_frame_us == INT64_MIN) return UINT32_MAX;
    const int64_t dt_ms = (esp_timer_get_time() - last_frame_us) / 1000;
    if (dt_ms < 0)                   return 0u;
    if (dt_ms > (int64_t)UINT32_MAX) return UINT32_MAX;
    return (uint32_t)dt_ms;
}