#include "uart_tcu.h"
#include "board_pins.h"
#include "protocol.h"
#include "crc16.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

/* UART1, not UART0. UART0 is the ROM console, and on this board revision
   the console pins and the transceiver pins are the same two wires -- see
   board_pins.h, PIN_RS422_RX / PIN_RS422_TX. The IDF console must be set
   to USB Serial/JTAG (menuconfig: ESP_CONSOLE_USB_SERIAL_JTAG) or the
   bootloader will drive the transceiver's RO output every reset. */
#define TCU_UART_PORT       UART_NUM_1
#define TCU_RX_BUF_SZ       512
#define TCU_TX_BUF_SZ       1024

/* Longest uplink payload is HEAT: 1 byte on/off + 4 bytes duration. */
#define TC_MAX_PAYLOAD      8
#define TC_FRAME_MAX        (TC_MAX_PAYLOAD + PROTO_OVERHEAD_BYTES)

/* Reassembly buffer. Bytes arrive in arbitrary chunks, so a frame may be
   split across several poll() calls; this holds the partial one. */
static uint8_t  rxbuf[TC_FRAME_MAX * 2];
static size_t   rxlen;

static bool     test_pending;
static bool     wipe_pending;
static bool     heat_pending;
static bool     heat_on;
static uint32_t heat_duration_s;

static int64_t  last_frame_us = INT64_MIN;

esp_err_t uart_tcu_init(void)
{
    uart_config_t cfg;
    memset(&cfg, 0, sizeof cfg);
    cfg.baud_rate  = TCU_BAUD_RATE;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err = uart_driver_install(TCU_UART_PORT, TCU_RX_BUF_SZ,
                                        TCU_TX_BUF_SZ, 0, NULL, 0);
    if (err != ESP_OK) return err;

    err = uart_param_config(TCU_UART_PORT, &cfg);
    if (err != ESP_OK) return err;

    /* Pins assigned explicitly through the GPIO matrix rather than using
       the default UART0 mapping, because the schematic wires RO and DI
       the opposite way round from the usual convention. If hardware
       swaps the two nets, these two arguments swap back. */
    err = uart_set_pin(TCU_UART_PORT, PIN_RS422_TX, PIN_RS422_RX,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    uart_flush_input(TCU_UART_PORT);
    rxlen = 0;
    return ESP_OK;
}

int uart_tcu_send(const void *data, size_t len)
{
    return uart_write_bytes(TCU_UART_PORT, data, len);
}

/* Consumes one frame starting at rxbuf[0], which is known to begin with
   the sync pattern. Returns the number of bytes consumed, or 0 if the
   frame is not complete yet. */
static size_t try_consume_frame(void)
{
    if (rxlen < PROTO_OVERHEAD_BYTES) return 0;

    const uint8_t id  = rxbuf[2];
    const uint8_t len = rxbuf[3];
    if (len > TC_MAX_PAYLOAD) return 1;      /* implausible: skip one byte
                                                and let the scanner resync */

    const size_t total = (size_t)len + PROTO_OVERHEAD_BYTES;
    if (rxlen < total) return 0;             /* wait for the rest */

    /* CRC covers id, length and payload -- not the sync bytes, which
       carry no information, and not itself. */
    const uint16_t want = (uint16_t)((uint16_t)rxbuf[total - 1] << 8
                                     | rxbuf[total - 2]);
    const uint16_t got  = crc16_ccitt(&rxbuf[2], (size_t)len + 2u);
    if (want != got) return 1;               /* bad frame: resync, do not act */

    const uint8_t *p = &rxbuf[4];
    switch (id) {
    case TC_ID_TEST:
        test_pending = true;
        break;
    case TC_ID_WIPE:
        wipe_pending = true;
        break;
    case TC_ID_HEAT:
        if (len >= 5) {
            const uint32_t secs = (uint32_t)p[1]
                                | (uint32_t)p[2] << 8
                                | (uint32_t)p[3] << 16
                                | (uint32_t)p[4] << 24;
            /* A zero or absurd duration is rejected rather than clamped.
               An override with no end is the one thing the ground must
               not be able to ask for. */
            if (secs > 0u && secs <= 3600u) {
                heat_on         = (p[0] != 0u);
                heat_duration_s = secs;
                heat_pending    = true;
            }
        }
        break;
    default:
        break;                               /* unknown ID: well-formed,
                                                so still counts as a frame */
    }

    last_frame_us = esp_timer_get_time();
    return total;
}

void uart_tcu_poll(void)
{
    /* Top up the reassembly buffer. */
    if (rxlen < sizeof rxbuf) {
        const int n = uart_read_bytes(TCU_UART_PORT, &rxbuf[rxlen],
                                      (uint32_t)(sizeof rxbuf - rxlen), 0);
        if (n > 0) rxlen += (size_t)n;
    }

    /* Scan for sync, then try to take frames off the front. */
    for (;;) {
        size_t start = 0;
        while (start + 1u < rxlen
               && !(rxbuf[start] == PROTO_SYNC0 && rxbuf[start + 1] == PROTO_SYNC1)) {
            start++;
        }
        if (start > 0) {
            memmove(rxbuf, &rxbuf[start], rxlen - start);
            rxlen -= start;
        }
        if (rxlen < 2u) break;

        const size_t used = try_consume_frame();
        if (used == 0u) break;               /* incomplete: wait for more */
        memmove(rxbuf, &rxbuf[used], rxlen - used);
        rxlen -= used;
    }

    /* If the buffer filled with junk that never synced, drop it rather
       than wedging the parser forever. */
    if (rxlen >= sizeof rxbuf) rxlen = 0;
}

bool uart_tcu_take_test_request(void)
{
    const bool v = test_pending;
    test_pending = false;
    return v;
}

bool uart_tcu_take_wipe_request(void)
{
    const bool v = wipe_pending;
    wipe_pending = false;
    return v;
}

bool uart_tcu_take_heat_request(bool *on, uint32_t *duration_s)
{
    if (!heat_pending) return false;
    heat_pending = false;
    if (on)         *on         = heat_on;
    if (duration_s) *duration_s = heat_duration_s;
    return true;
}

uint32_t uart_tcu_ms_since_last_frame(void)
{
    if (last_frame_us == INT64_MIN) return UINT32_MAX;
    const int64_t dt_ms = (esp_timer_get_time() - last_frame_us) / 1000;
    if (dt_ms < 0)                   return 0u;
    if (dt_ms > (int64_t)UINT32_MAX) return UINT32_MAX;
    return (uint32_t)dt_ms;
}