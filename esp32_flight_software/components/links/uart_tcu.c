#include "uart_tcu.h"
#include "board_pins.h"
#include "crc16.h"
#include "error_manager.h"
#include "driver/uart.h"
#include <string.h>

#define TCU_UART_NUM    UART_NUM_2      /* TODO confirm against schematic */
#define TCU_RX_BUF_LEN  256
#define TCU_TX_BUF_LEN  256

static bool test_request_pending = false;


void uart_tcu_init(void)
{
    const uart_config_t cfg = {
        .baud_rate = BAUD_RATE,          /* 115200, ICD default */
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,   /* 8N1 */
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* TODO: enable with board_pins.h.
       Note the ICD's signal-integrity requirement: 100 ohm twisted pair
       with 100 ohm termination at both ends, and slew-rate-controlled
       transmitters below 1 Mbit/s. That is a hardware obligation, but
       worth a comment here because it is the first thing to check when
       the link is noisy. */
#if 0
    uart_driver_install(TCU_UART_NUM, TCU_RX_BUF_LEN, TCU_TX_BUF_LEN, 0, NULL, 0);
    uart_param_config(TCU_UART_NUM, &cfg);
    uart_set_pin(TCU_UART_NUM, PIN_TCU_TX, PIN_TCU_RX,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
#else
    (void)cfg;
#endif
}


bool uart_tcu_send(const uint8_t *data, size_t len)
{
#if 0
    const int written = uart_write_bytes(TCU_UART_NUM, data, len);
    if (written != (int)len) {
        error_raise(ERR_BIT_TCU_UART_LINK);
        return false;
    }
    return true;
#else
    (void)data; (void)len;
    return false;
#endif
}


void uart_tcu_poll_uplink(void)
{
    /* TODO: read available bytes, run them through the same frame
       parser the Pi link uses, and dispatch:
         CMD_TEST -> test_request_pending = true
         CMD_WIPE -> forward to the Pi over the SPI link
         HEAT     -> temperature_control_override(...)   [SED 4.9]

       Two things to settle before writing it:
         - HEAT is in the SED but NOT in command_t in protocol.h. Either
           add it or drop it from the SED.
         - The HEAT override needs a duration. An override with no
           expiry on a heater inside a sealed vessel full of living
           cells is a safety hazard, not a convenience. */
}


bool uart_tcu_take_test_request(void)
{
    const bool r = test_request_pending;
    test_request_pending = false;
    return r;
}