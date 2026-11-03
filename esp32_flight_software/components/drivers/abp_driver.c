#include "abp_driver.h"
#include "board_pins.h"
#include "spi_interface.h"
#include <stddef.h>

/* The ABP has no register map and no commands. Assert CS, clock out
   four bytes, release CS -- that is the entire protocol. The bytes are:
     [0] bits 7:6 status, bits 5:0 pressure MSBs
     [1]          pressure LSBs            (14 bits total)
     [2] bits 7:0 temperature bits 10:3
     [3] bits 7:5 temperature bits 2:0, bits 4:0 unused  (11 bits total)
   Parts without the temperature option still return four bytes; bytes
   2 and 3 are then meaningless, which is why temperature is optional
   at the call site. */
#define ABP_FRAME_LEN   4

/* Honeywell specifies 800 kHz maximum. The part is at the end of a
   harness on J1, so half that is the sensible choice. */
#define ABP_SPI_CLOCK_HZ  500000
#define ABP_SPI_MODE      0        /* CPOL = 0, CPHA = 0 */

/* 11-bit temperature maps linearly onto -50..+150 C:
     C = counts * 200 / 2047 - 50
   Scaled to 0.01 C and done in integer arithmetic:
     centi_C = counts * 20000 / 2047 - 5000
   counts is at most 2047, so counts * 20000 fits comfortably in int32. */
#define ABP_TEMP_SPAN_CENTI   20000
#define ABP_TEMP_COUNTS_MAX   2047
#define ABP_TEMP_OFFSET_CENTI 5000

static spi_device_handle_t dev;
static abp_status_t        last_status = ABP_STATUS_DIAGNOSTIC;

esp_err_t abp_driver_init(void)
{
    return spi_interface_add_device(PIN_CS_ABP, ABP_SPI_CLOCK_HZ,
                                    ABP_SPI_MODE, &dev);
}

abp_status_t abp_driver_last_status(void) { return last_status; }

esp_err_t abp_driver_read(int16_t *out_counts, int16_t *out_temp_centi_c)
{
    uint8_t rx[ABP_FRAME_LEN] = { 0 };

    /* Nothing is sent. tx_buffer stays NULL: the driver clocks zeros out
       on MOSI, which this part ignores and which is not wired anyway. */
    spi_transaction_t t = {
        .length    = ABP_FRAME_LEN * 8u,
        .tx_buffer = NULL,
        .rx_buffer = rx,
    };

    esp_err_t err = spi_device_polling_transmit(dev, &t);
    if (err != ESP_OK) {
        last_status = ABP_STATUS_DIAGNOSTIC;
        return err;
    }

    last_status = (abp_status_t)((rx[0] >> 6) & 0x03u);

    /* Only status 0 carries usable data. STALE means we polled faster
       than the part updates; CMD_MODE and DIAGNOSTIC mean the part is
       not measuring. In none of those cases is the payload meaningful,
       so the caller's variables are left untouched rather than filled
       with something plausible. */
    if (last_status != ABP_STATUS_NORMAL) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* 14-bit pressure: the low six bits of byte 0 are the MSBs, byte 1
       the LSBs. Masking with 0x3F00 is what drops the status bits. */
    const uint16_t counts = (uint16_t)(((uint16_t)rx[0] << 8) & 0x3F00u)
                          | (uint16_t)rx[1];

    if (out_counts) *out_counts = (int16_t)counts;   /* max 16383: fits */

    if (out_temp_centi_c) {
        /* 11-bit temperature, split across the two bytes: byte 2 holds
           bits 10:3, and the top three bits of byte 3 hold bits 2:0. */
        const uint16_t tc = (uint16_t)((((uint16_t)rx[2] << 3) & 0x07F8u)
                                     | (((uint16_t)rx[3] >> 5) & 0x0007u));

        const int32_t centi =
            ((int32_t)tc * ABP_TEMP_SPAN_CENTI) / ABP_TEMP_COUNTS_MAX
            - ABP_TEMP_OFFSET_CENTI;

        *out_temp_centi_c = (int16_t)centi;   /* -5000..+15000, fits */
    }

    return ESP_OK;
}

int32_t abp_counts_to_pa(int16_t counts)
{
    const int32_t span_counts = ABP_COUNTS_MAX - ABP_COUNTS_MIN;
    const int32_t span_pa     = ABP_RANGE_MAX_PA - ABP_RANGE_MIN_PA;

    return (((int32_t)counts - ABP_COUNTS_MIN) * span_pa) / span_counts
           + ABP_RANGE_MIN_PA;
}