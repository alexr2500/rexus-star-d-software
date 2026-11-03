#include "spi_interface.h"
#include "board_pins.h"
#include "driver/spi_common.h"
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* SPI2_HOST is the general-purpose host on the ESP32-S3. SPI0/SPI1 are
   tied to the in-package flash and must never be reconfigured. SPI3_HOST
   is left for the Pi link. */
#define SENSOR_SPI_HOST   SPI2_HOST

/* Largest single transfer on this bus is the BME280 calibration read:
   1 address byte + 26 data bytes. Rounded up. */
#define SENSOR_MAX_TRANSFER_SZ  64

static bool bus_ready = false;

esp_err_t spi_interface_init(void)
{
    if (bus_ready) return ESP_OK;      /* idempotent: safe to call twice */

    spi_bus_config_t cfg;
    memset(&cfg, 0, sizeof cfg);
    cfg.mosi_io_num     = PIN_SPI_MOSI;
    cfg.miso_io_num     = PIN_SPI_MISO;
    cfg.sclk_io_num     = PIN_SPI_SCK;
    cfg.quadwp_io_num   = -1;          /* not used: plain 4-wire SPI */
    cfg.quadhd_io_num   = -1;
    cfg.max_transfer_sz = SENSOR_MAX_TRANSFER_SZ;

    esp_err_t err = spi_bus_initialize(SENSOR_SPI_HOST, &cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) return err;

    bus_ready = true;
    return ESP_OK;
}

esp_err_t spi_interface_add_device(gpio_num_t cs_pin,
                                   int clock_hz,
                                   uint8_t spi_mode,
                                   spi_device_handle_t *out)
{
    if (out == NULL || spi_mode > 3) return ESP_ERR_INVALID_ARG;
    if (!bus_ready)                  return ESP_ERR_INVALID_STATE;

    spi_device_interface_config_t dev;
    memset(&dev, 0, sizeof dev);
    dev.mode            = spi_mode;
    dev.clock_speed_hz  = clock_hz;
    dev.spics_io_num    = cs_pin;      /* the driver toggles CS for us */
    dev.queue_size      = 1;           /* polling transfers only, no queueing */

    /* One extra half-clock of CS setup and hold. The off-board sensors on
       J1 and J7 sit at the end of a harness, and the MAX31865 in
       particular wants CS stable well before the first edge. */
    dev.cs_ena_pretrans  = 1;
    dev.cs_ena_posttrans = 1;

    return spi_bus_add_device(SENSOR_SPI_HOST, &dev, out);
}