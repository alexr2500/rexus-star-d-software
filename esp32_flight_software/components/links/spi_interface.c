#include "spi_interface.h"
#include "board_pins.h"
#include <string.h>

bool spi_interface_init(void)
{
    const spi_bus_config_t bus = {
        .mosi_io_num     = PIN_SPI_MOSI,
        .miso_io_num     = PIN_SPI_MISO,
        .sclk_io_num     = PIN_SPI_SCK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 64,
    };
#if 0   /* TODO: enable with board_pins.h */
    return spi_bus_initialize(SENSOR_SPI_HOST, &bus, SPI_DMA_CH_AUTO) == ESP_OK;
#else
    (void)bus; return false;
#endif
}


bool spi_interface_add_device(int cs_pin, uint8_t mode, int clock_hz,
                              spi_device_handle_t *out)
{
    spi_device_interface_config_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.command_bits    = 0;
    dev.address_bits    = 8;        /* every device here is
                                       "one address byte, then data" */
    dev.mode            = mode;
    dev.clock_speed_hz  = clock_hz;
    dev.spics_io_num    = cs_pin;
    dev.queue_size      = 1;

#if 0
    return spi_bus_add_device(SENSOR_SPI_HOST, &dev, out) == ESP_OK;
#else
    (void)out; return false;
#endif
}