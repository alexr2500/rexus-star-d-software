idf_component_register(
    SRCS "crc16.c"
         "spi_interface.c"
         "spi_pi_link.c"
         "rs422_interface.c"
         "uart_tcu.c"
         "i2c_interface.c"
    INCLUDE_DIRS "include"
    REQUIRES protocol
    PRIV_REQUIRES driver esp_timer
)