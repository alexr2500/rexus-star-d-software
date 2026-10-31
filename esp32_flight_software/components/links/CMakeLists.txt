idf_component_register(
    SRCS "crc16.c"
         "rs422_interface.c"
         "i2c_interface.c"
         "spi_interface.c"
         "spi_pi_link.c"
         "uart_tcu.c"
    INCLUDE_DIRS "include"
    REQUIRES protocol
    PRIV_REQUIRES driver esp_timer
)