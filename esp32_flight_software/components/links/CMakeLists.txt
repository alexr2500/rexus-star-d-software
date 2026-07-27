idf_component_register(
    SRCS "uart_pi_link.c" "uart_tcu.c" "rs422_interface.c" "i2c_interface.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
)