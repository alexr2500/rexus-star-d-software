idf_component_register(
    SRCS "bme_driver.c" "imu_driver.c" "pt100_driver.c" "gpio_driver.c" "abp_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES protocol
    PRIV_REQUIRES driver links
)