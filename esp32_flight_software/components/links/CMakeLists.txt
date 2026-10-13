idf_component_register(
    SRCS "crc16.c"
         "rs422_interface.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver
)