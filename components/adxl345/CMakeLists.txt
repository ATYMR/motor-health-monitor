idf_component_register(
    SRCS "adxl345.c"
    INCLUDE_DIRS "."
    REQUIRES
        hal_i2c
        esp_driver_gpio
)
