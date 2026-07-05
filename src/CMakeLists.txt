idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        hal_i2c
        adxl345
        sensor_manager
        health_engine
)