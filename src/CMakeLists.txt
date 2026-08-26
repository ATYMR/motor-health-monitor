idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        hal_i2c
        adxl345
        ds18b20
        sensor_manager
        health_engine
        signal_processing
        frequency_tracker
)