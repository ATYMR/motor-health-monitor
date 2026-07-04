idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES adxl345 hal_i2c
)