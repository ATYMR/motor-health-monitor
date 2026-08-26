idf_component_register(
    SRCS "data_logger.c"
    INCLUDE_DIRS "."
    REQUIRES
        health_engine
        signal_processing
        frequency_tracker
)