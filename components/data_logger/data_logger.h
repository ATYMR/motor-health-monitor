#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "health_engine.h"
#include "signal_processing.h"
#include "frequency_tracker.h"


typedef struct
{
    uint32_t window_number;
    int64_t timestamp_ms;

    vibration_metrics_t x_metrics;
    vibration_metrics_t y_metrics;
    vibration_metrics_t z_metrics;
    vibration_metrics_t magnitude_metrics;

    frequency_metrics_t x_frequency;
    frequency_metrics_t y_frequency;
    frequency_metrics_t z_frequency;

    bool consensus_valid;
    float consensus_frequency_hz;

    frequency_tracker_state_t tracker_state;
    float candidate_frequency_hz;
    float stable_frequency_hz;

    int confirmation_count;
    int missed_window_count;

} data_logger_record_t;


esp_err_t data_logger_init(void);


esp_err_t data_logger_write(
    const data_logger_record_t *record
);


#endif