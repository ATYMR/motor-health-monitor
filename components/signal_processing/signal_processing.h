#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include <stddef.h>

#include "esp_err.h"
#include <stdbool.h>


typedef struct
{
    float dominant_frequency_hz;

    float dominant_amplitude;

    float confidence_ratio;

    bool valid_peak;

} frequency_metrics_t;


/*
 * Remove DC component
 */
esp_err_t signal_processing_remove_dc(
    const float *input_samples,
    float *output_samples,
    size_t sample_count
);


/*
 * Apply Hann window
 */
esp_err_t signal_processing_apply_hann_window(
    const float *input_samples,
    float *output_samples,
    size_t sample_count
);


/*
 * Analyze frequency spectrum
 */
esp_err_t signal_processing_analyze_frequency(
    const float *samples,
    size_t sample_count,
    float sample_rate_hz,
    frequency_metrics_t *metrics
);


#endif