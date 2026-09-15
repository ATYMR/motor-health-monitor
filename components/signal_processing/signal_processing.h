#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include <stddef.h>

#include "esp_err.h"
#include <stdbool.h>


#define SIGNAL_PROCESSING_DIAGNOSTIC_PEAK_COUNT 5


typedef struct
{
    float dominant_frequency_hz;

    float dominant_amplitude;

    float confidence_ratio;

    bool valid_peak;

} frequency_metrics_t;


/*
 * One bin from the top-N spectral peak diagnostic.
 */
typedef struct
{
    float frequency_hz;

    float amplitude_g;

} spectral_peak_t;


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


/*
 * Diagnostic only: returns the top N spectral bins
 * (by amplitude) within the same analysis band used by
 * signal_processing_analyze_frequency(), sorted descending.
 *
 * Does not affect analyze_frequency's output in any way.
 *
 * peaks_out    - caller-provided array, at least max_peaks long
 * max_peaks    - capacity of peaks_out (e.g. SIGNAL_PROCESSING_DIAGNOSTIC_PEAK_COUNT)
 * peaks_found  - actual number of bins written (<= max_peaks)
 */
esp_err_t signal_processing_top_peaks(
    const float *samples,
    size_t sample_count,
    float sample_rate_hz,
    spectral_peak_t *peaks_out,
    size_t max_peaks,
    size_t *peaks_found
);


#endif