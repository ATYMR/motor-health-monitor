#include "data_logger.h"

#include <stdio.h>
#include <stddef.h>


static float get_valid_frequency(
    const frequency_metrics_t *frequency
)
{
    if (frequency->valid_peak)
    {
        return frequency->dominant_frequency_hz;
    }

    return 0.0f;
}


esp_err_t data_logger_init(void)
{
    /*
     * CSV header.
     *
     * Every following data row uses this exact order.
     */

    printf(
        "CSV_HEADER,"
        "window,"
        "timestamp_ms,"
        "x_rms_g,"
        "y_rms_g,"
        "z_rms_g,"
        "mag_rms_g,"
        "mag_peak_g,"
        "mag_peak_to_peak_g,"
        "mag_crest_factor,"
        "x_freq_hz,"
        "x_confidence,"
        "y_freq_hz,"
        "y_confidence,"
        "z_freq_hz,"
        "z_confidence,"
        "consensus_valid,"
        "consensus_freq_hz,"
        "tracker_state,"
        "candidate_freq_hz,"
        "stable_freq_hz,"
        "confirmation_count,"
        "missed_window_count\n"
    );

    return ESP_OK;
}


esp_err_t data_logger_write(
    const data_logger_record_t *record
)
{
    if (record == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }


    printf(
        "CSV_DATA,"
        "%lu,"
        "%lld,"
        "%.6f,"
        "%.6f,"
        "%.6f,"
        "%.6f,"
        "%.6f,"
        "%.6f,"
        "%.3f,"
        "%.2f,"
        "%.2f,"
        "%.2f,"
        "%.2f,"
        "%.2f,"
        "%.2f,"
        "%d,"
        "%.2f,"
        "%s,"
        "%.2f,"
        "%.2f,"
        "%d,"
        "%d\n",

        (unsigned long)record->window_number,

        (long long)record->timestamp_ms,

        record->x_metrics.rms,
        record->y_metrics.rms,
        record->z_metrics.rms,

        record->magnitude_metrics.rms,
        record->magnitude_metrics.peak,
        record->magnitude_metrics.peak_to_peak,
        record->magnitude_metrics.crest_factor,

        get_valid_frequency(
            &record->x_frequency
        ),

        record->x_frequency.confidence_ratio,

        get_valid_frequency(
            &record->y_frequency
        ),

        record->y_frequency.confidence_ratio,

        get_valid_frequency(
            &record->z_frequency
        ),

        record->z_frequency.confidence_ratio,

        record->consensus_valid ? 1 : 0,

        record->consensus_valid
            ? record->consensus_frequency_hz
            : 0.0f,

        frequency_tracker_state_string(
            record->tracker_state
        ),

        record->candidate_frequency_hz,

        record->stable_frequency_hz,

        record->confirmation_count,

        record->missed_window_count
    );


    return ESP_OK;
}
