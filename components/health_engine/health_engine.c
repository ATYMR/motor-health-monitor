#include "health_engine.h"

#include <math.h>
#include <float.h>


esp_err_t health_engine_analyze(
    const float *samples,
    size_t sample_count,
    vibration_metrics_t *metrics
)
{
    if (samples == NULL ||
        metrics == NULL ||
        sample_count == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }


    /*
     * Calculate mean.
     *
     * This removes the DC component before
     * vibration metrics are calculated.
     */
    float sum = 0.0f;

    for (size_t i = 0; i < sample_count; i++)
    {
        sum += samples[i];
    }

    float mean = sum / (float)sample_count;


    /*
     * Analyze dynamic acceleration.
     */
    float sum_squared = 0.0f;

    float max_value = -FLT_MAX;
    float min_value = FLT_MAX;

    float peak = 0.0f;


    for (size_t i = 0; i < sample_count; i++)
    {
        float dynamic_sample =
            samples[i] - mean;

        sum_squared +=
            dynamic_sample * dynamic_sample;


        if (dynamic_sample > max_value)
        {
            max_value = dynamic_sample;
        }


        if (dynamic_sample < min_value)
        {
            min_value = dynamic_sample;
        }


        float absolute_value =
            fabsf(dynamic_sample);

        if (absolute_value > peak)
        {
            peak = absolute_value;
        }
    }


    /*
     * RMS
     */
    metrics->rms = sqrtf(
        sum_squared / (float)sample_count
    );


    /*
     * Peak
     */
    metrics->peak = peak;


    /*
     * Peak-to-Peak
     */
    metrics->peak_to_peak =
        max_value - min_value;


    /*
     * Crest Factor
     */
    if (metrics->rms > 0.000001f)
    {
        metrics->crest_factor =
            metrics->peak / metrics->rms;
    }
    else
    {
        metrics->crest_factor = 0.0f;
    }


    return ESP_OK;
}