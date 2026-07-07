#include "signal_processing.h"

#include <math.h>


#define PI_F 3.14159265358979323846f


/*----------------------------------------------------------
 * Remove DC Component
 *---------------------------------------------------------*/

esp_err_t signal_processing_remove_dc(
    const float *input_samples,
    float *output_samples,
    size_t sample_count
)
{
    if (input_samples == NULL ||
        output_samples == NULL ||
        sample_count == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }


    float sum = 0.0f;


    for (size_t i = 0; i < sample_count; i++)
    {
        sum += input_samples[i];
    }


    float mean =
        sum / (float)sample_count;


    for (size_t i = 0; i < sample_count; i++)
    {
        output_samples[i] =
            input_samples[i] - mean;
    }


    return ESP_OK;
}


/*----------------------------------------------------------
 * Hann Window
 *---------------------------------------------------------*/

esp_err_t signal_processing_apply_hann_window(
    const float *input_samples,
    float *output_samples,
    size_t sample_count
)
{
    if (input_samples == NULL ||
        output_samples == NULL ||
        sample_count < 2)
    {
        return ESP_ERR_INVALID_ARG;
    }


    for (size_t i = 0; i < sample_count; i++)
    {
        float coefficient =
            0.5f *
            (
                1.0f -
                cosf(
                    (2.0f * PI_F * (float)i) /
                    (float)(sample_count - 1)
                )
            );


        output_samples[i] =
            input_samples[i] * coefficient;
    }


    return ESP_OK;
}


/*----------------------------------------------------------
 * Frequency Analysis
 *
 * V1 implementation:
 * Direct frequency calculation.
 *
 * This allows WINDOW_SIZE = 100.
 * Later this can be replaced by optimized FFT.
 *---------------------------------------------------------*/

 esp_err_t signal_processing_analyze_frequency(
    const float *samples,
    size_t sample_count,
    float sample_rate_hz,
    frequency_metrics_t *metrics
)
{
    if (samples == NULL ||
        metrics == NULL ||
        sample_count < 4 ||
        sample_rate_hz <= 0.0f)
    {
        return ESP_ERR_INVALID_ARG;
    }

    float maximum_amplitude = 0.0f;
    float spectrum_sum = 0.0f;

    size_t dominant_bin = 0;
    size_t analyzed_bins = 0;


    /*
     * Analyze positive frequencies only.
     *
     * Bin 0 is skipped because it represents DC.
     */

    for (size_t k = 1;
         k < sample_count / 2;
         k++)
    {
        float real = 0.0f;
        float imaginary = 0.0f;


        for (size_t n = 0;
             n < sample_count;
             n++)
        {
            float angle =
                (2.0f * PI_F *
                 (float)k *
                 (float)n)
                /
                (float)sample_count;


            real +=
                samples[n] * cosf(angle);


            imaginary -=
                samples[n] * sinf(angle);
        }


        float amplitude =
            sqrtf(
                (real * real) +
                (imaginary * imaginary)
            );


        amplitude *=
            2.0f / (float)sample_count;


        spectrum_sum += amplitude;

        analyzed_bins++;


        if (amplitude > maximum_amplitude)
        {
            maximum_amplitude = amplitude;

            dominant_bin = k;
        }
    }


    /*
     * Calculate average amplitude of all
     * non-dominant spectral bins.
     */

    float average_noise_amplitude = 0.0f;


    if (analyzed_bins > 1)
    {
        average_noise_amplitude =
            (spectrum_sum - maximum_amplitude)
            /
            (float)(analyzed_bins - 1);
    }


    /*
     * Confidence ratio:
     *
     * dominant peak amplitude
     * -----------------------
     * average spectral noise
     */

    float confidence_ratio = 0.0f;


    if (average_noise_amplitude > 0.000001f)
    {
        confidence_ratio =
            maximum_amplitude /
            average_noise_amplitude;
    }


    /*
     * Store results.
     */

    metrics->dominant_frequency_hz =
        ((float)dominant_bin *
         sample_rate_hz)
        /
        (float)sample_count;


    metrics->dominant_amplitude =
        maximum_amplitude;


    metrics->confidence_ratio =
        confidence_ratio;


    /*
     * V1 peak-validity rule.
     *
     * This is deliberately a relative threshold.
     * We will tune it using stationary and
     * controlled-vibration test data.
     */

    metrics->valid_peak =
        confidence_ratio >= 3.0f;


    return ESP_OK;
}