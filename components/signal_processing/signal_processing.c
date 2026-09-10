#include "signal_processing.h"

#include <math.h>

#define PI_F 3.14159265358979323846f

#define MIN_ANALYSIS_FREQUENCY_HZ 5.0f
#define MAX_ANALYSIS_FREQUENCY_HZ 40.0f


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
 * Direct frequency calculation using DFT.
 *
 * The original V1 algorithm is preserved.
 * Only the analysis band is constrained to 5-40 Hz
 * to avoid obvious DC/edge-frequency artifacts.
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
     * Calculate the valid analysis range in DFT bins.
     *
     * For the current configuration:
     *
     * sample_rate = 100 Hz
     * sample_count = 100
     * frequency resolution = 1 Hz/bin
     *
     * Therefore:
     *
     * 5 Hz  -> bin 5
     * 40 Hz -> bin 40
     */

    size_t min_bin =
        (size_t)ceilf(
            (MIN_ANALYSIS_FREQUENCY_HZ *
             (float)sample_count) /
            sample_rate_hz
        );

    size_t max_bin =
        (size_t)floorf(
            (MAX_ANALYSIS_FREQUENCY_HZ *
             (float)sample_count) /
            sample_rate_hz
        );


    /*
     * Positive-frequency DFT bins only.
     *
     * Nyquist bin is excluded because the original V1
     * implementation analyzed bins below sample_count / 2.
     */

    size_t positive_frequency_limit =
        sample_count / 2;

    if (positive_frequency_limit == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (min_bin < 1)
    {
        min_bin = 1;
    }

    if (max_bin >= positive_frequency_limit)
    {
        max_bin =
            positive_frequency_limit - 1;
    }

    if (min_bin > max_bin)
    {
        return ESP_ERR_INVALID_ARG;
    }


    /*
     * Analyze only the selected frequency band.
     */

    for (size_t k = min_bin;
         k <= max_bin;
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
     * Preserve the original confidence threshold.
     */

    metrics->valid_peak =
        confidence_ratio >= 3.0f;


    return ESP_OK;
}