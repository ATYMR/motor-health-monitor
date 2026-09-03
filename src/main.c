#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_timer.h"

#include "hal_i2c.h"
#include "adxl345.h"
#include "sensor_manager.h"
#include "health_engine.h"
#include "signal_processing.h"
#include "frequency_tracker.h"
#include "data_logger.h"
#include "ds18b20.h"


/*----------------------------------------------------------
 * Acquisition Configuration
 *---------------------------------------------------------*/

#define SAMPLE_RATE_HZ             100
#define WINDOW_SIZE                100

#define DATA_READY_TIMEOUT_MS      30

#define CONSENSUS_TOLERANCE_HZ     1.0f

#define DS18B20_GPIO               4


/*----------------------------------------------------------
 * Static Sample Buffers
 *---------------------------------------------------------*/

static float x_window[WINDOW_SIZE];

static float y_window[WINDOW_SIZE];

static float z_window[WINDOW_SIZE];

static float magnitude_window[WINDOW_SIZE];


/*----------------------------------------------------------
 * Signal Processing Buffers
 *---------------------------------------------------------*/

static float x_dc_removed[WINDOW_SIZE];

static float y_dc_removed[WINDOW_SIZE];

static float z_dc_removed[WINDOW_SIZE];

static float x_windowed[WINDOW_SIZE];

static float y_windowed[WINDOW_SIZE];

static float z_windowed[WINDOW_SIZE];


/*----------------------------------------------------------
 * Print Frequency Result
 *---------------------------------------------------------*/

static void print_frequency_result(
    const char *axis_name,
    const frequency_metrics_t *metrics
)
{
    if (
        metrics->valid_peak
    )
    {
        printf(
            "%s Axis | Freq: %.2f Hz | "
            "Amp: %.5f g | "
            "Conf: %.2f | VALID\n",

            axis_name,

            metrics->dominant_frequency_hz,

            metrics->dominant_amplitude,

            metrics->confidence_ratio
        );
    }
    else
    {
        printf(
            "%s Axis | Freq: NONE | "
            "Conf: %.2f | INVALID\n",

            axis_name,

            metrics->confidence_ratio
        );
    }
}


/*----------------------------------------------------------
 * Calculate Consensus Frequency
 *
 * V1 rule:
 *
 * At least two valid axes must agree within
 * CONSENSUS_TOLERANCE_HZ.
 *---------------------------------------------------------*/

static bool calculate_consensus_frequency(
    const frequency_metrics_t *x,
    const frequency_metrics_t *y,
    const frequency_metrics_t *z,
    float *consensus_frequency
)
{
    if (
        x == NULL ||
        y == NULL ||
        z == NULL ||
        consensus_frequency == NULL
    )
    {
        return false;
    }


    float frequencies[3];

    float confidences[3];

    int valid_count = 0;


    /*------------------------------------------------------
     * Collect valid frequencies
     *-----------------------------------------------------*/

    if (x->valid_peak)
    {
        frequencies[valid_count] =
            x->dominant_frequency_hz;

        confidences[valid_count] =
            x->confidence_ratio;

        valid_count++;
    }


    if (y->valid_peak)
    {
        frequencies[valid_count] =
            y->dominant_frequency_hz;

        confidences[valid_count] =
            y->confidence_ratio;

        valid_count++;
    }


    if (z->valid_peak)
    {
        frequencies[valid_count] =
            z->dominant_frequency_hz;

        confidences[valid_count] =
            z->confidence_ratio;

        valid_count++;
    }


    /*
     * Need at least two axes.
     */
    if (valid_count < 2)
    {
        return false;
    }


    /*------------------------------------------------------
     * Three-axis case
     *-----------------------------------------------------*/

    if (valid_count == 3)
    {
        float min_f =
            frequencies[0];

        float max_f =
            frequencies[0];


        for (int i = 1; i < 3; i++)
        {
            if (
                frequencies[i] < min_f
            )
            {
                min_f =
                    frequencies[i];
            }


            if (
                frequencies[i] > max_f
            )
            {
                max_f =
                    frequencies[i];
            }
        }


        /*
         * For three axes, require the full span
         * to be no greater than twice the tolerance.
         */
        if (
            (max_f - min_f) <=
            (2.0f * CONSENSUS_TOLERANCE_HZ)
        )
        {
            float weighted_sum =
                0.0f;

            float confidence_sum =
                0.0f;


            for (int i = 0; i < 3; i++)
            {
                weighted_sum +=
                    frequencies[i] *
                    confidences[i];

                confidence_sum +=
                    confidences[i];
            }


            if (
                confidence_sum > 0.0f
            )
            {
                *consensus_frequency =
                    weighted_sum /
                    confidence_sum;

                return true;
            }
        }
    }


    /*------------------------------------------------------
     * Pairwise consensus
     *-----------------------------------------------------*/

    bool pair_found =
        false;

    float best_pair_frequency =
        0.0f;

    float best_pair_confidence =
        0.0f;


    for (
        int i = 0;
        i < valid_count;
        i++
    )
    {
        for (
            int j = i + 1;
            j < valid_count;
            j++
        )
        {
            float difference =
                fabsf(
                    frequencies[i] -
                    frequencies[j]
                );


            if (
                difference <=
                CONSENSUS_TOLERANCE_HZ
            )
            {
                float combined_confidence =
                    confidences[i] +
                    confidences[j];


                if (
                    combined_confidence <= 0.0f
                )
                {
                    continue;
                }


                float pair_frequency =
                    (
                        (
                            frequencies[i] *
                            confidences[i]
                        )
                        +
                        (
                            frequencies[j] *
                            confidences[j]
                        )
                    )
                    /
                    combined_confidence;


                if (
                    !pair_found ||
                    combined_confidence >
                    best_pair_confidence
                )
                {
                    best_pair_frequency =
                        pair_frequency;

                    best_pair_confidence =
                        combined_confidence;

                    pair_found =
                        true;
                }
            }
        }
    }


    if (pair_found)
    {
        *consensus_frequency =
            best_pair_frequency;

        return true;
    }


    return false;
}


/*----------------------------------------------------------
 * Main Application
 *---------------------------------------------------------*/

void app_main(void)
{
    printf("\n");

    printf(
        "========================================\n"
    );

    printf(
        "          OCTARIAN INSIGHT\n"
    );

    printf(
        " INTERRUPT SYNCHRONIZED ACQUISITION V2\n"
    );

    printf(
        "========================================\n\n"
    );


    /*------------------------------------------------------
     * Initialize I2C
     *-----------------------------------------------------*/

    if (
        hal_i2c_init() != ESP_OK
    )
    {
        printf(
            "ERROR: Failed to initialize I2C bus\n"
        );

        return;
    }


    /*------------------------------------------------------
     * Add ADXL345
     *-----------------------------------------------------*/

    if (
        hal_i2c_add_device(
            ADXL345_I2C_ADDR
        ) != ESP_OK
    )
    {
        printf(
            "ERROR: Failed to add ADXL345 device\n"
        );

        return;
    }


    /*------------------------------------------------------
     * Configure ADXL345
     *
     * IMPORTANT:
     *
     * adxl345_init() configures the sensor but
     * does NOT enable measurement mode yet.
     *-----------------------------------------------------*/

    if (
        adxl345_init() != ESP_OK
    )
    {
        printf(
            "ERROR: Failed to initialize ADXL345\n"
        );

        return;
    }


    /*------------------------------------------------------
     * Configure ESP32 DATA_READY interrupt
     *-----------------------------------------------------*/

    if (
        adxl345_configure_data_ready_interrupt(
            ADXL345_INT1_GPIO
        ) != ESP_OK
    )
    {
        printf(
            "ERROR: Failed to configure ADXL345 "
            "DATA_READY GPIO %d\n",
            ADXL345_INT1_GPIO
        );

        return;
    }


    /*------------------------------------------------------
     * Enable measurement only AFTER the ESP32 interrupt
     * path is ready.
     *-----------------------------------------------------*/

    if (
        adxl345_enable_measurement() != ESP_OK
    )
    {
        printf(
            "ERROR: Failed to enable ADXL345 "
            "measurement mode\n"
        );

        return;
    }


    /*------------------------------------------------------
     * Initialize Sensor Manager
     *-----------------------------------------------------*/

    if (
        sensor_manager_init() != ESP_OK
    )
    {
        printf(
            "ERROR: Failed to initialize Sensor Manager\n"
        );

        return;
    }


    /*------------------------------------------------------
     * Initialize DS18B20 temperature sensor
     *-----------------------------------------------------*/

    if (
        ds18b20_init(DS18B20_GPIO) != ESP_OK
    )
    {
        printf(
            "ERROR: Failed to initialize DS18B20\n"
        );

        return;
    }


    printf(
        "DS18B20 initialized on GPIO %d\n",
        DS18B20_GPIO
    );


    printf(
        "System Initialized Successfully\n"
    );


    printf(
        "Sensor ODR          : %d Hz\n",
        SAMPLE_RATE_HZ
    );


    printf(
        "Window Size         : %d fresh samples\n",
        WINDOW_SIZE
    );


    printf(
        "Frequency Resolution: %.2f Hz\n",
        (
            (float)SAMPLE_RATE_HZ /
            (float)WINDOW_SIZE
        )
    );


    printf(
        "Consensus Tolerance : +/- %.2f Hz\n\n",
        CONSENSUS_TOLERANCE_HZ
    );


    /*------------------------------------------------------
     * Sensor sample
     *-----------------------------------------------------*/

    sensor_sample_t sample;

    float temperature_c = 0.0f;


    /*------------------------------------------------------
     * Time-domain metrics
     *-----------------------------------------------------*/

    vibration_metrics_t x_metrics;

    vibration_metrics_t y_metrics;

    vibration_metrics_t z_metrics;

    vibration_metrics_t magnitude_metrics;


    /*------------------------------------------------------
     * Frequency-domain metrics
     *-----------------------------------------------------*/

    frequency_metrics_t x_frequency;

    frequency_metrics_t y_frequency;

    frequency_metrics_t z_frequency;


    /*------------------------------------------------------
     * Consensus
     *-----------------------------------------------------*/

    float consensus_frequency =
        0.0f;

    bool consensus_valid =
        false;


    /*------------------------------------------------------
     * Frequency tracker
     *-----------------------------------------------------*/

    frequency_tracker_t frequency_tracker;


    if (
        frequency_tracker_init(
            &frequency_tracker
        ) != ESP_OK
    )
    {
        printf(
            "ERROR: Failed to initialize Frequency Tracker\n"
        );

        return;
    }


    /*------------------------------------------------------
     * Data logger
     *-----------------------------------------------------*/

    if (
        data_logger_init() != ESP_OK
    )
    {
        printf(
            "ERROR: Failed to initialize Data Logger\n"
        );

        return;
    }


    uint32_t window_number =
        0;


    /*------------------------------------------------------
     * Main acquisition loop
     *-----------------------------------------------------*/

    while (1)
    {
        int collected_samples =
            0;


        int read_errors =
            0;


        int ready_timeouts =
            0;


        int missed_samples =
            0;


        int64_t window_start_us =
            esp_timer_get_time();


        /*--------------------------------------------------
         * Collect one complete window
         *-------------------------------------------------*/

        while (
            collected_samples <
            WINDOW_SIZE
        )
        {
            uint32_t data_ready_events =
                0;


            esp_err_t err =
                adxl345_wait_for_data_ready(
                    DATA_READY_TIMEOUT_MS,
                    &data_ready_events
                );


            /*----------------------------------------------
             * DATA_READY timeout
             *---------------------------------------------*/

            if (
                err == ESP_ERR_TIMEOUT
            )
            {
                ready_timeouts++;

                continue;
            }


            /*----------------------------------------------
             * Other acquisition error
             *---------------------------------------------*/

            if (
                err != ESP_OK
            )
            {
                read_errors++;

                continue;
            }


            /*----------------------------------------------
             * More than one event accumulated.
             *
             * We only perform one current sensor read,
             * so extra events indicate the acquisition
             * task did not service every DATA_READY event
             * individually.
             *---------------------------------------------*/

            if (
                data_ready_events > 1
            )
            {
                missed_samples +=
                    (
                        int
                    )(
                        data_ready_events - 1
                    );
            }


            /*----------------------------------------------
             * Read the current XYZ sample
             *---------------------------------------------*/

            err =
                sensor_manager_read(
                    &sample
                );


            if (
                err != ESP_OK
            )
            {
                read_errors++;

                continue;
            }


            /*----------------------------------------------
             * Store X
             *---------------------------------------------*/

            x_window[
                collected_samples
            ] =
                sample.x_g;


            /*----------------------------------------------
             * Store Y
             *---------------------------------------------*/

            y_window[
                collected_samples
            ] =
                sample.y_g;


            /*----------------------------------------------
             * Store Z
             *---------------------------------------------*/

            z_window[
                collected_samples
            ] =
                sample.z_g;


            /*----------------------------------------------
             * Store magnitude
             *---------------------------------------------*/

            magnitude_window[
                collected_samples
            ] =
                sample.magnitude_g;


            collected_samples++;
        }


        /*--------------------------------------------------
         * End window timing
         *-------------------------------------------------*/

        int64_t window_end_us =
            esp_timer_get_time();


        float window_duration_ms =
            (
                (float)(
                    window_end_us -
                    window_start_us
                )
                /
                1000.0f
            );


        /*==================================================
         * DS18B20 TEMPERATURE
         *==================================================*/

        esp_err_t temperature_result =
            ds18b20_read_temperature(
                &temperature_c
            );


        if (
            temperature_result != ESP_OK
        )
        {
            printf(
                "Temperature Read ERROR\n"
            );
        }
        else
        {
            printf(
                "Temperature       : %.2f °C\n",
                temperature_c
            );
        }


        /*==================================================
         * TIME-DOMAIN ANALYSIS
         *=================================================*/

        esp_err_t x_result =
            health_engine_analyze(
                x_window,
                WINDOW_SIZE,
                &x_metrics
            );


        esp_err_t y_result =
            health_engine_analyze(
                y_window,
                WINDOW_SIZE,
                &y_metrics
            );


        esp_err_t z_result =
            health_engine_analyze(
                z_window,
                WINDOW_SIZE,
                &z_metrics
            );


        esp_err_t magnitude_result =
            health_engine_analyze(
                magnitude_window,
                WINDOW_SIZE,
                &magnitude_metrics
            );


        if (
            x_result != ESP_OK ||
            y_result != ESP_OK ||
            z_result != ESP_OK ||
            magnitude_result != ESP_OK
        )
        {
            printf(
                "ERROR: Health Engine analysis failed\n"
            );

            continue;
        }


        /*==================================================
         * X AXIS SIGNAL PROCESSING
         *=================================================*/

        esp_err_t x_dc_result =
            signal_processing_remove_dc(
                x_window,
                x_dc_removed,
                WINDOW_SIZE
            );


        esp_err_t x_window_result =
            signal_processing_apply_hann_window(
                x_dc_removed,
                x_windowed,
                WINDOW_SIZE
            );


        esp_err_t x_frequency_result =
            signal_processing_analyze_frequency(
                x_windowed,
                WINDOW_SIZE,
                (float)SAMPLE_RATE_HZ,
                &x_frequency
            );


        /*==================================================
         * Y AXIS SIGNAL PROCESSING
         *=================================================*/

        esp_err_t y_dc_result =
            signal_processing_remove_dc(
                y_window,
                y_dc_removed,
                WINDOW_SIZE
            );


        esp_err_t y_window_result =
            signal_processing_apply_hann_window(
                y_dc_removed,
                y_windowed,
                WINDOW_SIZE
            );


        esp_err_t y_frequency_result =
            signal_processing_analyze_frequency(
                y_windowed,
                WINDOW_SIZE,
                (float)SAMPLE_RATE_HZ,
                &y_frequency
            );


        /*==================================================
         * Z AXIS SIGNAL PROCESSING
         *=================================================*/

        esp_err_t z_dc_result =
            signal_processing_remove_dc(
                z_window,
                z_dc_removed,
                WINDOW_SIZE
            );


        esp_err_t z_window_result =
            signal_processing_apply_hann_window(
                z_dc_removed,
                z_windowed,
                WINDOW_SIZE
            );


        esp_err_t z_frequency_result =
            signal_processing_analyze_frequency(
                z_windowed,
                WINDOW_SIZE,
                (float)SAMPLE_RATE_HZ,
                &z_frequency
            );


        /*--------------------------------------------------
         * Verify signal-processing results
         *-------------------------------------------------*/

        if (
            x_dc_result != ESP_OK ||
            x_window_result != ESP_OK ||
            x_frequency_result != ESP_OK ||

            y_dc_result != ESP_OK ||
            y_window_result != ESP_OK ||
            y_frequency_result != ESP_OK ||

            z_dc_result != ESP_OK ||
            z_window_result != ESP_OK ||
            z_frequency_result != ESP_OK
        )
        {
            printf(
                "ERROR: Signal processing failed\n"
            );

            continue;
        }


        /*==================================================
         * CONSENSUS ANALYSIS
         *=================================================*/

        consensus_frequency =
            0.0f;


        consensus_valid =
            calculate_consensus_frequency(
                &x_frequency,
                &y_frequency,
                &z_frequency,
                &consensus_frequency
            );


        /*==================================================
         * FREQUENCY TRACKER
         *=================================================*/

        esp_err_t tracker_result =
            frequency_tracker_update(
                &frequency_tracker,
                consensus_valid,
                consensus_frequency
            );


        if (
            tracker_result != ESP_OK
        )
        {
            printf(
                "ERROR: Frequency Tracker update failed\n"
            );

            continue;
        }


        window_number++;


        /*==================================================
         * DATA LOGGER
         *=================================================*/

        data_logger_record_t log_record =
        {
            .window_number =
                window_number,

            .timestamp_ms =
                esp_timer_get_time() /
                1000,

            .x_metrics =
                x_metrics,

            .y_metrics =
                y_metrics,

            .z_metrics =
                z_metrics,

            .magnitude_metrics =
                magnitude_metrics,

            .x_frequency =
                x_frequency,

            .y_frequency =
                y_frequency,

            .z_frequency =
                z_frequency,

            .consensus_valid =
                consensus_valid,

            .consensus_frequency_hz =
                consensus_frequency,

            .tracker_state =
                frequency_tracker.state,

            .candidate_frequency_hz =
                frequency_tracker.candidate_frequency_hz,

            .stable_frequency_hz =
                frequency_tracker.stable_frequency_hz,

            .confirmation_count =
                frequency_tracker.confirmation_count,

            .missed_window_count =
                frequency_tracker.missed_window_count
        };


        esp_err_t logger_result =
            data_logger_write(
                &log_record
            );


        if (
            logger_result != ESP_OK
        )
        {
            printf(
                "ERROR: Data Logger write failed\n"
            );
        }


        /*==================================================
         * PRINT REPORT
         *=================================================*/

        printf("\n");


        printf(
            "========== VIBRATION REPORT ==========\n"
        );


        printf(
            "X Axis | RMS: %.5f g | Peak: %.5f g\n",
            x_metrics.rms,
            x_metrics.peak
        );


        printf(
            "Y Axis | RMS: %.5f g | Peak: %.5f g\n",
            y_metrics.rms,
            y_metrics.peak
        );


        printf(
            "Z Axis | RMS: %.5f g | Peak: %.5f g\n",
            z_metrics.rms,
            z_metrics.peak
        );


        printf(
            "--------------------------------------\n"
        );


        printf(
            "MAG RMS          : %.5f g\n",
            magnitude_metrics.rms
        );


        printf(
            "MAG Peak         : %.5f g\n",
            magnitude_metrics.peak
        );


        printf(
            "MAG Peak-to-Peak : %.5f g\n",
            magnitude_metrics.peak_to_peak
        );


        printf(
            "MAG Crest Factor : %.3f\n",
            magnitude_metrics.crest_factor
        );


        /*==================================================
         * FREQUENCY REPORT
         *=================================================*/

        printf(
            "--------------------------------------\n"
        );


        printf(
            "========== FREQUENCY ANALYSIS ========\n"
        );


        print_frequency_result(
            "X",
            &x_frequency
        );


        print_frequency_result(
            "Y",
            &y_frequency
        );


        print_frequency_result(
            "Z",
            &z_frequency
        );


        /*==================================================
         * CONSENSUS REPORT
         *=================================================*/

        printf(
            "--------------------------------------\n"
        );


        if (
            consensus_valid
        )
        {
            printf(
                "Consensus Frequency : %.2f Hz\n",
                consensus_frequency
            );


            printf(
                "Consensus Status    : CONFIRMED\n"
            );
        }
        else
        {
            printf(
                "Consensus Frequency : NONE\n"
            );


            printf(
                "Consensus Status    : UNCONFIRMED\n"
            );
        }


        /*==================================================
         * FREQUENCY TRACKER REPORT
         *=================================================*/

        printf(
            "--------------------------------------\n"
        );


        printf(
            "Tracker State       : %s\n",
            frequency_tracker_state_string(
                frequency_tracker.state
            )
        );


        if (
            frequency_tracker.state ==
            FREQUENCY_STATE_CANDIDATE
        )
        {
            printf(
                "Candidate Frequency : %.2f Hz\n",
                frequency_tracker.candidate_frequency_hz
            );


            printf(
                "Confirmations       : %d / 3\n",
                frequency_tracker.confirmation_count
            );


            printf(
                "Missed Windows      : %d / 2\n",
                frequency_tracker.missed_window_count
            );
        }
        else if (
            frequency_tracker.state ==
            FREQUENCY_STATE_STABLE
        )
        {
            printf(
                "Stable Frequency    : %.2f Hz\n",
                frequency_tracker.stable_frequency_hz
            );


            printf(
                "Missed Windows      : %d / 2\n",
                frequency_tracker.missed_window_count
            );
        }


        /*==================================================
         * ACQUISITION STATUS
         *=================================================*/

        printf(
            "--------------------------------------\n"
        );


        printf(
            "Fresh Samples    : %d / %d\n",
            collected_samples,
            WINDOW_SIZE
        );


        printf(
            "I2C Read Errors  : %d\n",
            read_errors
        );


        printf(
            "Ready Timeouts   : %d\n",
            ready_timeouts
        );


        printf(
            "Missed Samples   : %d\n",
            missed_samples
        );


        printf(
            "Window Duration  : %.2f ms\n",
            window_duration_ms
        );


        printf(
            "======================================\n"
        );
    }
}