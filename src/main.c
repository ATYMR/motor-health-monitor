#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_timer.h"

#include "hal_i2c.h"
#include "adxl345.h"
#include "sensor_manager.h"
#include "health_engine.h"
#include "signal_processing.h"


#define SAMPLE_RATE_HZ             100
#define WINDOW_SIZE                100

#define DATA_READY_POLL_MS         1
#define DATA_READY_TIMEOUT_MS      50


/*----------------------------------------------------------
 * Wait for fresh ADXL345 data
 *---------------------------------------------------------*/

static esp_err_t wait_for_data_ready(void)
{
    bool ready = false;

    int elapsed_ms = 0;

    while (elapsed_ms < DATA_READY_TIMEOUT_MS)
    {
        esp_err_t err =
            adxl345_is_data_ready(&ready);

        if (err != ESP_OK)
        {
            return err;
        }

        if (ready)
        {
            return ESP_OK;
        }

        vTaskDelay(
            pdMS_TO_TICKS(DATA_READY_POLL_MS)
        );

        elapsed_ms += DATA_READY_POLL_MS;
    }

    return ESP_ERR_TIMEOUT;
}


/*----------------------------------------------------------
 * Main Application
 *---------------------------------------------------------*/

void app_main(void)
{
    printf("\n");
    printf("========================================\n");
    printf("          OCTARIAN INSIGHT\n");
    printf(" VIBRATION ANALYSIS V1\n");
    printf("========================================\n\n");


    /*------------------------------------------------------
     * Initialize I2C bus
     *-----------------------------------------------------*/

    if (hal_i2c_init() != ESP_OK)
    {
        printf(
            "ERROR: Failed to initialize I2C bus\n"
        );

        return;
    }


    /*------------------------------------------------------
     * Add ADXL345 to I2C bus
     *-----------------------------------------------------*/

    if (hal_i2c_add_device(
            ADXL345_I2C_ADDR
        ) != ESP_OK)
    {
        printf(
            "ERROR: Failed to add ADXL345 device\n"
        );

        return;
    }


    /*------------------------------------------------------
     * Initialize ADXL345
     *-----------------------------------------------------*/

    if (adxl345_init() != ESP_OK)
    {
        printf(
            "ERROR: Failed to initialize ADXL345\n"
        );

        return;
    }


    /*------------------------------------------------------
     * Initialize Sensor Manager
     *-----------------------------------------------------*/

    if (sensor_manager_init() != ESP_OK)
    {
        printf(
            "ERROR: Failed to initialize Sensor Manager\n"
        );

        return;
    }


    printf(
        "System Initialized Successfully\n"
    );

    printf(
        "Sensor ODR  : %d Hz\n",
        SAMPLE_RATE_HZ
    );

    printf(
        "Window Size : %d fresh samples\n",
        WINDOW_SIZE
    );

    printf(
        "Frequency Resolution : %.2f Hz\n\n",
        (float)SAMPLE_RATE_HZ /
        (float)WINDOW_SIZE
    );


    /*------------------------------------------------------
     * Raw sample windows
     *-----------------------------------------------------*/

    float x_window[WINDOW_SIZE];

    float y_window[WINDOW_SIZE];

    float z_window[WINDOW_SIZE];

    float magnitude_window[WINDOW_SIZE];


    /*------------------------------------------------------
     * Signal processing buffers
     *-----------------------------------------------------*/

    float magnitude_dc_removed[WINDOW_SIZE];

    float magnitude_windowed[WINDOW_SIZE];


    /*------------------------------------------------------
     * Sensor sample
     *-----------------------------------------------------*/

    sensor_sample_t sample;


    /*------------------------------------------------------
     * Health Engine result structures
     *-----------------------------------------------------*/

    vibration_metrics_t x_metrics;

    vibration_metrics_t y_metrics;

    vibration_metrics_t z_metrics;

    vibration_metrics_t magnitude_metrics;


    /*------------------------------------------------------
     * Frequency analysis result structure
     *-----------------------------------------------------*/

    frequency_metrics_t frequency_metrics;


    /*------------------------------------------------------
     * Main acquisition loop
     *-----------------------------------------------------*/

    while (1)
    {
        int collected_samples = 0;

        int read_errors = 0;

        int ready_timeouts = 0;


        /*
         * Start window timing.
         */

        int64_t window_start_us =
            esp_timer_get_time();


        /*--------------------------------------------------
         * Collect exactly WINDOW_SIZE fresh samples
         *-------------------------------------------------*/

        while (collected_samples < WINDOW_SIZE)
        {
            esp_err_t err =
                wait_for_data_ready();


            /*
             * DATA_READY timeout
             */

            if (err == ESP_ERR_TIMEOUT)
            {
                ready_timeouts++;

                continue;
            }


            /*
             * Error while checking DATA_READY
             */

            if (err != ESP_OK)
            {
                read_errors++;

                continue;
            }


            /*
             * Fresh sample available
             */

            err = sensor_manager_read(
                &sample
            );


            if (err != ESP_OK)
            {
                read_errors++;

                continue;
            }


            /*
             * Store X axis
             */

            x_window[collected_samples] =
                sample.x_g;


            /*
             * Store Y axis
             */

            y_window[collected_samples] =
                sample.y_g;


            /*
             * Store Z axis
             */

            z_window[collected_samples] =
                sample.z_g;


            /*
             * Store acceleration magnitude
             */

            magnitude_window[collected_samples] =
                sample.magnitude_g;


            collected_samples++;
        }


        /*--------------------------------------------------
         * End window timing
         *-------------------------------------------------*/

        int64_t window_end_us =
            esp_timer_get_time();


        float window_duration_ms =
            (window_end_us - window_start_us)
            / 1000.0f;


        /*--------------------------------------------------
         * Time-domain analysis
         *-------------------------------------------------*/

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


        /*--------------------------------------------------
         * Verify Health Engine results
         *-------------------------------------------------*/

        if (x_result != ESP_OK ||
            y_result != ESP_OK ||
            z_result != ESP_OK ||
            magnitude_result != ESP_OK)
        {
            printf(
                "ERROR: Health Engine analysis failed\n"
            );

            continue;
        }


        /*--------------------------------------------------
         * Remove DC component
         *-------------------------------------------------*/

        esp_err_t dc_result =
            signal_processing_remove_dc(
                magnitude_window,
                magnitude_dc_removed,
                WINDOW_SIZE
            );


        if (dc_result != ESP_OK)
        {
            printf(
                "ERROR: DC removal failed\n"
            );

            continue;
        }


        /*--------------------------------------------------
         * Apply Hann window
         *-------------------------------------------------*/

        esp_err_t window_result =
            signal_processing_apply_hann_window(
                magnitude_dc_removed,
                magnitude_windowed,
                WINDOW_SIZE
            );


        if (window_result != ESP_OK)
        {
            printf(
                "ERROR: Hann window failed\n"
            );

            continue;
        }


        /*--------------------------------------------------
         * Frequency-domain analysis
         *-------------------------------------------------*/

        esp_err_t frequency_result =
            signal_processing_analyze_frequency(
                magnitude_windowed,
                WINDOW_SIZE,
                (float)SAMPLE_RATE_HZ,
                &frequency_metrics
            );


        if (frequency_result != ESP_OK)
        {
            printf(
                "ERROR: Frequency analysis failed\n"
            );

            continue;
        }


        /*--------------------------------------------------
         * Print diagnostic report
         *-------------------------------------------------*/

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


        printf(
            "--------------------------------------\n"
        );


        if (frequency_metrics.valid_peak)
        {
        printf(
        "Dominant Frequency : %.2f Hz\n",
        frequency_metrics.dominant_frequency_hz
        );

        printf(
        "Dominant Amplitude : %.5f g\n",
        frequency_metrics.dominant_amplitude
        );
        }
        else
{
    printf(
        "Dominant Frequency : NONE\n"
    );
}

printf(
    "Frequency Confidence: %.2f\n",
    frequency_metrics.confidence_ratio
);


        printf(
            "--------------------------------------\n"
        );


        printf(
            "Fresh Samples    : %d / %d\n",
            collected_samples,
            WINDOW_SIZE
        );


        printf(
            "Read Errors      : %d\n",
            read_errors
        );


        printf(
            "Ready Timeouts   : %d\n",
            ready_timeouts
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