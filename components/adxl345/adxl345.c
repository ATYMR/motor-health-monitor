/******************************************************************************
 * ADXL345 Driver
 *
 * Provides:
 *  - Device ID verification
 *  - Measurement range configuration
 *  - Data rate configuration
 *  - DATA_READY support
 *  - Measurement mode configuration
 *  - XYZ raw acceleration reading
 *  - Acceleration conversion to g
 *  - ESP32 DATA_READY interrupt support
 *
 * Used by Octarian Insight
 ******************************************************************************/

#include "adxl345.h"
#include "hal_i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_log.h"


static const char *TAG = "ADXL345";


/*
 * Task that consumes DATA_READY notifications.
 */
static TaskHandle_t data_ready_task_handle = NULL;


/*
 * GPIO connected to ADXL345 INT1.
 */
static gpio_num_t data_ready_gpio = GPIO_NUM_NC;


/*----------------------------------------------------------
 * DATA_READY ISR
 *---------------------------------------------------------*/

static void IRAM_ATTR adxl345_data_ready_isr(
    void *arg
)
{
    (void)arg;

    /*
     * Safety check.
     *
     * The ISR should never normally run before
     * the task handle is initialized.
     */
    if (data_ready_task_handle == NULL)
    {
        return;
    }

    BaseType_t higher_priority_task_woken =
        pdFALSE;


    /*
     * Increment the task notification count.
     *
     * One DATA_READY interrupt = one notification.
     */
    vTaskNotifyGiveFromISR(
        data_ready_task_handle,
        &higher_priority_task_woken
    );


    if (higher_priority_task_woken)
    {
        portYIELD_FROM_ISR();
    }
}


/*----------------------------------------------------------
 * Private register helpers
 *---------------------------------------------------------*/

static esp_err_t adxl345_read_register(
    uint8_t reg,
    uint8_t *data
)
{
    return hal_i2c_read(
        reg,
        data,
        1
    );
}


static esp_err_t adxl345_write_register(
    uint8_t reg,
    uint8_t value
)
{
    return hal_i2c_write(
        reg,
        value
    );
}


/*----------------------------------------------------------
 * Initialization
 *---------------------------------------------------------*/

esp_err_t adxl345_init(void)
{
    uint8_t id = 0;

    esp_err_t err =
        adxl345_get_device_id(
            &id
        );


    /*
     * Verify communication.
     */
    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to communicate with ADXL345"
        );

        return err;
    }


    /*
     * Verify Device ID.
     */
    if (id != ADXL345_DEVICE_ID)
    {
        ESP_LOGE(
            TAG,
            "Invalid Device ID: 0x%02X",
            id
        );

        return ESP_FAIL;
    }


    ESP_LOGI(
        TAG,
        "ADXL345 detected (ID = 0x%02X)",
        id
    );


    /*
     * Configure full-resolution ±2 g mode.
     */
    err =
        adxl345_set_range(
            ADXL345_FULL_RES |
            ADXL345_RANGE_2G
        );


    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to configure measurement range"
        );

        return err;
    }


    ESP_LOGI(
        TAG,
        "Full resolution mode, +/-2g range configured"
    );


    /*
     * Configure output data rate.
     */
    err =
        adxl345_set_data_rate(
            ADXL345_RATE_100HZ
        );


    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to configure data rate"
        );

        return err;
    }


    ESP_LOGI(
        TAG,
        "Data rate configured to 100 Hz"
    );


    /*
     * Configure DATA_READY routing and enable
     * the DATA_READY interrupt source.
     *
     * DATA_READY -> INT1
     */
    err =
        adxl345_enable_data_ready();


    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to configure DATA_READY"
        );

        return err;
    }


    ESP_LOGI(
        TAG,
        "DATA_READY enabled and routed to INT1"
    );


    /*
     * IMPORTANT:
     *
     * Measurement mode is intentionally NOT enabled here.
     *
     * main.c must first configure the ESP32 GPIO interrupt
     * and then explicitly enable measurement.
     *
     * This prevents DATA_READY events from being generated
     * before the ESP32 interrupt path is ready.
     */

    return ESP_OK;
}


/*----------------------------------------------------------
 * Device ID
 *---------------------------------------------------------*/

esp_err_t adxl345_get_device_id(
    uint8_t *id
)
{
    if (id == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }


    return adxl345_read_register(
        REG_DEVID,
        id
    );
}


/*----------------------------------------------------------
 * Measurement Mode
 *---------------------------------------------------------*/

esp_err_t adxl345_enable_measurement(void)
{
    esp_err_t err =
        adxl345_write_register(
            REG_POWER_CTL,
            ADXL345_MEASURE_MODE
        );


    if (err == ESP_OK)
    {
        ESP_LOGI(
            TAG,
            "Measurement mode enabled"
        );
    }


    return err;
}


/*----------------------------------------------------------
 * Range Configuration
 *---------------------------------------------------------*/

esp_err_t adxl345_set_range(
    uint8_t range
)
{
    return adxl345_write_register(
        REG_DATA_FORMAT,
        range
    );
}


/*----------------------------------------------------------
 * Data Rate Configuration
 *---------------------------------------------------------*/

esp_err_t adxl345_set_data_rate(
    uint8_t rate
)
{
    return adxl345_write_register(
        REG_BW_RATE,
        rate
    );
}


/*----------------------------------------------------------
 * DATA_READY Configuration
 *---------------------------------------------------------*/

esp_err_t adxl345_enable_data_ready(void)
{
    esp_err_t err;


    /*
     * Explicitly route DATA_READY to INT1.
     *
     * For ADXL345:
     *
     * INT_MAP bit = 0
     * -> interrupt goes to INT1
     */
    err =
        adxl345_write_register(
            REG_INT_MAP,
            0x00
        );


    if (err != ESP_OK)
    {
        return err;
    }


    /*
     * Enable DATA_READY interrupt.
     */
    return adxl345_write_register(
        REG_INT_ENABLE,
        ADXL345_INT_DATA_READY
    );
}


/*----------------------------------------------------------
 * DATA_READY GPIO Interrupt
 *---------------------------------------------------------*/

esp_err_t adxl345_configure_data_ready_interrupt(
    gpio_num_t gpio_num
)
{
    if (gpio_num < 0)
    {
        return ESP_ERR_INVALID_ARG;
    }


    /*
     * Remember the task that will wait for the
     * DATA_READY notifications.
     */
    data_ready_task_handle =
        xTaskGetCurrentTaskHandle();


    if (data_ready_task_handle == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }


    /*
     * GPIO34 is input-only.
     *
     * ADXL345 DATA_READY is an input signal
     * to the ESP32, therefore this is appropriate.
     */
    gpio_config_t config =
    {
        .pin_bit_mask =
            (1ULL << gpio_num),

        .mode =
            GPIO_MODE_INPUT,

        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .intr_type =
            GPIO_INTR_POSEDGE
    };


    esp_err_t err =
        gpio_config(
            &config
        );


    if (err != ESP_OK)
    {
        return err;
    }


    /*
     * Install the shared GPIO ISR service.
     */
    err =
        gpio_install_isr_service(
            0
        );


    if (
        err != ESP_OK &&
        err != ESP_ERR_INVALID_STATE
    )
    {
        return err;
    }


    /*
     * Attach our DATA_READY ISR.
     */
    err =
        gpio_isr_handler_add(
            gpio_num,
            adxl345_data_ready_isr,
            NULL
        );


    if (err != ESP_OK)
    {
        return err;
    }


    data_ready_gpio =
        gpio_num;


    ESP_LOGI(
        TAG,
        "DATA_READY interrupt configured on GPIO %d",
        gpio_num
    );


    return ESP_OK;
}


/*----------------------------------------------------------
 * Wait for DATA_READY
 *---------------------------------------------------------*/

esp_err_t adxl345_wait_for_data_ready(
    uint32_t timeout_ms,
    uint32_t *event_count
)
{
    if (
        data_ready_task_handle == NULL ||
        data_ready_gpio == GPIO_NUM_NC ||
        event_count == NULL
    )
    {
        return ESP_ERR_INVALID_STATE;
    }


    TickType_t timeout_ticks =
        pdMS_TO_TICKS(
            timeout_ms
        );


    /*
     * Protect against very small timeout
     * converting to zero ticks.
     */
    if (
        timeout_ms > 0 &&
        timeout_ticks == 0
    )
    {
        timeout_ticks = 1;
    }


    /*
     * pdTRUE clears the notification value
     * after retrieving it.
     *
     * The returned value is the number of
     * DATA_READY events accumulated.
     */
    uint32_t events =
        ulTaskNotifyTake(
            pdTRUE,
            timeout_ticks
        );


    if (events == 0)
    {
        return ESP_ERR_TIMEOUT;
    }


    *event_count =
        events;


    return ESP_OK;
}


/*----------------------------------------------------------
 * Check DATA_READY Status
 *
 * Retained for diagnostic/polling use.
 * Not used by the interrupt-driven acquisition path.
 *---------------------------------------------------------*/

esp_err_t adxl345_is_data_ready(
    bool *ready
)
{
    if (ready == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }


    uint8_t interrupt_source =
        0;


    esp_err_t err =
        adxl345_read_register(
            REG_INT_SOURCE,
            &interrupt_source
        );


    if (err != ESP_OK)
    {
        return err;
    }


    *ready =
        (
            interrupt_source &
            ADXL345_INT_DATA_READY
        ) != 0;


    return ESP_OK;
}


/*----------------------------------------------------------
 * Read Raw XYZ Data
 *---------------------------------------------------------*/

esp_err_t adxl345_read_xyz(
    int16_t *x,
    int16_t *y,
    int16_t *z
)
{
    if (
        x == NULL ||
        y == NULL ||
        z == NULL
    )
    {
        return ESP_ERR_INVALID_ARG;
    }


    uint8_t data[6];


    /*
     * Burst read:
     *
     * DATAX0 ... DATAZ1
     */
    esp_err_t err =
        hal_i2c_read(
            REG_DATAX0,
            data,
            sizeof(data)
        );


    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to read XYZ data"
        );

        return err;
    }


    /*
     * X
     */
    *x =
        (int16_t)(
            (
                (uint16_t)data[1] << 8
            )
            |
            data[0]
        );


    /*
     * Y
     */
    *y =
        (int16_t)(
            (
                (uint16_t)data[3] << 8
            )
            |
            data[2]
        );


    /*
     * Z
     */
    *z =
        (int16_t)(
            (
                (uint16_t)data[5] << 8
            )
            |
            data[4]
        );


    return ESP_OK;
}


/*----------------------------------------------------------
 * Read Acceleration in g
 *---------------------------------------------------------*/

esp_err_t adxl345_read_acceleration(
    float *x_g,
    float *y_g,
    float *z_g
)
{
    if (
        x_g == NULL ||
        y_g == NULL ||
        z_g == NULL
    )
    {
        return ESP_ERR_INVALID_ARG;
    }


    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;


    esp_err_t err =
        adxl345_read_xyz(
            &x_raw,
            &y_raw,
            &z_raw
        );


    if (err != ESP_OK)
    {
        return err;
    }


    *x_g =
        x_raw *
        ADXL345_SCALE_FACTOR;


    *y_g =
        y_raw *
        ADXL345_SCALE_FACTOR;


    *z_g =
        z_raw *
        ADXL345_SCALE_FACTOR;


    return ESP_OK;
}