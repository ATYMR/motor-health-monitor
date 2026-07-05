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
 *
 * Used by Octarian Insight
 ******************************************************************************/

#include "adxl345.h"
#include "hal_i2c.h"

#include "esp_log.h"


static const char *TAG = "ADXL345";


/*----------------------------------------------------------
 * Private Functions
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
        adxl345_get_device_id(&id);


    /*
     * Verify communication
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
     * Verify Device ID
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
     * Configure:
     *
     * FULL_RES = enabled
     * Range    = +/-2g
     */
    err = adxl345_set_range(
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
     * Configure output data rate
     */
    err = adxl345_set_data_rate(
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
     * Enable DATA_READY interrupt source
     */
    err = adxl345_enable_data_ready();


    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to enable DATA_READY"
        );

        return err;
    }


    ESP_LOGI(
        TAG,
        "DATA_READY enabled"
    );


    /*
     * Enable measurement mode
     */
    err = adxl345_enable_measurement();


    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to enable measurement mode"
        );

        return err;
    }


    ESP_LOGI(
        TAG,
        "Measurement mode enabled"
    );


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
    return adxl345_write_register(
        REG_POWER_CTL,
        ADXL345_MEASURE_MODE
    );
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
    return adxl345_write_register(
        REG_INT_ENABLE,
        ADXL345_INT_DATA_READY
    );
}


/*----------------------------------------------------------
 * Check DATA_READY Status
 *---------------------------------------------------------*/

esp_err_t adxl345_is_data_ready(
    bool *ready
)
{
    if (ready == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }


    uint8_t interrupt_source = 0;


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
        (interrupt_source &
         ADXL345_INT_DATA_READY) != 0;


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
    if (x == NULL ||
        y == NULL ||
        z == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }


    uint8_t data[6];


    esp_err_t err = hal_i2c_read(
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
     * ADXL345 data order:
     *
     * DATAX0 = X low byte
     * DATAX1 = X high byte
     *
     * DATAY0 = Y low byte
     * DATAY1 = Y high byte
     *
     * DATAZ0 = Z low byte
     * DATAZ1 = Z high byte
     */


    *x = (int16_t)(
        ((uint16_t)data[1] << 8) |
        data[0]
    );


    *y = (int16_t)(
        ((uint16_t)data[3] << 8) |
        data[2]
    );


    *z = (int16_t)(
        ((uint16_t)data[5] << 8) |
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
    if (x_g == NULL ||
        y_g == NULL ||
        z_g == NULL)
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
        x_raw * ADXL345_SCALE_FACTOR;


    *y_g =
        y_raw * ADXL345_SCALE_FACTOR;


    *z_g =
        z_raw * ADXL345_SCALE_FACTOR;


    return ESP_OK;
}