/******************************************************************************
 * ADXL345 Driver
 *
 * Provides:
 *  - Device ID verification
 *  - Measurement mode configuration
 *  - XYZ acceleration reading
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
 * Public Functions
 *---------------------------------------------------------*/

esp_err_t adxl345_init(void)
{
    uint8_t id = 0;

    esp_err_t err = adxl345_get_device_id(&id);

    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to communicate with ADXL345"
        );

        return err;
    }

    if (id != 0xE5)
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


esp_err_t adxl345_get_device_id(uint8_t *id)
{
    return adxl345_read_register(
        REG_DEVID,
        id
    );
}


esp_err_t adxl345_enable_measurement(void)
{
    return adxl345_write_register(
        REG_POWER_CTL,
        0x08
    );
}


esp_err_t adxl345_read_xyz(
    int16_t *x,
    int16_t *y,
    int16_t *z
)
{
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