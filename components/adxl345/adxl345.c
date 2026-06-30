/******************************************************************************
 * ADXL345 Driver
 *
 * Provides:
 *  - Device ID
 *  - Measurement Mode
 *  - XYZ Reading
 *
 * Used by Octarian Insight
 ******************************************************************************/

#include "adxl345.h"

#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#define I2C_MASTER_NUM I2C_NUM_0

static const char *TAG = "ADXL345";

/*----------------------------------------------------------
 * Private Functions
 *---------------------------------------------------------*/

static esp_err_t adxl345_read_register(uint8_t reg, uint8_t *data)
{
    return i2c_master_write_read_device(
        I2C_MASTER_NUM,
        ADXL345_I2C_ADDR,
        &reg,
        1,
        data,
        1,
        pdMS_TO_TICKS(100));
}

static esp_err_t adxl345_write_register(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};

    return i2c_master_write_to_device(
        I2C_MASTER_NUM,
        ADXL345_I2C_ADDR,
        buffer,
        sizeof(buffer),
        pdMS_TO_TICKS(100));
}

/*----------------------------------------------------------
 * Public Functions
 *---------------------------------------------------------*/

esp_err_t adxl345_init(void)
{
    uint8_t id;

    esp_err_t err = adxl345_get_device_id(&id);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to communicate with ADXL345");
        return err;
    }

    if (id != 0xE5)
    {
        ESP_LOGE(TAG, "Invalid Device ID: 0x%02X", id);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ADXL345 detected (ID = 0x%02X)", id);

    return adxl345_enable_measurement();
}

esp_err_t adxl345_get_device_id(uint8_t *id)
{
    return adxl345_read_register(REG_DEVID, id);
}

esp_err_t adxl345_enable_measurement(void)
{
    return adxl345_write_register(REG_POWER_CTL, 0x08);
}

esp_err_t adxl345_read_xyz(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t data[6];

    uint8_t start_reg = REG_DATAX0;

    esp_err_t err = i2c_master_write_read_device(
        I2C_MASTER_NUM,
        ADXL345_I2C_ADDR,
        &start_reg,
        1,
        data,
        6,
        pdMS_TO_TICKS(100));

    if (err != ESP_OK)
    {
        return err;
    }

    *x = (int16_t)((data[1] << 8) | data[0]);
    *y = (int16_t)((data[3] << 8) | data[2]);
    *z = (int16_t)((data[5] << 8) | data[4]);

    return ESP_OK;
}