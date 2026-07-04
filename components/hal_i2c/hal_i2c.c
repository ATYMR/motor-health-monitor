#include "hal_i2c.h"

#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "HAL_I2C";

static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t device_handle = NULL;


esp_err_t hal_i2c_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = HAL_I2C_PORT,
        .sda_io_num = HAL_I2C_SDA_PIN,
        .scl_io_num = HAL_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(
        &bus_config,
        &bus_handle
    );

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "I2C bus initialized successfully");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to initialize I2C bus");
    }

    return err;
}


esp_err_t hal_i2c_add_device(uint8_t device_addr)
{
    if (bus_handle == NULL)
    {
        ESP_LOGE(TAG, "I2C bus is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_addr,
        .scl_speed_hz = HAL_I2C_SPEED,
    };

    esp_err_t err = i2c_master_bus_add_device(
        bus_handle,
        &dev_config,
        &device_handle
    );

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "I2C device added at address 0x%02X", device_addr);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to add I2C device");
    }

    return err;
}


esp_err_t hal_i2c_write(uint8_t reg_addr, uint8_t data)
{
    if (device_handle == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buffer[2] = {
        reg_addr,
        data
    };

    return i2c_master_transmit(
        device_handle,
        buffer,
        sizeof(buffer),
        100
    );
}


esp_err_t hal_i2c_read(
    uint8_t reg_addr,
    uint8_t *data,
    size_t len
)
{
    if (device_handle == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    return i2c_master_transmit_receive(
        device_handle,
        &reg_addr,
        1,
        data,
        len,
        100
    );
}