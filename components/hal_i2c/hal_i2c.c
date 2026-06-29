#include "hal_i2c.h"

#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "HAL_I2C";

static i2c_master_bus_handle_t bus_handle = NULL;

esp_err_t hal_i2c_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "I2C initialized successfully");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to initialize I2C");
    }

    return err;
}