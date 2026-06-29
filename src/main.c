#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define I2C_MASTER_FREQ_HZ 100000

static const char *TAG = "I2C_SCAN";

void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

void i2c_scan(void)
{
    printf("\nScanning I2C Bus...\n");

    for (uint8_t address = 1; address < 127; address++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(
            I2C_MASTER_NUM,
            cmd,
            pdMS_TO_TICKS(100));

        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK)
        {
            printf("Found device at 0x%02X\n", address);
        }
    }

    printf("Scan Complete\n\n");
}

void app_main(void)
{
    printf("\n=============================\n");
    printf(" OCTARIAN INSIGHT\n");
    printf("=============================\n");

    i2c_master_init();

    while (1)
    {
        i2c_scan();
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}