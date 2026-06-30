#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "esp_err.h"

#include "adxl345.h"

#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_SDA_IO    GPIO_NUM_21
#define I2C_MASTER_SCL_IO    GPIO_NUM_22
#define I2C_MASTER_FREQ_HZ   100000

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

void app_main(void)
{
    printf("\n=============================\n");
    printf(" OCTARIAN INSIGHT\n");
    printf("=============================\n");

    i2c_master_init();

    if (adxl345_init() != ESP_OK)
    {
        printf("Failed to initialize ADXL345!\n");
        return;
    }

    printf("ADXL345 Initialized Successfully!\n");

    int16_t x, y, z;

    while (1)
    {
        if (adxl345_read_xyz(&x, &y, &z) == ESP_OK)
        {
            printf("X: %6d  Y: %6d  Z: %6d\n", x, y, z);
        }
        else
        {
            printf("Read Error!\n");
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}