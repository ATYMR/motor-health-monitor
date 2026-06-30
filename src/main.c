#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "esp_err.h"

#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_SDA_IO    GPIO_NUM_21
#define I2C_MASTER_SCL_IO    GPIO_NUM_22
#define I2C_MASTER_FREQ_HZ   100000

#define ADXL345_ADDR         0x53
#define REG_DEVID            0x00
#define REG_POWER_CTL        0x2D
#define REG_DATAX0           0x32
#define REG_DATAX1           0x33
#define REG_DATAY0           0x34
#define REG_DATAY1           0x35
#define REG_DATAZ0           0x36
#define REG_DATAZ1           0x37

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

esp_err_t adxl345_read_register(uint8_t reg, uint8_t *data)
{
    return i2c_master_write_read_device(
        I2C_MASTER_NUM,
        ADXL345_ADDR,
        &reg,
        1,
        data,
        1,
        pdMS_TO_TICKS(100));
}

esp_err_t adxl345_write_register(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};

    return i2c_master_write_to_device(
        I2C_MASTER_NUM,
        ADXL345_ADDR,
        buffer,
        sizeof(buffer),
        pdMS_TO_TICKS(100)
    );
}

esp_err_t adxl345_read_xyz(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t data[6];

    esp_err_t err = i2c_master_write_read_device(
        I2C_MASTER_NUM,
        ADXL345_ADDR,
        (uint8_t[]){REG_DATAX0},
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

void app_main(void)
{
    printf("\n=============================\n");
    printf(" OCTARIAN INSIGHT\n");
    printf("=============================\n");

    i2c_master_init();

    uint8_t devid = 0;

    // Read Device ID
    if (adxl345_read_register(REG_DEVID, &devid) == ESP_OK)
    {
        printf("Device ID : 0x%02X\n", devid);

        // Enable Measurement Mode
        if (adxl345_write_register(REG_POWER_CTL, 0x08) == ESP_OK)
        {
            printf("ADXL345 Measurement Mode Enabled!\n");
        }
        else
        {
            printf("Failed to enable Measurement Mode!\n");
        }
    }
    else
    {
        printf("Failed to read Device ID!\n");
    }

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