#include "adxl345.h"

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_NUM I2C_NUM_0

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