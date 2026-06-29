#ifndef HAL_I2C_H
#define HAL_I2C_H

#include "driver/i2c_master.h"
#include "esp_err.h"

#define I2C_PORT       I2C_NUM_0
#define I2C_SDA_PIN    GPIO_NUM_21
#define I2C_SCL_PIN    GPIO_NUM_22
#define I2C_SPEED      100000

esp_err_t hal_i2c_init(void);

#endif