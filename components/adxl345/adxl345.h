#ifndef ADXL345_H
#define ADXL345_H

#include <stdint.h>
#include "esp_err.h"

#define ADXL345_I2C_ADDR     0x53

#define REG_DEVID            0x00
#define REG_POWER_CTL        0x2D

#define REG_DATAX0           0x32
#define REG_DATAX1           0x33

#define REG_DATAY0           0x34
#define REG_DATAY1           0x35

#define REG_DATAZ0           0x36
#define REG_DATAZ1           0x37

esp_err_t adxl345_init(void);

esp_err_t adxl345_get_device_id(uint8_t *id);

esp_err_t adxl345_enable_measurement(void);

esp_err_t adxl345_read_xyz(
    int16_t *x,
    int16_t *y,
    int16_t *z);

#endif