#ifndef ADXL345_H
#define ADXL345_H

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"


/*----------------------------------------------------------
 * Device Configuration
 *---------------------------------------------------------*/

#define ADXL345_I2C_ADDR         0x53
#define ADXL345_DEVICE_ID        0xE5


/*----------------------------------------------------------
 * Register Addresses
 *---------------------------------------------------------*/

#define REG_DEVID                0x00

#define REG_BW_RATE              0x2C
#define REG_POWER_CTL            0x2D

#define REG_INT_ENABLE           0x2E
#define REG_INT_MAP              0x2F
#define REG_INT_SOURCE           0x30

#define REG_DATA_FORMAT          0x31

#define REG_DATAX0               0x32
#define REG_DATAX1               0x33

#define REG_DATAY0               0x34
#define REG_DATAY1               0x35

#define REG_DATAZ0               0x36
#define REG_DATAZ1               0x37


/*----------------------------------------------------------
 * Configuration Values
 *---------------------------------------------------------*/

/*
 * POWER_CTL register
 */
#define ADXL345_MEASURE_MODE     0x08


/*
 * DATA_FORMAT register
 */
#define ADXL345_FULL_RES         0x08

#define ADXL345_RANGE_2G         0x00
#define ADXL345_RANGE_4G         0x01
#define ADXL345_RANGE_8G         0x02
#define ADXL345_RANGE_16G        0x03


/*
 * BW_RATE register
 */
#define ADXL345_RATE_100HZ       0x0A
#define ADXL345_RATE_200HZ       0x0B
#define ADXL345_RATE_400HZ       0x0C
#define ADXL345_RATE_800HZ       0x0D


/*
 * Interrupt bits
 */
#define ADXL345_INT_DATA_READY   0x80


/*
 * Full-resolution scale factor
 *
 * Approximately 3.9 mg/LSB
 */
#define ADXL345_SCALE_FACTOR     0.0039f


/*----------------------------------------------------------
 * Public API
 *---------------------------------------------------------*/

esp_err_t adxl345_init(void);


esp_err_t adxl345_get_device_id(
    uint8_t *id
);


esp_err_t adxl345_enable_measurement(void);


esp_err_t adxl345_set_range(
    uint8_t range
);


esp_err_t adxl345_set_data_rate(
    uint8_t rate
);


esp_err_t adxl345_enable_data_ready(void);


esp_err_t adxl345_is_data_ready(
    bool *ready
);


esp_err_t adxl345_read_xyz(
    int16_t *x,
    int16_t *y,
    int16_t *z
);


esp_err_t adxl345_read_acceleration(
    float *x_g,
    float *y_g,
    float *z_g
);


#endif