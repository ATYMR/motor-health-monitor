#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "esp_err.h"

typedef struct
{
    float x_g;
    float y_g;
    float z_g;
    float magnitude_g;
} sensor_sample_t;

esp_err_t sensor_manager_init(void);

esp_err_t sensor_manager_read(
    sensor_sample_t *sample
);

#endif