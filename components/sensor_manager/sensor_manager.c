#include "sensor_manager.h"

#include <math.h>

#include "adxl345.h"
#include "esp_log.h"


static const char *TAG = "SENSOR_MANAGER";

esp_err_t sensor_manager_init(void)
{
    ESP_LOGI(TAG, "Sensor Manager initialized");

    return ESP_OK;
}

esp_err_t sensor_manager_read(
    sensor_sample_t *sample
)
{
    if (sample == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = adxl345_read_acceleration(
        &sample->x_g,
        &sample->y_g,
        &sample->z_g
    );

    if (err != ESP_OK)
    {
        return err;
    }

    sample->magnitude_g = sqrtf(
        (sample->x_g * sample->x_g) +
        (sample->y_g * sample->y_g) +
        (sample->z_g * sample->z_g)
    );

    return ESP_OK;
}