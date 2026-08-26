#ifndef DS18B20_H
#define DS18B20_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DS18B20_DEFAULT_GPIO 4

/**
 * Initialize the DS18B20 sensor.
 *
 * @param gpio_num GPIO connected to DS18B20 DATA.
 *
 * @return ESP_OK on success.
 */
esp_err_t ds18b20_init(int gpio_num);

/**
 * Read temperature from DS18B20.
 *
 * @param temperature_c Pointer to temperature in Celsius.
 *
 * @return ESP_OK on success.
 */
esp_err_t ds18b20_read_temperature(float *temperature_c);

#ifdef __cplusplus
}
#endif

#endif