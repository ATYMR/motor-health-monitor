#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"

#include "hal_i2c.h"
#include "adxl345.h"


void app_main(void)
{
    printf("\n=============================\n");
    printf(" OCTARIAN INSIGHT\n");
    printf("=============================\n");

    /* Initialize I2C bus */
    if (hal_i2c_init() != ESP_OK)
    {
        printf("Failed to initialize I2C bus!\n");
        return;
    }

    /* Add ADXL345 to I2C bus */
    if (hal_i2c_add_device(ADXL345_I2C_ADDR) != ESP_OK)
    {
        printf("Failed to add ADXL345 device!\n");
        return;
    }

    /* Initialize ADXL345 */
    if (adxl345_init() != ESP_OK)
    {
        printf("Failed to initialize ADXL345!\n");
        return;
    }

    printf("ADXL345 Initialized Successfully!\n");

    int16_t x;
    int16_t y;
    int16_t z;

    while (1)
    {
        if (adxl345_read_xyz(&x, &y, &z) == ESP_OK)
        {
            printf(
                "X: %6d  Y: %6d  Z: %6d\n",
                x,
                y,
                z
            );
        }
        else
        {
            printf("Read Error!\n");
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}