#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    printf("\n");
    printf("=====================================\n");
    printf(" OCTARIAN INSIGHT\n");
    printf(" ESP-IDF Working!\n");
    printf("=====================================\n\n");

    while (1)
    {
        printf("System Running...\n");
        fflush(stdout);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}