#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "main.h"


void main_app()
{
    uros_app_init();
    gnss_app_init();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    uros_app_start();
    gnss_app_start();
    
    while (1)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}