#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_intr_alloc.h>

#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>

#include "main.h"

#define GNSS_HW_WIRE Wire1

TaskHandle_t gnss_app_task_handle;
SFE_UBLOX_GNSS gnss_dev;
gnss_data_t gnss_data;

void gnss_app_init()
{
    // Init HW
    GNSS_HW_WIRE.setPins(GNSS_HW_I2C_SDA, GNSS_HW_I2C_SCL);
    GNSS_HW_WIRE.begin();

    esp_log_level_set(GNSS_APP_LOG_TAG, GNSS_APP_LOG_LEVEL);

    // Init GNSS
    while (gnss_dev.begin(GNSS_HW_WIRE, GNSS_HW_I2C_ADDRESS) == false) //Connect to the u-blox module using our custom port and address
    {
        ESP_LOGE(GNSS_APP_LOG_TAG, "u-blox GNSS not detected. Retrying...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Configure GNSS
    gnss_dev.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    // gnss_dev.setI2CInput(COM_TYPE_RTCM3);

    ESP_LOGI(GNSS_APP_LOG_TAG, "GNSS APP initialized");
}

void gnss_app_main_task(void *arg)
{
    ESP_LOGI(GNSS_APP_LOG_TAG, "GNSS APP task started");

    while (1)
    {
        // Check if there is any RTCM message
        rtcm_data_t rtcm_data;
        if (xQueueReceive(uros_rtcm_queue, &rtcm_data, 0) == pdTRUE)
        {
            // ESP_LOGI(GNSS_APP_LOG_TAG, "Received RTCM data length: %d", rtcm_data.size); 
            gnss_dev.pushRawData(rtcm_data.data, rtcm_data.size); // Push the RTCM data to the GNSS module
        }
        
        if (gnss_dev.getPVT() == true)
        {
            gnss_data.latitude = (double)gnss_dev.getLatitude() / 10000000.0;
            gnss_data.longitude = (double)gnss_dev.getLongitude() / 10000000.0;
            gnss_data.altitude = (double)gnss_dev.getAltitudeMSL() / 1000.0; // Altitude above Mean Sea Level

            xQueueOverwrite(uros_gnss_queue, &gnss_data);

            // ESP_LOGI(GNSS_APP_LOG_TAG, "Lat: %f, Long: %f, Alt: %f", gnss_data.latitude, gnss_data.longitude, gnss_data.altitude);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void gnss_app_start()
{
    xTaskCreate(gnss_app_main_task,\
                "gnss_app_main_task",\
                GNSS_APP_TASK_STACK_SIZE,\
                NULL,\
                GNSS_APP_TASK_PRIORITY,\
                &gnss_app_task_handle);
}

