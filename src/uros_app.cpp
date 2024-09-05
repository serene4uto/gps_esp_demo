#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_intr_alloc.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <rtcm_msgs/msg/message.h>
#include <std_msgs/msg/string.h>

#include <rosidl_runtime_c/string_functions.h>

#include <HardwareSerial.h>

#include "main.h"


HardwareSerial uROS_Serial(2);

rcl_allocator_t uros_allocator;
rclc_support_t uros_support;
rcl_node_t uros_node;
rcl_publisher_t uros_pub;
rcl_subscription_t uros_sub;
rcl_timer_t uros_pub_timer;
rclc_executor_t uros_executor;

QueueHandle_t uros_gnss_queue;
QueueHandle_t uros_rtcm_queue;

TaskHandle_t uros_app_task_handle;

rtcm_msgs__msg__Message uros_sub_allocated_msg;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
static void error_loop() {
  while(1) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

int64_t uros_get_epoch_millis()
{
    if (rmw_uros_epoch_synchronized())
    {
        return rmw_uros_epoch_millis();
    }
    else
    {
        ESP_LOGE(UROS_APP_LOG_TAG,\
            "Cannot get epoch millis. Time is not synchronized with the agent.");
        return -1; 
    }
}

static void uros_pub_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;
    if (timer == NULL) {
        ESP_LOGE(UROS_APP_LOG_TAG, "Timer is NULL");
        return;
    }

    gnss_data_t msg;

    if(xQueueReceive(uros_gnss_queue, &msg, 0) != pdTRUE)
    {
        // ESP_LOGE(UROS_APP_LOG_TAG, "Failed to receive GNSS data from the queue");
        return;
    }

    sensor_msgs__msg__NavSatFix fix_msg = {};
    int64_t epoch_millis = rmw_uros_epoch_millis();
    fix_msg.header.stamp.sec = epoch_millis / 1000;
    fix_msg.header.stamp.nanosec = (epoch_millis % 1000) * 1e6;

    // Initialize strings to avoid any garbage data
    rosidl_runtime_c__String__init(&fix_msg.header.frame_id);
    rosidl_runtime_c__String__assign(&fix_msg.header.frame_id, "gps");

    fix_msg.latitude = msg.latitude;
    fix_msg.longitude = msg.longitude;
    fix_msg.altitude = msg.altitude;

    //TODO: Check again these values
    fix_msg.position_covariance_type = 0;
    memset(&fix_msg.position_covariance, 0, sizeof(fix_msg.position_covariance));

    
    RCSOFTCHECK(rcl_publish(&uros_pub, &fix_msg, NULL));

}

static void uros_sub_callback(const void * msgin)
{
    const rtcm_msgs__msg__Message * msg = (const rtcm_msgs__msg__Message *)msgin;
    // ESP_LOGI(UROS_APP_LOG_TAG, "Received message: %s", msg->message.data);
    rtcm_data_t rtcm_data;
    rtcm_data.size = msg->message.size;
    memcpy(rtcm_data.data, msg->message.data, msg->message.size);
    xQueueSend(uros_rtcm_queue, &rtcm_data, 0);
}

void uros_app_init()
{
    esp_log_level_set(UROS_APP_LOG_TAG, UROS_APP_LOG_LEVEL);
    
    // Init HW
    uROS_Serial.begin(UROS_HW_UART_BAUDRATE, SERIAL_8N1, UROS_HW_UART_RX, UROS_HW_UART_TX);
    while (!uROS_Serial)
    {
        ESP_LOGE(UROS_APP_LOG_TAG, "Serial port not available");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }


    // Init uROS
    set_microros_serial_transports(uROS_Serial);

    // ping the agent to test the connection
    ESP_LOGI(UROS_APP_LOG_TAG, "Checking micro-ROS agent...");
    rmw_ret_t ping_result = rmw_uros_ping_agent(1000, 5);
    if (RMW_RET_OK != ping_result)
    {
        ESP_LOGE(UROS_APP_LOG_TAG,\
            "micro-ROS agent not found. Please check and do reset to try again.");
        error_loop();
    }
    ESP_LOGI(UROS_APP_LOG_TAG, "micro-ROS agent found. Initializing...");

    // Initialize the micro-ROS
    // create allocator
    uros_allocator = rcl_get_default_allocator();    
    // create init_options
    RCCHECK(rclc_support_init(&uros_support,0, NULL, &uros_allocator));
    // create node
    RCCHECK(rclc_node_init_default(&uros_node,\
                                UROS_NODE_NAME,\
                                UROS_NODE_NAMESPACE,\
                                &uros_support));   
    // create publisher
    RCCHECK(rclc_publisher_init_default(&uros_pub,\
                                        &uros_node,\
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),\
                                        UROS_PUB_TOPIC_NAME));


    // create subscriber
    RCCHECK(rclc_subscription_init_default(&uros_sub,\
                                            &uros_node,\
                                            ROSIDL_GET_MSG_TYPE_SUPPORT(rtcm_msgs, msg, Message),\
                                            UROS_SUB_TOPIC_NAME));

    // create timer
    RCCHECK(rclc_timer_init_default(
        &uros_pub_timer,\
        &uros_support,\
        RCL_MS_TO_NS(UROS_PUB_TIMER_PERIOD_MS),\
        uros_pub_timer_callback)); 

    // create executor
    RCCHECK(rclc_executor_init(&uros_executor, &uros_support.context, 2, &uros_allocator));

    // add timer to executor
    RCCHECK(rclc_executor_add_timer(&uros_executor, &uros_pub_timer));

    // memory allocation for the rtcm message
    uros_sub_allocated_msg.message.capacity = UROS_SUB_RTCM_MSG_DATA_SIZE;
    uros_sub_allocated_msg.message.data = (uint8_t *)malloc(UROS_SUB_RTCM_MSG_DATA_SIZE * sizeof(uint8_t));
    uros_sub_allocated_msg.message.size = 0;
    
    // add subscription to executor
    RCCHECK(rclc_executor_add_subscription(&uros_executor,\
                                        &uros_sub,\
                                        &uros_sub_allocated_msg,\
                                        &uros_sub_callback,\
                                        ON_NEW_DATA));

    vTaskDelay(100 / portTICK_PERIOD_MS);

    // synchronize time with the agent
    if (RMW_RET_OK != rmw_uros_sync_session(1000))
    {
        ESP_LOGE(UROS_APP_LOG_TAG,\
            "Failed to synchronize time with the agent. Please check and do reset to try again.");
        error_loop();
    }

    // Create the queues
    uros_gnss_queue = xQueueCreate(UROS_GNSS_QUEUE_SIZE, sizeof(gnss_data_t));
    uros_rtcm_queue = xQueueCreate(UROS_RTCM_QUEUE_SIZE, sizeof(rtcm_data_t));

    ESP_LOGI(UROS_APP_LOG_TAG, "uROS APP initialized");
}

void uros_app_main_task(void *arg)
{
    ESP_LOGI(UROS_APP_LOG_TAG, "uROS APP task started");
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // RCSOFTCHECK(rclc_executor_spin(&uros_executor));

    if(rclc_executor_spin(&uros_executor) != RCL_RET_OK)
    {
        ESP_LOGE(UROS_APP_LOG_TAG, "Failed to spin the executor");
        error_loop();
    }

    while(1)
    {
        // ESP_LOGI(UROS_APP_LOG_TAG, "uROS APP task running");
        if (rclc_executor_spin_some(&uros_executor, RCL_MS_TO_NS(100)) != RCL_RET_OK)
        {
            ESP_LOGE(UROS_APP_LOG_TAG, "Failed to spin the executor");
            error_loop();
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void uros_app_start()
{
    xTaskCreate(uros_app_main_task,\
                "uros_app_main_task",\
                UROS_APP_TASK_STACK_SIZE*10,\
                NULL,\
                UROS_APP_TASK_PRIORITY,\
                &uros_app_task_handle);
}