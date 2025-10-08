/**
 * @file main.cpp
 * @brief Main application entry point for the e-bike controller.
 *
 * This file initializes all system components (sensors, CAN bus, control logic, UI),
 * creates the necessary FreeRTOS objects (queues, mutexes), sets initial user
 * settings, and starts the FreeRTOS tasks for each component.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "shared_state.h"
#include "sensors.h"
#include "can_bus_comm.h"
#include "control_logic.h"
#include "ui_task.h"

static const char *TAG = "AppMain"; // for debug

// --- Global State Definitions ---
// Define the actual global state variables (before it was inside each component)
SystemState_t g_system_state;
SemaphoreHandle_t g_system_state_mutex;
UserSettings_t g_user_settings;
SemaphoreHandle_t g_user_settings_mutex;

// extern "C" is crucial to ensure the C linker can find this function.
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "--- E-Bike Controller Application Starting ---");

    // 1. Create FreeRTOS objects (must be done before any tasks are started)
    g_system_state_mutex = xSemaphoreCreateMutex();
    g_user_settings_mutex = xSemaphoreCreateMutex();

    // Create the queues for inter-task communication
    QueueHandle_t sensor_data_queue = xQueueCreate(5, sizeof(SensorData));
    QueueHandle_t can_command_queue = xQueueCreate(5, sizeof(CAN_Command_t));
    QueueHandle_t button_event_queue = xQueueCreate(10, sizeof(ButtonEvent));

    // 2. Initialize all the components in a safe order
    ESP_LOGI(TAG, "Initializing components...");
    ui_task_init();
    bike_sensors_init(sensor_data_queue, button_event_queue);
    can_bus_init(can_command_queue);
    control_logic_init(sensor_data_queue, can_command_queue);
    ESP_LOGI(TAG, "Component initialization complete.");

    // 3. Set the initial user settings before starting the tasks.
    ESP_LOGI(TAG, "Setting initial user settings...");
    if (xSemaphoreTake(g_user_settings_mutex, portMAX_DELAY) == pdTRUE) {
        g_user_settings.current_map = MAP_TORQUE;
        g_user_settings.current_assist_level = 4;
        g_user_settings.max_motor_rpm = 5400.0f;

        xSemaphoreGive(g_user_settings_mutex);
        ESP_LOGI(TAG, "User settings updated.");
    } else {
        ESP_LOGE(TAG, "Failed to take user settings mutex during initialization!");
    }

    // 4. Start all the component tasks
    ESP_LOGI(TAG, "Starting all system tasks...");
    ui_task_start_task(button_event_queue); // Pass the queue to the UI task
    bike_sensors_start_task();
    can_bus_start_task();
    control_logic_start_task();

    ESP_LOGI(TAG, "--- System is running ---");
}

