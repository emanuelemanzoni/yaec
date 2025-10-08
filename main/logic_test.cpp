/**
 * @file main.cpp
 * @brief Main application entry point for the e-bike controller.
 *
 * This file initializes all the system components (sensors, CAN bus, control logic),
 * sets the initial user settings, and starts the FreeRTOS tasks for each component.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"

// Include all project components
#include "shared_state.h"
#include "sensors.h"
#include "can_bus_comm.h"
#include "control_logic.h"

static const char *TAG = "AppMain";

// --- Global State Definitions ---
// Define the actual global state variables here, in one central location.
SystemState_t g_system_state;
SemaphoreHandle_t g_system_state_mutex;
UserSettings_t g_user_settings;
SemaphoreHandle_t g_user_settings_mutex;


// The main entry point for the ESP32 application.
// extern "C" is crucial to ensure the C linker can find this function.
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "--- E-Bike Controller Application Starting ---");

    // 1. Create the mutexes that protect shared data
    g_system_state_mutex = xSemaphoreCreateMutex();
    g_user_settings_mutex = xSemaphoreCreateMutex();

    // 2. Create the queues that connect the tasks
    QueueHandle_t sensor_data_queue = xQueueCreate(5, sizeof(SensorData));
    QueueHandle_t can_command_queue = xQueueCreate(5, sizeof(CAN_Command_t));
    QueueHandle_t button_event_queue = xQueueCreate(5, sizeof(ButtonEvent));

    // Check if all creations were successful
    if (!g_system_state_mutex || !g_user_settings_mutex || !sensor_data_queue || !can_command_queue || !button_event_queue) {
        ESP_LOGE(TAG, "Failed to create one or more RTOS objects. Halting.");
        while(1); // Halt
    }

    // 3. Initialize all the components
    ESP_LOGI(TAG, "Initializing components...");
    bike_sensors_init(sensor_data_queue, button_event_queue);
    can_bus_init(can_command_queue);
    control_logic_init(sensor_data_queue, can_command_queue);
    ESP_LOGI(TAG, "Component initialization complete.");

    // 4. Set the initial user settings before starting the tasks.
    ESP_LOGI(TAG, "Setting initial user settings: Map=CADENCE(PAS), Level=1, MaxRPM=5400");
    if (xSemaphoreTake(g_user_settings_mutex, portMAX_DELAY) == pdTRUE) {
        g_user_settings.current_map = MAP_CADENCE;
        g_user_settings.current_assist_level = 1;
        g_user_settings.max_motor_rpm = 5400.0f; // Initialize the new setting
        xSemaphoreGive(g_user_settings_mutex);
        ESP_LOGI(TAG, "User settings updated.");
    } else {
        ESP_LOGE(TAG, "Failed to take user settings mutex during initialization!");
    }

    // 5. Start all the component tasks
    ESP_LOGI(TAG, "Starting all system tasks...");
    bike_sensors_start_task();
    can_bus_start_task();
    control_logic_start_task();

    ESP_LOGI(TAG, "--- System is running ---");
}

