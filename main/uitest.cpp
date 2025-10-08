#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "shared_state.h"
#include "sensors.h"
#include "ui_task.h"

// --- Define the global variables declared as 'extern' in the header files ---
SystemState_t g_system_state;
SemaphoreHandle_t g_system_state_mutex;
UserSettings_t g_user_settings;
SemaphoreHandle_t g_user_settings_mutex;
QueueHandle_t button_event_queue;

/**
 * @brief A mock task to simulate telemetry data changing over time.
 * In the real system, this data would come from a CAN bus task.
 */
void mock_telemetry_task(void *pvParameters) {
    float rpm = 0;
    for (;;) {
        if (xSemaphoreTake(g_system_state_mutex, portMAX_DELAY) == pdTRUE) {
            // Simulate motor RPM ramping up and down
            rpm = (rpm > 5000.0f) ? 0 : rpm + 150.0f;
            g_system_state.motor_rpm = rpm;
            g_system_state.input_voltage = 48.2f;
            g_system_state.motor_temp = 35.0f + (rpm / 1000.0f); // Temp rises with speed
            
            // This is important for testing the unlock feature
            g_system_state.brake_active = (rpm > 2000.0f && rpm < 2500.0f); 

            xSemaphoreGive(g_system_state_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // Update telemetry every 500ms
    }
}

/**
 * @brief A mock task to simulate a user pressing buttons.
 * In the real system, these events would come from a sensor/input task.
 */
void mock_input_task(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(2000)); // Initial delay before first button press

    for (;;) {
        ButtonEvent event;

        // 1. Cycle through assist levels in TORQUE mode
        printf("MAIN: Simulating UP press\n");
        event = {ButtonID::UP, PressType::SHORT_PRESS};
        xQueueSend(button_event_queue, &event, 0);
        vTaskDelay(pdMS_TO_TICKS(1500));

        // 2. Switch to PAS mode
        printf("MAIN: Simulating SELECT press\n");
        event = {ButtonID::SELECT, PressType::SHORT_PRESS};
        xQueueSend(button_event_queue, &event, 0);
        vTaskDelay(pdMS_TO_TICKS(1500));
        
        // 3. Cycle up in PAS mode
        printf("MAIN: Simulating UP press\n");
        event = {ButtonID::UP, PressType::SHORT_PRESS};
        xQueueSend(button_event_queue, &event, 0);
        vTaskDelay(pdMS_TO_TICKS(1500));

        // 4. Try to unlock (will only work if brake is active)
        printf("MAIN: Simulating LONG UP press (for unlock)\n");
        event = {ButtonID::UP, PressType::LONG_PRESS};
        xQueueSend(button_event_queue, &event, 0);
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait longer to see the effect
    }
}


extern "C" void app_main(void)
{
    // --- Initialization ---
    printf("MAIN: Initializing system...\n");

    // 1. Create mutexes for shared data
    g_system_state_mutex = xSemaphoreCreateMutex();
    g_user_settings_mutex = xSemaphoreCreateMutex();

    // 2. Create the queue for button events
    button_event_queue = xQueueCreate(10, sizeof(ButtonEvent));

    // 3. Check if all handles were created successfully
    if (!g_system_state_mutex || !g_user_settings_mutex || !button_event_queue) {
        printf("MAIN: Failed to create FreeRTOS objects. Halting.\n");
        while(1);
    }

    // 4. Initialize shared state with default values
    if (xSemaphoreTake(g_system_state_mutex, portMAX_DELAY) == pdTRUE) {
        g_system_state = { .motor_rpm = 0.0f, .motor_temp = 25.0f, .input_voltage = 50.4f, .brake_active = false };
        xSemaphoreGive(g_system_state_mutex);
    }
    if (xSemaphoreTake(g_user_settings_mutex, portMAX_DELAY) == pdTRUE) {
        g_user_settings = { .current_map = MAP_TORQUE, .current_assist_level = 0, .max_motor_rpm = 0.0f };
        xSemaphoreGive(g_user_settings_mutex);
    }

    // --- Start Tasks ---
    printf("MAIN: Starting tasks...\n");

    // 1. Initialize the UI hardware (OLED display)
    ui_task_init();

    // 2. Start all the tasks
    ui_task_start_task();
    xTaskCreate(mock_telemetry_task, "Mock Telemetry", 2048, NULL, 5, NULL);
    xTaskCreate(mock_input_task, "Mock Input", 2048, NULL, 5, NULL);

    printf("MAIN: Initialization complete. Tasks are running.\n");
}
