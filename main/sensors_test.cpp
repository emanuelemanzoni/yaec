#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "sensors.h" // Your component's header

// --- Configuration ---
static const char* TAG = "EBIKE_MAIN";
#define MAIN_TASK_STACK_SIZE 4096

// --- Global Handles ---
QueueHandle_t sensor_data_queue;
QueueHandle_t button_event_queue;

// --- Helper Functions for Printing ---

const char* direction_to_string(PedalDirection dir) {
    switch (dir) {
        case PedalDirection::FORWARD:      return "FORWARD";
        case PedalDirection::BACKWARD:     return "BACKWARD";
        case PedalDirection::NOT_PEDALING: return "NOT PEDALING";
        default:                           return "UNKNOWN";
    }
}

const char* button_id_to_string(ButtonID btn) {
    switch (btn) {
        case ButtonID::UP:     return "UP";
        case ButtonID::DOWN:   return "DOWN";
        case ButtonID::SELECT: return "SELECT";
        default:               return "UNKNOWN";
    }
}

const char* press_type_to_string(PressType type) {
    switch (type) {
        case PressType::SHORT_PRESS: return "SHORT PRESS";
        case PressType::LONG_PRESS:  return "LONG PRESS";
        default:                     return "UNKNOWN";
    }
}

/**
 * @brief Task to handle and print incoming data at a controlled rate.
 *
 * This task continuously reads data from the queues to prevent them from
 * overflowing, but only prints the latest sensor data every 500ms.
 */
void data_handler_task(void* pvParameters) {
    SensorData latest_sensor_data = {}; // Store the most recent data
    ButtonEvent received_button_event;

    const TickType_t print_interval = pdMS_TO_TICKS(500);
    TickType_t last_print_time = 0;

    ESP_LOGI(TAG, "Data handler task started. Waiting for sensor data...");

    while (1) {
        // Continuously drain the sensor queue, always keeping the latest data.
        if (xQueueReceive(sensor_data_queue, &latest_sensor_data, pdMS_TO_TICKS(20))) {
            // We successfully received data.
        }

        // Check if it's time to print the latest data we have.
        TickType_t current_time = xTaskGetTickCount();
        if ((current_time - last_print_time) >= print_interval) {
            printf("--------------------------------\n");
            printf("Sensor Data (at %lu ms):\n", current_time * portTICK_PERIOD_MS);
            printf("  - Torque:           %.2f\n", latest_sensor_data.torque_reading);
            printf("  - Cadence:          %.2f RPM\n", latest_sensor_data.cadence_rpm);
            printf("  - Direction:        %s\n", direction_to_string(latest_sensor_data.direction));
            printf("  - Digital Brake:    %s\n", latest_sensor_data.brake_active ? "ACTIVE" : "INACTIVE");
            printf("  - Analog Brake (Regen): %.2f\n", latest_sensor_data.analog_brake_value); // Display new value
            printf("--------------------------------\n\n");
            
            last_print_time = current_time; // Reset the print timer
        }

        // Always check for and immediately print button events.
        while (xQueueReceive(button_event_queue, &received_button_event, 0)) {
            printf("*********************************\n");
            printf(">>> BUTTON EVENT: %s - %s\n",
                   button_id_to_string(received_button_event.button),
                   press_type_to_string(received_button_event.press_type));
            printf("*********************************\n\n");
        }
    }
}


extern "C" void app_main(void) {
    ESP_LOGI(TAG, "E-Bike Controller Startup.");

    // 1. Create the FreeRTOS queues
    sensor_data_queue = xQueueCreate(10, sizeof(SensorData));
    button_event_queue = xQueueCreate(5, sizeof(ButtonEvent));

    if (sensor_data_queue == NULL || button_event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queues. Halting.");
        while(1);
    }
    
    // 2. Initialize the sensor component
    bike_sensors_init(sensor_data_queue, button_event_queue);

    // 3. Start the sensor reading task
    bike_sensors_start_task();

    // 4. Create a task to handle the printing of the data
    xTaskCreate(
        data_handler_task,
        "data_handler_task",
        MAIN_TASK_STACK_SIZE,
        NULL,
        5, // Lower priority than the sensor task
        NULL
    );

    ESP_LOGI(TAG, "Initialization complete. Monitoring sensor data.");
}

