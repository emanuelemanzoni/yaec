#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "can_bus_comm.h" // Include our component's header

static const char* TAG = "MAIN_APP";

// This is the main entry point for the application.
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting E-Bike Controller Test Application.");

    // Initialize the CAN bus component.
    // We pass NULL for the queue handle because this test doesn't send commands.
    can_bus_init(NULL);
    can_bus_start_task();

    // Main loop to print telemetry data every second.
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Safely take the mutex before accessing the shared data
        if (xSemaphoreTake(g_system_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            
            printf("--- VESC Telemetry ---\n");
            printf("ERPM: %.2f\n", g_system_state.motor_rpm);
            printf("Motor Temp: %.2f C\n", g_system_state.motor_temp);
            printf("Voltage: %.2f V\n", g_system_state.input_voltage);
            printf("----------------------\n\n");

            // Give the mutex back
            xSemaphoreGive(g_system_state_mutex);
        } else {
            ESP_LOGE(TAG, "Failed to get system state mutex to print data.");
        }
    }
}

