#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "can_bus_comm.h" // Include our component's header

static const char* TAG = "MAIN_APP";

// Define the command sending frequency (Hz)
#define COMMAND_FREQUENCY_HZ 50
#define COMMAND_DELAY_MS (1000 / COMMAND_FREQUENCY_HZ)

// This is the main entry point for the application.
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting E-Bike Controller Transmit Test.");

    // Create the queue for sending commands to the CAN task
    QueueHandle_t can_command_queue = xQueueCreate(10, sizeof(CAN_Command_t));
    if (can_command_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create CAN command queue.");
        return;
    }

    // Initialize the CAN bus component and pass it the queue handle
    can_bus_init(can_command_queue);
    can_bus_start_task();

    // Give the can_bus_task a moment to start up
    vTaskDelay(pdMS_TO_TICKS(200));

    CAN_Command_t cmd;

    while (1) {
        // --- Phase 1: Send 2.0A for 1 second ---
        printf("--- Sending Current: 2.0A for 1 second ---\n");
        cmd.command = VESC_CAN_COMMAND::SET_CURRENT;
        cmd.value = 2.0f;
        for (int i = 0; i < 1000 / COMMAND_DELAY_MS; i++) {
            xQueueSend(can_command_queue, &cmd, 0);
            vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
        }

        // --- Phase 2: Send 0.0A for 1 second ---
        printf("--- Sending Current: 0.0A for 1 second ---\n");
        cmd.command = VESC_CAN_COMMAND::SET_CURRENT;
        cmd.value = 0.0f;
        for (int i = 0; i < 1000 / COMMAND_DELAY_MS; i++) {
            xQueueSend(can_command_queue, &cmd, 0);
            vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
        }

        // --- Phase 3: Send Brake Current @ 2.0A for 1 second ---
        printf("--- Sending Brake Current: 2.0A for 1 second ---\n");
        cmd.command = VESC_CAN_COMMAND::SET_BRAKE_CURRENT;
        cmd.value = 2.0f;
        for (int i = 0; i < 1000 / COMMAND_DELAY_MS; i++) {
            xQueueSend(can_command_queue, &cmd, 0);
            vTaskDelay(pdMS_TO_TICKS(COMMAND_DELAY_MS));
        }
        
        // --- Phase 4: Send a single 0.0A command to release the brake ---
        printf("--- Sending Current: 0.0A to release brake ---\n");
        cmd.command = VESC_CAN_COMMAND::SET_CURRENT;
        cmd.value = 0.0f;
        xQueueSend(can_command_queue, &cmd, 0);


        // --- Phase 5: Pause for 5 seconds ---
        printf("--- Pausing for 5 seconds ---\n\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

