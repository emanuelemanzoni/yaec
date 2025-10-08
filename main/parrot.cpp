#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"

// --- Configuration ---
#define CAN_TX_PIN      GPIO_NUM_21
#define CAN_RX_PIN      GPIO_NUM_22
static const char* TAG = "CAN_PARROT";

extern "C" void app_main(void)
{
    // --- 1. Configure and Install Driver ---
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_LOGI(TAG, "Installing and starting TWAI driver...");
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install driver");
        return;
    }
    if (twai_start() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start driver");
        return;
    }
    ESP_LOGI(TAG, "Driver started. Listening for messages...");

    // --- 2. Main Loop with Enhanced Diagnostics ---
    while (1) {
        // Periodically print the status of the CAN controller.
        twai_status_info_t status_info;
        if (twai_get_status_info(&status_info) == ESP_OK) {
            ESP_LOGI(TAG, "Bus Status: State=%d, MsgsToRx=%lu, RxErrors=%lu, TxErrors=%lu",
                     status_info.state,
                     status_info.msgs_to_rx,
                     status_info.rx_error_counter,
                     status_info.tx_error_counter);
        }

        twai_message_t rx_message;
        // Check for a message with a short timeout instead of blocking forever.
        esp_err_t result = twai_receive(&rx_message, pdMS_TO_TICKS(20));

        if (result == ESP_OK) {
            ESP_LOGI(TAG, "Message received! ID: 0x%lX, DLC: %d", rx_message.identifier, rx_message.data_length_code);
            
            ESP_LOGI(TAG, "Parroting message back...");
            esp_err_t tx_result = twai_transmit(&rx_message, pdMS_TO_TICKS(100));

            if (tx_result == ESP_OK) {
                ESP_LOGI(TAG, "Parrot message sent successfully.");
            } else {
                ESP_LOGE(TAG, "Failed to send parrot message: %s", esp_err_to_name(tx_result));
            }
        } else if (result != ESP_ERR_TIMEOUT) {
            // Log any errors other than a simple timeout.
            ESP_LOGE(TAG, "Failed to receive message: %s", esp_err_to_name(result));
        }
        
        // Wait for 1 second before printing the status again.
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

