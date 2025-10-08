#include "can_bus_comm.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "freertos/task.h"

// --- Configuration Macros ---

// Task & Timing
#define CAN_TASK_STACK_SIZE     4096
#define CAN_TASK_PRIORITY       12 // Higher priority for fast CAN processing
#define CAN_TASK_DELAY_MS       10 // Loop frequency

// CAN Bus Hardware
#define CAN_TX_PIN              GPIO_NUM_21
#define CAN_RX_PIN              GPIO_NUM_22
#define VESC_CONTROLLER_ID      10 // The CAN ID of your VESC

// VESC CAN Protocol Definitions
enum CAN_PACKET_ID {
    CAN_PACKET_SET_CURRENT = 1,
    CAN_PACKET_SET_CURRENT_BRAKE = 2,
    CAN_PACKET_SET_RPM = 3,
    CAN_PACKET_STATUS_1 = 9,
    CAN_PACKET_STATUS_4 = 16,
    CAN_PACKET_STATUS_5 = 27,
};


// --- Private Variables ---
static const char* TAG = "CAN_BUS";
static QueueHandle_t g_command_queue = nullptr;
// The definitions for g_system_state and g_system_state_mutex are removed from here.
// They are now defined in main.cpp and declared as 'extern' in shared_state.h.


// --- Helper Functions ---

static void pack_float_to_int32_big_endian(twai_message_t& message, float value, float scale) {
    int32_t scaled_value = static_cast<int32_t>(value * scale);
    message.data[0] = (scaled_value >> 24) & 0xFF;
    message.data[1] = (scaled_value >> 16) & 0xFF;
    message.data[2] = (scaled_value >> 8) & 0xFF;
    message.data[3] = scaled_value & 0xFF;
    message.data_length_code = 4;
}

static void parse_status_1(const twai_message_t& message) {
    if (message.data_length_code < 4) return;
    int32_t rpm_raw = (message.data[0] << 24) | (message.data[1] << 16) | (message.data[2] << 8) | message.data[3];
    if (xSemaphoreTake(g_system_state_mutex, 0) == pdTRUE) {
        g_system_state.motor_rpm = static_cast<float>(rpm_raw);
        xSemaphoreGive(g_system_state_mutex);
    }
}

static void parse_status_4(const twai_message_t& message) {
    if (message.data_length_code < 4) return;
    int16_t temp_motor_raw = (message.data[2] << 8) | message.data[3];
    if (xSemaphoreTake(g_system_state_mutex, 0) == pdTRUE) {
        g_system_state.motor_temp = static_cast<float>(temp_motor_raw) / 10.0f;
        xSemaphoreGive(g_system_state_mutex);
    }
}

static void parse_status_5(const twai_message_t& message) {
    if (message.data_length_code < 6) return;
    int16_t voltage_raw = (message.data[4] << 8) | message.data[5];
    if (xSemaphoreTake(g_system_state_mutex, 0) == pdTRUE) {
        g_system_state.input_voltage = static_cast<float>(voltage_raw) / 10.0f;
        xSemaphoreGive(g_system_state_mutex);
    }
}


static void can_bus_task(void* pvParameters) {
    ESP_LOGI(TAG, "CAN bus task started.");
    twai_message_t rx_message;
    CAN_Command_t received_command;
    twai_message_t tx_message;
    tx_message.extd = 1;
    tx_message.rtr = 0;

    while (1) {
        if (g_command_queue != nullptr && xQueueReceive(g_command_queue, &received_command, 0) == pdPASS) {
            CAN_PACKET_ID packet_id = CAN_PACKET_SET_CURRENT;
            float scale = 1.0f;
            switch (received_command.command) {
                case VESC_CAN_COMMAND::SET_CURRENT:
                    packet_id = CAN_PACKET_SET_CURRENT;
                    scale = 1000.0f;
                    break;
                case VESC_CAN_COMMAND::SET_BRAKE_CURRENT:
                    packet_id = CAN_PACKET_SET_CURRENT_BRAKE;
                    scale = 1000.0f;
                    break;
                case VESC_CAN_COMMAND::SET_RPM:
                    packet_id = CAN_PACKET_SET_RPM;
                    scale = 1.0f;
                    break;
            }
            tx_message.identifier = (static_cast<uint32_t>(packet_id) << 8) | VESC_CONTROLLER_ID;
            pack_float_to_int32_big_endian(tx_message, received_command.value, scale);
            twai_transmit(&tx_message, 0);
        }

        twai_status_info_t status;
        twai_get_status_info(&status);
        if (status.state == TWAI_STATE_RUNNING) {
             esp_err_t result = twai_receive(&rx_message, 0);
             if (result == ESP_OK) {
                if (rx_message.extd) {
                    uint8_t controller_id = rx_message.identifier & 0xFF;
                    CAN_PACKET_ID packet_id = static_cast<CAN_PACKET_ID>((rx_message.identifier >> 8) & 0xFF);
                    if (controller_id == VESC_CONTROLLER_ID) {
                        switch (packet_id) {
                            case CAN_PACKET_STATUS_1: parse_status_1(rx_message); break;
                            case CAN_PACKET_STATUS_4: parse_status_4(rx_message); break;
                            case CAN_PACKET_STATUS_5: parse_status_5(rx_message); break;
                            default: break;
                        }
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(CAN_TASK_DELAY_MS));
    }
}


// --- Public API Implementation ---

void can_bus_init(QueueHandle_t command_queue) {
    ESP_LOGI(TAG, "Waiting for CAN transceiver to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(250)); // Delay for hardware power-on

    g_command_queue = command_queue;

    // The mutex and global state are created in main.cpp, so we don't create them here.
    // We just need to initialize the g_system_state struct.
    if (g_system_state_mutex != NULL) {
        if (xSemaphoreTake(g_system_state_mutex, portMAX_DELAY) == pdTRUE) {
            g_system_state = {}; // Zero-initialize the struct
            xSemaphoreGive(g_system_state_mutex);
        }
    } else {
         ESP_LOGE(TAG, "System state mutex has not been created before can_bus_init!");
    }


    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "CAN bus component initialized.");
}

void can_bus_start_task() {
    xTaskCreate(can_bus_task, "can_bus_task", CAN_TASK_STACK_SIZE, NULL, CAN_TASK_PRIORITY, NULL);
}

