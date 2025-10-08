/**
 * @file control_logic.cpp
 * @brief Implements the main e-bike control task.
 */

#include "control_logic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <algorithm>
#include <cmath>

#include "sensors.h"
#include "can_bus_comm.h"
#include "shared_state.h"

// --- Private Constants and State ---
static const char* TAG = "ControlLogic";
#define CONTROL_TASK_STACK_SIZE 4096
#define CONTROL_TASK_PRIORITY   10

const int PEDAL_STOP_TIMEOUT_MS = 150;
const float MAX_REGEN_AMPS = 55.0f;

const float TORQUE_MULTIPLIERS[9] = {10.0f, 15.0f, 20.0f, 25.0f, 30.0f, 45.0f, 40.0f, 45.0f, 54.0f};
const float PAS_POWER_LEVELS[5] = {14.0f, 24.0f, 34.0f, 44.0f, 54.0f};

static QueueHandle_t s_sensor_data_queue = nullptr;
static QueueHandle_t s_can_command_queue = nullptr;
static uint64_t last_pedal_timestamp = 0;


// --- Helper Functions ---

/**
 * @brief Sends a SET_CURRENT command to the CAN queue.
 * @param amps The current to send (can be positive or negative).
 */
static void send_motor_current(float amps) {
    if (s_can_command_queue == nullptr) return;
    CAN_Command_t can_command;
    can_command.command = VESC_CAN_COMMAND::SET_CURRENT;
    can_command.value = amps;
    xQueueSend(s_can_command_queue, &can_command, 0);
}

/**
 * @brief Sends a SET_BRAKE_CURRENT command to the CAN queue.
 * @param amps The braking current to apply (should be positive).
 */
static void send_brake_current(float amps) {
    if (s_can_command_queue == nullptr) return;
    CAN_Command_t can_command;
    can_command.command = VESC_CAN_COMMAND::SET_BRAKE_CURRENT;
    can_command.value = amps;
    xQueueSend(s_can_command_queue, &can_command, 0);
}


// --- Private Task Function ---

static void control_task(void* pvParameters) {
    ESP_LOGI(TAG, "Control task started.");
    
    while (1) {
        SensorData sensor_data;
        if (xQueueReceive(s_sensor_data_queue, &sensor_data, portMAX_DELAY) == pdPASS) {

            // Local snapshot of shared state
            RidingMap_t current_map = MAP_TORQUE;
            int current_assist_level = 0;
            float motor_rpm = 0.0f;
            float max_motor_rpm = 5200.0f;
            bool state_snapshot_valid = false;

            // --- Read Shared State ---
            if (xSemaphoreTake(g_system_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (xSemaphoreTake(g_user_settings_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    motor_rpm = g_system_state.motor_rpm;
                    current_map = g_user_settings.current_map;
                    current_assist_level = g_user_settings.current_assist_level;
                    max_motor_rpm = g_user_settings.max_motor_rpm;
                    
                    state_snapshot_valid = true;
                    xSemaphoreGive(g_user_settings_mutex);
                }
                xSemaphoreGive(g_system_state_mutex);
            }

            if (!state_snapshot_valid) {
                ESP_LOGW(TAG, "Failed to get state snapshot, skipping control loop.");
                continue;
            }

            // --- Control Logic (in order of priority) ---

            // 1. Digital brake cutoff (highest priority)
            if (sensor_data.brake_active) {
                float regen_amps = sensor_data.analog_brake_value * MAX_REGEN_AMPS;
                if(regen_amps > 0.1f){
                    send_brake_current(regen_amps);
                }else{send_motor_current(0.0f);}                
                continue; // Skip the rest of the loop
            }

            // 2. Analog regen brake
            if (sensor_data.analog_brake_value > 0.12f) {
                float regen_amps = sensor_data.analog_brake_value * MAX_REGEN_AMPS;
                send_brake_current(regen_amps);
                continue; // Skip the rest of the loop
            }
/*
            // 3. Backward pedaling cutoff
            if (sensor_data.direction == PedalDirection::BACKWARD) {
                send_motor_current(0.0f);
                continue; // Skip the rest of the loop
            }
*/
            // 4. Check for pedaling timeout
            if (sensor_data.direction == PedalDirection::FORWARD) {
                last_pedal_timestamp = esp_timer_get_time() / 1000;
            }
            if ((esp_timer_get_time() / 1000 - last_pedal_timestamp) > PEDAL_STOP_TIMEOUT_MS) {
                send_motor_current(0.0f);
                continue; // Skip the rest of the loop
            }

            // --- Main Riding Logic ---
            float target_amps = 0.0f;
            current_assist_level = std::max(0, std::min(current_assist_level, 8));
            switch (current_map) {
                case MAP_TORQUE:
                    current_assist_level = std::min(current_assist_level, (int)(sizeof(TORQUE_MULTIPLIERS)/sizeof(float) - 1));
                    target_amps = sensor_data.torque_reading * TORQUE_MULTIPLIERS[current_assist_level];
                    break;
                case MAP_CADENCE:
                    current_assist_level = std::min(current_assist_level, (int)(sizeof(PAS_POWER_LEVELS)/sizeof(float) - 1));
                    target_amps = PAS_POWER_LEVELS[current_assist_level];
                    break;
                case MAP_MIXED:
                    target_amps = 0.0f;
                    break;
            }

            // --- Speed Limiter ---
            const float rpm_taper_start = max_motor_rpm * 0.90f;
            if (motor_rpm > max_motor_rpm) {
                target_amps = 0.0f;
            } else if (motor_rpm > rpm_taper_start && max_motor_rpm > rpm_taper_start) {
                float scale = 1.0f - ((motor_rpm - rpm_taper_start) / (max_motor_rpm - rpm_taper_start));
                target_amps *= scale;
            }
            
            // --- Final Command ---
            // If we've gotten this far, send a motor current command.
            send_motor_current(fmax(0.0f, target_amps));
        }
    }
}

// --- Public API Implementation ---
void control_logic_init(QueueHandle_t sensor_queue, QueueHandle_t can_queue) {
    s_sensor_data_queue = sensor_queue;
    s_can_command_queue = can_queue;
}

void control_logic_start_task() {
    xTaskCreate(control_task, "control_task", CONTROL_TASK_STACK_SIZE, NULL, CONTROL_TASK_PRIORITY, NULL);
}

