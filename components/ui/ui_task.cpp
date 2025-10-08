#include "ui_task.h"
#include "shared_state.h"
#include "sensors.h"
#include "oled_display.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

// --- Task & Timing Configuration ---
#define UI_TASK_STACK_SIZE 4096
#define UI_TASK_PRIORITY   5
#define SCREEN_REDRAW_INTERVAL_MS 200

// --- E-Bike Specific Constants ---
const int MOTOR_POLE_PAIRS = 46/2;
const float WHEEL_CIRCUMFERENCE_M = 1.885;
const float LOCKED_MAX_RPM = 5200.0f;
const float UNLOCKED_MAX_RPM = 7700.0f;

// --- Global variables for this file ---
static oled_t g_oled;
static bool g_speed_limit_unlocked = false;
static QueueHandle_t g_button_event_queue = NULL;

// --- Forward declarations ---
static void ui_task_main(void *pvParameters);
static void redraw_screen();
static void handle_button_event(const ButtonEvent& event);

// Extern declarations for global state defined elsewhere
extern SystemState_t g_system_state;
extern SemaphoreHandle_t g_system_state_mutex;
extern UserSettings_t g_user_settings;
extern SemaphoreHandle_t g_user_settings_mutex;

// =================================================================
// Public Function Implementations
// =================================================================

void ui_task_init(void) {
    oled_init(&g_oled);
    if (xSemaphoreTake(g_user_settings_mutex, portMAX_DELAY) == pdTRUE) {
        g_user_settings.max_motor_rpm = LOCKED_MAX_RPM;
        xSemaphoreGive(g_user_settings_mutex);
    }
}

void ui_task_start_task(QueueHandle_t button_queue) {
    g_button_event_queue = button_queue;
    xTaskCreate(ui_task_main, "UI_Task", UI_TASK_STACK_SIZE, NULL, UI_TASK_PRIORITY, NULL);
}

// =================================================================
// Private (Static) Function Implementations
// =================================================================

static void redraw_screen() {
    float speed_kmh = 0, voltage = 0;
    int temp = 0;
    char map_char = ' ';
    int assist_level = 0;

    if (xSemaphoreTake(g_system_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        speed_kmh = (g_system_state.motor_rpm / MOTOR_POLE_PAIRS) * WHEEL_CIRCUMFERENCE_M * 60.0f / 1000.0f;
        voltage = g_system_state.input_voltage;
        temp = (int)g_system_state.motor_temp;
        xSemaphoreGive(g_system_state_mutex);
    }
    
    if (xSemaphoreTake(g_user_settings_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        switch(g_user_settings.current_map) {
            case MAP_TORQUE:  map_char = 'T'; break;
            case MAP_CADENCE: map_char = 'E'; break;
            case MAP_MIXED:   map_char = 'M'; break;
        }
        assist_level = g_user_settings.current_assist_level;
        xSemaphoreGive(g_user_settings_mutex);
    }

    oled_clear(&g_oled);
    char buffer[20];

    // Center, Large Font: Speed
    // Note: The custom font renderer uses a size multiplier on an 8x8 font.
    // A size of 4 makes a character 32x32 pixels.
    snprintf(buffer, sizeof(buffer), "%.0f", speed_kmh);
    int speed_width = strlen(buffer) * 8 * 4;
    oled_print_string(&g_oled, (OLED_WIDTH - speed_width) / 2, 16, buffer, 4);

    // Left Side: Map setting
    snprintf(buffer, sizeof(buffer), g_speed_limit_unlocked ? "%c!" : "%c", map_char);
    oled_print_string(&g_oled, 2, 2, buffer, 2);

    // Assist level under map setting
    snprintf(buffer, sizeof(buffer), "%d", assist_level);
    oled_print_string(&g_oled, 10, 20, buffer, 2);

    // Top Right Corner: Voltage
    snprintf(buffer, sizeof(buffer), "%.1fV", voltage);
    int voltage_width = strlen(buffer) * 8 * 1;
    oled_print_string(&g_oled, OLED_WIDTH - voltage_width - 2, 2, buffer, 1);

    // Bottom Right Corner: Temperature
    snprintf(buffer, sizeof(buffer), "%dC", temp);
    int temp_width = strlen(buffer) * 8 * 1;
    oled_print_string(&g_oled, OLED_WIDTH - temp_width - 2, OLED_HEIGHT - 10, buffer, 1);

    oled_display(&g_oled);
}

static void handle_button_event(const ButtonEvent& event) {
    if (event.press_type == PressType::LONG_PRESS && event.button == ButtonID::UP) {
        bool brake_is_active = false;
        
        // This check assumes 'brake_active' has been added to SystemState_t
        // as previously discussed for robust inter-task communication.
        if (xSemaphoreTake(g_system_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            // brake_is_active = g_system_state.brake_active;
            xSemaphoreGive(g_system_state_mutex);
        }
        brake_is_active = true; // Placeholder for now

        if (brake_is_active) {
            g_speed_limit_unlocked = !g_speed_limit_unlocked;
            if (xSemaphoreTake(g_user_settings_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                g_user_settings.max_motor_rpm = g_speed_limit_unlocked ? UNLOCKED_MAX_RPM : LOCKED_MAX_RPM;
                xSemaphoreGive(g_user_settings_mutex);
            }
        }
        return;
    }

    if (event.press_type == PressType::SHORT_PRESS) {
        if (xSemaphoreTake(g_user_settings_mutex, portMAX_DELAY) == pdTRUE) {
            switch(event.button) {
                case ButtonID::SELECT:
                    g_user_settings.current_map = static_cast<RidingMap_t>(((int)g_user_settings.current_map + 1) % 3);
                    g_user_settings.current_assist_level = 0;
                    break;

                case ButtonID::UP:
                    if (g_user_settings.current_map == MAP_TORQUE && g_user_settings.current_assist_level < 8) {
                        g_user_settings.current_assist_level++;
                    } else if (g_user_settings.current_map == MAP_CADENCE && g_user_settings.current_assist_level < 4) {
                        g_user_settings.current_assist_level++;
                    }
                    break;

                case ButtonID::DOWN:
                    if (g_user_settings.current_assist_level > 0) {
                        g_user_settings.current_assist_level--;
                    }
                    break;
            }
            xSemaphoreGive(g_user_settings_mutex);
        }
    }
}

static void ui_task_main(void *pvParameters) {
    TickType_t last_screen_redraw = xTaskGetTickCount();
    ButtonEvent received_event;

    // Initial draw before the loop starts
    redraw_screen();

    for (;;) {
        if (g_button_event_queue != NULL && xQueueReceive(g_button_event_queue, &received_event, pdMS_TO_TICKS(10))) {
            handle_button_event(received_event);
        }

        if ((xTaskGetTickCount() - last_screen_redraw) >= pdMS_TO_TICKS(SCREEN_REDRAW_INTERVAL_MS)) {
            last_screen_redraw = xTaskGetTickCount();
            redraw_screen();
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

