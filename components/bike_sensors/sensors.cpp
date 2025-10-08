#include "sensors.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <numeric>
#include <array>
#include <algorithm> // For std::clamp
#include "shared_state.h"

// --- Configuration Macros ---
// Hardware Pins
#define TORQUE_SENSOR_PIN         ADC_CHANNEL_6 // Mapped to GPIO34
#define ANALOG_BRAKE_SENSOR_PIN   ADC_CHANNEL_7 // Mapped to GPIO35
#define CADENCE_A_PIN             GPIO_NUM_13
#define CADENCE_B_PIN             GPIO_NUM_14
#define BRAKE_SENSOR_PIN          GPIO_NUM_16 
#define BUTTON_UP_PIN             GPIO_NUM_15
#define BUTTON_DOWN_PIN           GPIO_NUM_17
#define BUTTON_SELECT_PIN         GPIO_NUM_18 

// Task & Timing
#define SENSOR_TASK_STACK_SIZE 4096
#define SENSOR_TASK_PRIORITY   10
#define SENSOR_TASK_DELAY_MS   10

// Sensor Processing
#define TORQUE_FILTER_WINDOW_SIZE 5
#define ANALOG_BRAKE_FILTER_WINDOW_SIZE 5 
#define DEBOUNCE_STABLE_READINGS  3
#define LONG_PRESS_CYCLES         (1000 / SENSOR_TASK_DELAY_MS) 
#define PULSES_PER_REVOLUTION     32.0f
#define CADENCE_STOPPED_CYCLES    (500 / SENSOR_TASK_DELAY_MS) // Increased to 500ms timeout

// PCNT Specific limits
#define PCNT_HIGH_LIMIT           1000 
#define PCNT_LOW_LIMIT            -1000

// ADC Calibration (EXAMPLE VALUES - MUST BE CALIBRATED FOR YOUR SENSORS)
#define ANALOG_BRAKE_ADC_MIN 810 
#define ANALOG_BRAKE_ADC_MAX 2539
#define TORQUE_ADC_ZERO 750      // Raw ADC value when there is NO force on the pedals
#define TORQUE_ADC_MAX  3300     // Raw ADC value under a defined "maximum" force

// --- Private Variables ---
static const char* TAG = "BIKE_SENSORS";

// Queues for sending data out
static QueueHandle_t g_sensor_queue = nullptr;
static QueueHandle_t g_button_queue = nullptr;

// Handles for new drivers
static adc_oneshot_unit_handle_t g_adc_handle;
static pcnt_unit_handle_t g_pcnt_unit = nullptr;

// Sensor filtering buffers
static std::array<int, TORQUE_FILTER_WINDOW_SIZE> torque_readings;
static int torque_reading_index = 0;
static std::array<int, ANALOG_BRAKE_FILTER_WINDOW_SIZE> analog_brake_readings;
static int analog_brake_reading_index = 0;

// Cadence timing and state-holding
static int cadence_cycles_since_last_pulse = 0;
static float g_current_cadence_rpm = 0.0f;
static PedalDirection g_current_pedal_direction = PedalDirection::NOT_PEDALING;

// Digital brake sensor debouncing
static int brake_debounce_counter = 0;
static bool brake_state = false;

// Button state machines
struct ButtonState {
    int press_counter = 0;
    bool pressed = false;
    bool long_press_sent = false;
};
static ButtonState button_up_state, button_down_state, button_select_state;


// --- Helper Functions ---

static float process_torque_sensor() {
    int adc_raw_reading = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(g_adc_handle, TORQUE_SENSOR_PIN, &adc_raw_reading));

    torque_readings[torque_reading_index] = adc_raw_reading;
    torque_reading_index = (torque_reading_index + 1) % TORQUE_FILTER_WINDOW_SIZE;

    float sum = std::accumulate(torque_readings.begin(), torque_readings.end(), 0);
    float avg_adc = sum / TORQUE_FILTER_WINDOW_SIZE;
    
    float normalized_value = (avg_adc - TORQUE_ADC_ZERO) / (TORQUE_ADC_MAX - TORQUE_ADC_ZERO);

    return std::clamp(normalized_value, 0.0f, 1.0f);
}

static float process_analog_brake_sensor() {
    int adc_raw_reading = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(g_adc_handle, ANALOG_BRAKE_SENSOR_PIN, &adc_raw_reading));

    analog_brake_readings[analog_brake_reading_index] = adc_raw_reading;
    analog_brake_reading_index = (analog_brake_reading_index + 1) % ANALOG_BRAKE_FILTER_WINDOW_SIZE;
    
    float sum = std::accumulate(analog_brake_readings.begin(), analog_brake_readings.end(), 0);
    float avg_adc = sum / ANALOG_BRAKE_FILTER_WINDOW_SIZE;

    float normalized_value = (avg_adc - ANALOG_BRAKE_ADC_MIN) / (ANALOG_BRAKE_ADC_MAX - ANALOG_BRAKE_ADC_MIN);
    
    return std::clamp(normalized_value, 0.0f, 1.0f);
}


static bool process_digital_brake_sensor() {
    bool raw_state = !gpio_get_level(BRAKE_SENSOR_PIN); 

    if (raw_state == brake_state) {
        brake_debounce_counter = 0;
    } else {
        brake_debounce_counter++;
        if (brake_debounce_counter >= DEBOUNCE_STABLE_READINGS) {
            brake_state = raw_state;
            brake_debounce_counter = 0;
        }
    }
    return brake_state;
}

static void process_button(ButtonState& state, ButtonID id, gpio_num_t pin) {
    bool is_pressed_now = !gpio_get_level(pin); 

    if (is_pressed_now) {
        state.press_counter++;
        if (!state.pressed) {
            state.pressed = true;
        }
        if (state.press_counter >= LONG_PRESS_CYCLES && !state.long_press_sent) {
            ButtonEvent event = {id, PressType::LONG_PRESS};
            xQueueSend(g_button_queue, &event, 0);
            state.long_press_sent = true;
        }
    } else {
        if (state.pressed) {
            if (!state.long_press_sent) {
                ButtonEvent event = {id, PressType::SHORT_PRESS};
                xQueueSend(g_button_queue, &event, 0);
            }
            state.pressed = false;
            state.long_press_sent = false;
            state.press_counter = 0;
        }
    }
}

// ====================================================================
// ============ REWRITTEN CADENCE LOGIC (STATE-HOLDING) ===============
// ====================================================================
static void process_cadence(float& rpm, PedalDirection& dir) {
    int pulse_count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(g_pcnt_unit, &pulse_count));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(g_pcnt_unit));

    cadence_cycles_since_last_pulse++; // Always increment the timer

    if (pulse_count != 0) {
        // A pulse was detected. Calculate new RPM based on time since last pulse.
        float time_elapsed_ms = (float)cadence_cycles_since_last_pulse * SENSOR_TASK_DELAY_MS;
        float time_elapsed_min = time_elapsed_ms / 60000.0f;
        float revolutions = (float)std::abs(pulse_count) / PULSES_PER_REVOLUTION;
        
        g_current_cadence_rpm = revolutions / time_elapsed_min;
        g_current_pedal_direction = (pulse_count > 0) ? PedalDirection::FORWARD : PedalDirection::BACKWARD;

        // Reset the timer for the next interval
        cadence_cycles_since_last_pulse = 0;
    } else {
        // No pulse detected in this cycle. Check if timeout has been reached.
        if (cadence_cycles_since_last_pulse > CADENCE_STOPPED_CYCLES) {
            // It's been too long, we are officially stopped.
            g_current_cadence_rpm = 0.0f;
            g_current_pedal_direction = PedalDirection::NOT_PEDALING;
        }
        // If timeout is NOT reached, we do nothing. The global state variables hold their previous values.
    }

    // Always output the current state (which holds the last calculated value until timeout)
    rpm = g_current_cadence_rpm;
    dir = g_current_pedal_direction;
}
// ====================================================================

static void bike_sensors_task(void* pvParameters) {
    ESP_LOGI(TAG, "Sensor task started.");
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        SensorData sensor_data;
        
        sensor_data.torque_reading = process_torque_sensor();
        sensor_data.analog_brake_value = process_analog_brake_sensor(); 
        sensor_data.brake_active = process_digital_brake_sensor();
        
        // --- ADDED: Update the global shared state with the brake status ---
        // We use a non-blocking take on the mutex. If it's busy (e.g., UI is reading it),
        // we simply skip the update for this 10ms cycle. This prevents our high-priority
        // sensor task from ever being delayed by a lower-priority task.
        if (xSemaphoreTake(g_system_state_mutex, 0) == pdTRUE) {
            g_system_state.brake_active = sensor_data.brake_active;
            xSemaphoreGive(g_system_state_mutex);
        }
        // --------------------------------------------------------------------

        process_cadence(sensor_data.cadence_rpm, sensor_data.direction);

        process_button(button_up_state, ButtonID::UP, BUTTON_UP_PIN);
        process_button(button_down_state, ButtonID::DOWN, BUTTON_DOWN_PIN);
        process_button(button_select_state, ButtonID::SELECT, BUTTON_SELECT_PIN);
        
        if (g_sensor_queue != nullptr) {
            xQueueSend(g_sensor_queue, &sensor_data, 0);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_TASK_DELAY_MS));
    }
}

// --- Public API Implementation ---

void bike_sensors_init(QueueHandle_t sensor_queue, QueueHandle_t button_queue) {
    g_sensor_queue = sensor_queue;
    g_button_queue = button_queue;

    // --- GPIO Configuration ---
    gpio_config_t input_conf = {};
    input_conf.intr_type = GPIO_INTR_DISABLE;
    input_conf.mode = GPIO_MODE_INPUT;
    input_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    input_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    input_conf.pin_bit_mask = (1ULL << BRAKE_SENSOR_PIN) |
                              (1ULL << BUTTON_UP_PIN)  |
                              (1ULL << BUTTON_DOWN_PIN)|
                              (1ULL << BUTTON_SELECT_PIN);
    ESP_ERROR_CHECK(gpio_config(&input_conf));

    // --- ADC Configuration ---
    adc_oneshot_unit_init_cfg_t adc_init_cfg = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT, 
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_cfg, &g_adc_handle));
    
    adc_oneshot_chan_cfg_t adc_chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc_handle, TORQUE_SENSOR_PIN, &adc_chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc_handle, ANALOG_BRAKE_SENSOR_PIN, &adc_chan_cfg));
    
    int initial_adc_val = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(g_adc_handle, TORQUE_SENSOR_PIN, &initial_adc_val));
    torque_readings.fill(initial_adc_val);

    ESP_ERROR_CHECK(adc_oneshot_read(g_adc_handle, ANALOG_BRAKE_SENSOR_PIN, &initial_adc_val));
    analog_brake_readings.fill(initial_adc_val);

    // --- PCNT Configuration ---
    ESP_LOGI(TAG, "Configuring PCNT for quadrature encoder.");
    pcnt_unit_config_t unit_config = {};
    unit_config.high_limit = PCNT_HIGH_LIMIT;
    unit_config.low_limit = PCNT_LOW_LIMIT;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &g_pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = { .max_glitch_ns = 1000 };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(g_pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_a_config = {};
    chan_a_config.edge_gpio_num = CADENCE_A_PIN;
    chan_a_config.level_gpio_num = CADENCE_B_PIN;
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(g_pcnt_unit, &chan_a_config, &pcnt_chan_a));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    ESP_ERROR_CHECK(pcnt_unit_enable(g_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(g_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(g_pcnt_unit));

    ESP_LOGI(TAG, "Sensor component initialized.");
}

void bike_sensors_start_task() {
    xTaskCreate(
        bike_sensors_task,
        "bike_sensors_task",
        SENSOR_TASK_STACK_SIZE,
        NULL,
        SENSOR_TASK_PRIORITY,
        NULL
    );
}


