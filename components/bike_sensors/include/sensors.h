#pragma once // Use include guards

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// ### Data for the main control logic ###
enum class PedalDirection {
    NOT_PEDALING,
    FORWARD,
    BACKWARD
};

struct SensorData {
    float torque_reading;       // A filtered, smoothed torque value.
    float cadence_rpm;          // Cadence in revolutions per minute.
    PedalDirection direction;   // The direction of pedaling.
    bool brake_active;          // True if the digital brake (engine cutoff) is engaged.
    float analog_brake_value;   // Filtered, normalized (0.0-1.0) value for regen brake.
};

// ### Data for the user interface logic ###
enum class ButtonID {
    UP,
    DOWN,
    SELECT
};

enum class PressType {
    SHORT_PRESS,
    LONG_PRESS
};

struct ButtonEvent {
    ButtonID button;
    PressType press_type;
};

// ### Public Function Declarations ###
/**
 * @brief Initializes the sensor component and its underlying hardware.
 *
 * @param sensor_queue Handle to the queue for sending SensorData.
 * @param button_queue Handle to the queue for sending ButtonEvent.
 */
void bike_sensors_init(QueueHandle_t sensor_queue, QueueHandle_t button_queue);

/**
 * @brief Creates and starts the FreeRTOS task for continuously reading sensors.
 * This should be called after bike_sensors_init.
 */
void bike_sensors_start_task();

