#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ### System State (Live Data) ###
// This struct holds all live data shared across the system.
struct SystemState_t {
    float motor_rpm;        // From CAN_PACKET_STATUS_1
    float motor_temp;       // From CAN_PACKET_STATUS_4
    float input_voltage;    // From CAN_PACKET_STATUS_5
    bool brake_active;      // To be updated by the task reading sensors
};

extern SystemState_t g_system_state;
extern SemaphoreHandle_t g_system_state_mutex;


// ### User Settings (Configurable Data) ###
// Defines the different assistance calculation methods.
enum RidingMap_t {
    MAP_TORQUE,
    MAP_CADENCE,
    MAP_MIXED
};

// This struct holds settings that are changed by the user.
struct UserSettings_t {
    RidingMap_t current_map;
    int current_assist_level;
    float max_motor_rpm; // User-configurable speed limit
};

extern UserSettings_t g_user_settings;
extern SemaphoreHandle_t g_user_settings_mutex;

