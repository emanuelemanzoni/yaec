#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "shared_state.h"

// Defines commands sent from other tasks to this CAN task.
enum class VESC_CAN_COMMAND {
    SET_CURRENT,
    SET_BRAKE_CURRENT,
    SET_RPM
};

struct CAN_Command_t {
    VESC_CAN_COMMAND command;
    float value;
};

// Public Function Declarations for this component
void can_bus_init(QueueHandle_t command_queue);
void can_bus_start_task();

