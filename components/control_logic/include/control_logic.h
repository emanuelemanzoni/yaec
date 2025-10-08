/**
 * @file control_logic.h
 * @brief Public interface for the e-bike control logic component.
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "shared_state.h" // Now includes all shared data structures

/**
 * @brief Initializes the control logic component.
 *
 * Must be called before starting the task. This function sets up the necessary
 * queues for inter-task communication.
 *
 * @param sensor_data_queue Handle to the queue for receiving SensorData.
 * @param can_command_queue Handle to the queue for sending CAN_Command_t.
 */
void control_logic_init(QueueHandle_t sensor_data_queue, QueueHandle_t can_command_queue);

/**
 * @brief Creates and starts the FreeRTOS task for the main control logic.
 *
 * This should be called after control_logic_init.
 */
void control_logic_start_task();

