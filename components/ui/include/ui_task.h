#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sensors.h" // For ButtonEvent struct

/**
 * @brief Initializes the UI task's underlying hardware (the display).
 */
void ui_task_init(void);

/**
 * @brief Creates and starts the FreeRTOS task for the UI.
 *
 * @param button_queue Handle to the queue for receiving ButtonEvent messages.
 */
void ui_task_start_task(QueueHandle_t button_queue);


#ifdef __cplusplus
}
#endif
