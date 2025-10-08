#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define OLED_WIDTH  128
#define OLED_HEIGHT 64

// Pin definitions - **ADJUST FOR YOUR SETUP**
#define OLED_CLK_PIN   (gpio_num_t)19
#define OLED_MOSI_PIN  (gpio_num_t)23
#define OLED_RES_PIN   (gpio_num_t)33
#define OLED_DC_PIN    (gpio_num_t)32
#define OLED_CS_PIN    (gpio_num_t)5

typedef struct {
    spi_device_handle_t spi;
    uint8_t buffer[OLED_WIDTH * OLED_HEIGHT / 8];
} oled_t;

esp_err_t oled_init(oled_t* oled);
void oled_clear(oled_t* oled);
void oled_display(oled_t* oled);
void oled_set_pixel(oled_t* oled, int x, int y, bool on);
void oled_print_string(oled_t* oled, int x, int y, const char* str, int size);
void oled_print_number(oled_t* oled, int x, int y, int number, int size);
void oled_print_float(oled_t* oled, int x, int y, float number, int decimal_places, int size);

#ifdef __cplusplus
}
#endif

