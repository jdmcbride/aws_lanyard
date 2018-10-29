/* AWS ESP32 lanyard by Tekt Industries

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"

#include "lanyard_led.h"

#define GPIO_OUTPUT_IO_LED 13
#define GPIO_OUTPUT_IO_CLK 14
#define GPIO_OUTPUT_IO_DATA 16
#define GPIO_INPUT_BTN1 32
#define GPIO_INPUT_BTN2 33
#define NUM_LEDS 6

#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_LED)

#define START_DELIMITER 0x00
#define END_DELIMITER 0xFF

static esp_err_t config_gpio_for_led()
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    return gpio_config(&io_conf);
}

static esp_err_t config_gpio_for_output(int io_pin)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = 1ULL << io_pin;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    return gpio_config(&io_conf);
}

static esp_err_t config_gpio_for_input(int io_pin)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = 1ULL << io_pin;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    return gpio_config(&io_conf);
}

static void SPI_tranfer(int data_pin, int clk_pin, uint8_t val) {
    // Bit banger to drive A
    uint8_t bit;

    for (bit = 0x80; bit; bit >>= 1) {
        /* Shift-out a bit to the MOSI line */
        gpio_set_level(data_pin, val & bit);
        vTaskDelay(1 / portTICK_PERIOD_MS);

        /* clock the data */
        gpio_set_level(clk_pin, 1);
        vTaskDelay(1 / portTICK_PERIOD_MS);
        gpio_set_level(clk_pin, 0);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

static void write_delimiter(uint8_t val)
{
    SPI_tranfer(GPIO_OUTPUT_IO_DATA, GPIO_OUTPUT_IO_CLK, val);
    SPI_tranfer(GPIO_OUTPUT_IO_DATA, GPIO_OUTPUT_IO_CLK, val);
    SPI_tranfer(GPIO_OUTPUT_IO_DATA, GPIO_OUTPUT_IO_CLK, val);
    SPI_tranfer(GPIO_OUTPUT_IO_DATA, GPIO_OUTPUT_IO_CLK, val);
}

static void write_led(uint8_t red, uint8_t green, uint8_t blue, uint8_t global)
{
    SPI_tranfer(GPIO_OUTPUT_IO_DATA, GPIO_OUTPUT_IO_CLK, global);
    SPI_tranfer(GPIO_OUTPUT_IO_DATA, GPIO_OUTPUT_IO_CLK, blue);
    SPI_tranfer(GPIO_OUTPUT_IO_DATA, GPIO_OUTPUT_IO_CLK, green);
    SPI_tranfer(GPIO_OUTPUT_IO_DATA, GPIO_OUTPUT_IO_CLK, red);
}

typedef struct button_read {
    int value;
    int count;
} button_read_t;

static button_read_t button1;
static button_read_t button2;

#define DEBOUNCE_THRESHOLD 2

static int read_button(int pin, button_read_t *out_br)
{     
    int current_val = gpio_get_level(pin);
    if (current_val == out_br->value)
    {
        out_br->count++;
    } else {
        out_br->count = 0;
    }
    out_br->value = current_val;
    return (out_br->count == DEBOUNCE_THRESHOLD);
}

static uint8_t global = 0xE6;
static uint8_t glb = 0;

static uint8_t led_array[NUM_LEDS][3] = {
    { 0xFF, 0, 0 },
    { 0, 0xFF, 0 },
    { 0, 0, 0xFF },
    { 0xFF, 0, 0 },
    { 0, 0xFF, 0 },
    { 0, 0, 0xFF }
};

void lanyard_rotate_leds(int sequence_posn)
{
    write_delimiter(START_DELIMITER);

   for (int i = 0; i < NUM_LEDS; i++) {
        if (read_button(GPIO_INPUT_BTN1, &button1) && button1.value == 0) {
            glb = ((glb + 1) & 0x1F);
            global = 0xE0 + glb;
            printf("Global %d : %d\n", global, glb);
        }
        if (read_button(GPIO_INPUT_BTN2, &button2) && button2.value == 0) {
            global = 0xE0;
            printf("Global %d\n", global);
        }
        int idx = (i + sequence_posn) % NUM_LEDS;
        uint8_t red = led_array[idx][0];
        uint8_t green = led_array[idx][1];
        uint8_t blue = led_array[idx][2];
        write_led(red, green, blue, global);
    }

    write_led(255, 0, 0, global);
    write_led(0, 255, 0, global);
    write_led(0, 0, 255, global);
    write_led(255, 255, 0, global);
    write_led(0, 255, 255, global);
    write_led(255, 0, 255, global);

    write_delimiter(END_DELIMITER);
}

void lanyard_set_led(in state)
{
}

void lanyard_led_setup()
{
    config_gpio_for_led();
    config_gpio_for_output(GPIO_OUTPUT_IO_CLK);
    gpio_set_level(GPIO_OUTPUT_IO_CLK, 0);
    config_gpio_for_output(GPIO_OUTPUT_IO_DATA);
    config_gpio_for_input(GPIO_INPUT_BTN1);
    config_gpio_for_input(GPIO_INPUT_BTN2);
    gpio_set_level(GPIO_OUTPUT_IO_LED, 1); // turn on LED
}
