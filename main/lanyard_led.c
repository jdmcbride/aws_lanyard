/* AWS ESP32 lanyard by Tekt Industries
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"

#include "lanyard_led.h"

#define GPIO_OUTPUT_IO_LED 13
#define GPIO_OUTPUT_IO_CLK 14
#define GPIO_OUTPUT_IO_DATA 16
#define GPIO_INPUT_BTN1 32
#define GPIO_INPUT_BTN2 33
#define NUM_LEDS 6

#define START_DELIMITER 0x00
#define END_DELIMITER 0xFF
#define PWM_GLOBAL_OFS 0xE0

#define LED_ON  1
#define LED_OFF 0

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
    // Bit banger to drive APA102-2020 RGB leds
    // Simple but inefficient as it waits - LED docs
    // say 512kHz max clock rate, we are much lower using the tick.
    uint8_t bit;

    for (bit = 0x80; bit; bit >>= 1) {
        /* Shift-out a bit to the Data line */
        gpio_set_level(data_pin, val & bit);
        vTaskDelay(1 / portTICK_PERIOD_MS);

        /* clock the data, 0->1 writes the data*/
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

#define DEBOUNCE_THRESHOLD 1

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

static uint8_t led_pwm = 0x1; // PWM set to lowest value (0-31)

static uint8_t led_array[NUM_LEDS][3] = {
    { 0xFF, 0, 0 },
    { 0, 0xFF, 0 },
    { 0, 0, 0xFF },
    { 0xFF, 0, 0 },
    { 0, 0xFF, 0 },
    { 0, 0, 0xFF }
};

static void read_buttons()
{
    int changed = 0;

    if (read_button(GPIO_INPUT_BTN1, &button1) && button1.value == 0) {
        led_pwm = (led_pwm + 1) & 0x1F;
        changed++;
    }
    if (read_button(GPIO_INPUT_BTN2, &button2) && button2.value == 0) {
        led_pwm = 0;
        changed++;
    }
    if (changed)
    {
        printf("LED PWM %d\n", led_pwm);
    }
}

void lanyard_rotate_leds(int sequence_posn)
{
    write_delimiter(START_DELIMITER);

    read_buttons();
    for (int i = 0; i < NUM_LEDS; i++)
    {
            int idx = (i + sequence_posn) % NUM_LEDS;
            uint8_t red = led_array[idx][0];
            uint8_t green = led_array[idx][1];
            uint8_t blue = led_array[idx][2];
            write_led(red, green, blue, PWM_GLOBAL_OFS + led_pwm);
        }

    write_delimiter(END_DELIMITER);
}

void lanyard_set_led(int state)
{
    gpio_set_level(GPIO_OUTPUT_IO_LED, state);
}

void lanyard_led_setup()
{
    config_gpio_for_output(GPIO_OUTPUT_IO_LED);
    config_gpio_for_output(GPIO_OUTPUT_IO_CLK);
    gpio_set_level(GPIO_OUTPUT_IO_CLK, 0); // Ensure clock line is low
    config_gpio_for_output(GPIO_OUTPUT_IO_DATA);
    config_gpio_for_input(GPIO_INPUT_BTN1);
    config_gpio_for_input(GPIO_INPUT_BTN2);
    lanyard_set_led(LED_ON);
}
