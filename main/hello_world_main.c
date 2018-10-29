/* Hello World Example

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


void app_main()
{
    printf("Hello world LED!\n");

    lanyard_led_setup();
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    printf("Tick period %d\n", portTICK_PERIOD_MS);

    while (1) {
        for (int i = 10; i >= 0; i--) {
            // gpio_set_level(GPIO_OUTPUT_IO_LED, i & 1);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            lanyard_rotate_leds(i);
            // gpio_set_level(GPIO_OUTPUT_IO_LED, i & 1);
        }
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
