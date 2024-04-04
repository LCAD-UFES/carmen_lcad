/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"


static const char *TAG = "test";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_DRIVE_GPIO 23
#define BLINK_REVERSE_GPIO 5
#define BUTTON_GPIO 4

#define DRIVE 1
#define NEUTRAL 0
#define REVERSE -1

#define ON 0 // H-Bridge Turns "ON" when the ESP32 voltage is below VDD voltage, setting value as "0" (LOW) does the job
#define OFF 1

#define DELAY_TIME 500 // milisegundos

static int car_state = NEUTRAL;

void
configure_gpio(void)
{
    ESP_LOGI(TAG, "ESP32 GPIO CONFIG!");
    gpio_reset_pin(BLINK_DRIVE_GPIO);
    gpio_reset_pin(BLINK_REVERSE_GPIO);
    gpio_reset_pin(BUTTON_GPIO);

    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_DRIVE_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(BLINK_REVERSE_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);

    /*Enabling pulldown*/
    gpio_pullup_dis(BUTTON_GPIO); // Por padrão o pullup é habilitado, logo é necessário desabilitá-lo.
    gpio_pulldown_en(BUTTON_GPIO);
}


void car_mode_leds(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    if(car_state == DRIVE)
    {
        gpio_set_level(BLINK_REVERSE_GPIO,OFF); //Desabilitar um pino para depois ligar o outro, evitando um curto na Ponte H.
        vTaskDelay(10 / portTICK_PERIOD_MS); // 10ms delay
        gpio_set_level(BLINK_DRIVE_GPIO,ON);
    }
    
    if(car_state == REVERSE)
    {
        gpio_set_level(BLINK_DRIVE_GPIO,OFF); //Desabilitar um pino para depois ligar o outro, evitando um curto na Ponte H.
        vTaskDelay(10 / portTICK_PERIOD_MS); // 10ms delay
        gpio_set_level(BLINK_REVERSE_GPIO,ON);    
    }
}

static void 
car_mode_task(void)
{
    while (1)
    {
        ESP_LOGI(TAG, "Car is on %d mode!", car_state);
        car_mode_leds();
        vTaskDelay(DELAY_TIME / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}


static void
change_car_mode_task(void)
{
    while (1)
    {
        if(gpio_get_level(BUTTON_GPIO)) //PullDown, apenas 1 quando o botão é apertado.
        {
            if(car_state == DRIVE)car_state = REVERSE;

            else car_state = DRIVE;
        }
        vTaskDelay(DELAY_TIME / portTICK_PERIOD_MS);
    }
    
    vTaskDelete(NULL);
}

void 
app_main(void)
{
    configure_gpio();
  
    xTaskCreatePinnedToCore(car_mode_task, "car_mode_task", 1024*2, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(change_car_mode_task, "change_car_mode_task", 1024*2, NULL, 2, NULL, 0);

}


