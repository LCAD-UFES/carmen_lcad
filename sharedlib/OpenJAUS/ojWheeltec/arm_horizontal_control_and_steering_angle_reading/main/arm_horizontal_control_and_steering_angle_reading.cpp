#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/adc_common.h"
#include "esp_adc_cal.h"

#include "Arduino.h"

void step_motor_setup();
void motor_task(void *arg);


//ADC Channels
#define ADC_ANGLE_POTENTIOMETER          ADC1_CHANNEL_7

//ADC Attenuation
#define ADC_EXAMPLE_ATTEN           ADC_ATTEN_DB_11

//ADC Calibration
#if CONFIG_IDF_TARGET_ESP32
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32C3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif

static const char *TAG = "Stepper Motor and angle measurement";
static esp_adc_cal_characteristics_t adc1_chars;


static bool adc_calibration_init(void)
{
    esp_err_t ret;
    bool cali_enable = false;

    ret = esp_adc_cal_check_efuse(ADC_EXAMPLE_CALI_SCHEME);
    if (ret == ESP_ERR_NOT_SUPPORTED) 
    {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    } 
    else if (ret == ESP_ERR_INVALID_VERSION) 
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } 
    else if (ret == ESP_OK) 
    {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
    } 
    else 
    {
        ESP_LOGE(TAG, "Invalid arg");
    }

    return cali_enable;
}


static void 
adc1_task(void *arg)
{
    uint32_t voltage = 0;
    while (1) 
    {
        for (int i = 0; i < 200; i++)
        {
            uint32_t adc_reading = adc1_get_raw(ADC_ANGLE_POTENTIOMETER);
            voltage += esp_adc_cal_raw_to_voltage(adc_reading, &adc1_chars);
            vTaskDelay(0);
        }
        voltage /= 200;
        printf("Voltage: %dmV\n", voltage);
        fflush(stdout);
        vTaskDelay((1000 / 40) / portTICK_PERIOD_MS);
        voltage = 0;
    }
}


extern "C" void 
app_main(void)
{
	initArduino();

    adc_calibration_init();

    vTaskDelay(pdMS_TO_TICKS(1000));
    step_motor_setup();

    // Angle potentiometer ADC config
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_ANGLE_POTENTIOMETER, ADC_EXAMPLE_ATTEN));

    xTaskCreatePinnedToCore(adc1_task, "adc1_task", 1024*2, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(motor_task, "motor_task", 1024*2, NULL, 2, NULL, 1);

    while (1) 
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
