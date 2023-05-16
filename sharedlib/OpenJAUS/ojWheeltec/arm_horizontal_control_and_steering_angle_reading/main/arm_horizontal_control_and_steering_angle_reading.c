#include "sdkconfig.h"
#include "driver/rmt.h"
#include "step_motor.h"
#include "step_motor_driver_io_a4988.h"

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

// GPIO configuration
#define STEP_MOTOR_DIRECTION_PIN GPIO_NUM_32
#define STEP_MOTOR_STEP_PIN GPIO_NUM_33
#define STEP_MOTOR_ENABLE_PIN GPIO_NUM_13
// #define STEP_MOTOR_SLEEP_PIN GPIO_NUM_16
// #define STEP_MOTOR_RESET_PIN GPIO_NUM_15
// #define STEP_MOTOR_MS3_PIN GPIO_NUM_7
// #define STEP_MOTOR_MS2_PIN GPIO_NUM_6
// #define STEP_MOTOR_MS1_PIN GPIO_NUM_5

#define RMT_TX_CHANNEL RMT_CHANNEL_0


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
#define RMT_TX_CHANNEL RMT_CHANNEL_0


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

static void 
motor_task(void *arg)
{
    // Apply default RMT configuration
    rmt_config_t dev_config = RMT_DEFAULT_CONFIG_TX(STEP_MOTOR_STEP_PIN, RMT_TX_CHANNEL);

    step_motor_io_a4988_conf_t a4988_conf = {
        .direction_pin = STEP_MOTOR_DIRECTION_PIN,
        .enable_pin = STEP_MOTOR_ENABLE_PIN,
    };

    // Install low part driver
    step_motor_driver_io_t *a4988_io;
    ESP_ERROR_CHECK(step_motor_new_a4988_io_driver(&a4988_conf, &a4988_io));

    // Install rmt driver
    step_motor_t *motor = NULL;
    ESP_ERROR_CHECK(step_motor_create_rmt(a4988_io, &dev_config, &motor));

    step_motor_init(motor);
    ESP_LOGI(TAG, "init");

    ESP_LOGI(TAG, "set_step");
    // configure Microstep to Full Step
    step_motor_set_step(motor, 1, STEP_MOTOR_DIRECTION_POSITIVE);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "step 10 @ 1000/s");
    step_motor_step(motor, 10, 1000);
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "step 100 @ 1000/s");
    step_motor_step(motor, 100, 1000);
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "step 1000 @ 1200/s");
    step_motor_step(motor, 1000, 1200);

    ESP_LOGI(TAG, "smoothstep start 5000 steps @ 500~1400/s");
    step_motor_smooth_step(motor, 5000, 1000, 500, 1400);
    ESP_LOGI(TAG, "smoothstep finish");
    vTaskDelay(pdMS_TO_TICKS(1000));

    vTaskDelay(pdMS_TO_TICKS(1000));
    step_motor_deinit(motor);
    ESP_LOGI(TAG, "deinit");
    ESP_ERROR_CHECK(step_motor_delete_rmt(motor));

    ESP_ERROR_CHECK(step_motor_delete_a4988_io_driver(a4988_io));

    // while (1) 
    // {
    //     vTaskDelay(pdMS_TO_TICKS(1000/40));
    // }
}

void app_main(void)
{
    adc_calibration_init();

    // Angle potentiometer ADC config
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_ANGLE_POTENTIOMETER, ADC_EXAMPLE_ATTEN));

    xTaskCreatePinnedToCore(adc1_task, "adc1_task", 1024*2, NULL, configMAX_PRIORITIES, NULL, 0);
    xTaskCreatePinnedToCore(motor_task, "motor_task", 1024*2, NULL, configMAX_PRIORITIES, NULL, 1);

    while (1) 
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
