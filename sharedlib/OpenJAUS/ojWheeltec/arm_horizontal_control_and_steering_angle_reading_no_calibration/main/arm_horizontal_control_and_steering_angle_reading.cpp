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
#include "driver/mcpwm.h"


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

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH    1100 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH    1900 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE        180 //Maximum angle in degree upto which servo can rotate
#define SERVO_MIN_DEGREE        0 //Minimum angle in degree upto which servo can rotate
#define SERVO_PULSE_GPIO        (18)   // GPIO connects to the PWM signal line

static esp_adc_cal_characteristics_t adc1_chars;

static const char *TAG = "Stepper Motor, angle measurement and steering servo control";

double current_steering_angle = (double) (SERVO_MIN_DEGREE + (SERVO_MAX_DEGREE + SERVO_MIN_DEGREE) / 2.0);


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
        printf("Voltage: %d mV\n", voltage);
        fflush(stdout);
        vTaskDelay((1000 / 40) / portTICK_PERIOD_MS);
        voltage = 0;
    }
}

/**
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
static uint32_t 
servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

void 
init_steering_servo_control(void)
{
    // 1. mcpwm gpio initialization
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PULSE_GPIO);  // Set GPIO 18 as PWM0A, to which servo is connected

    // 2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 100; // frequency = 100Hz, i.e. for this servo motor, time period should be 10ms
    pwm_config.cmpr_a = 0;      // duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;      // duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    // Configure PWM0A & PWM0B with above settings
}

void
set_steering_servo_angle(uint32_t angle)
{
    uint32_t angle_in_us = servo_per_degree_init(angle);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle_in_us);
}

void
set_steering_effort(int steering_effort)
{
    current_steering_angle += (double) steering_effort / 10.0;
    if (current_steering_angle > SERVO_MAX_DEGREE)
        current_steering_angle = SERVO_MAX_DEGREE;
    else if (current_steering_angle < SERVO_MIN_DEGREE)
        current_steering_angle = SERVO_MIN_DEGREE;
    
    int new_angle = round(current_steering_angle);
    set_steering_servo_angle(new_angle);
}

extern "C" void 
app_main(void)
{
	initArduino();

    adc_calibration_init();

    vTaskDelay(pdMS_TO_TICKS(1000));
    step_motor_setup();

    init_steering_servo_control();

    // Angle potentiometer ADC config
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_ANGLE_POTENTIOMETER, ADC_EXAMPLE_ATTEN));

    xTaskCreatePinnedToCore(adc1_task, "adc1_task", 1024*2, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(motor_task, "motor_task", 1024*2, NULL, 2, NULL, 1);

    while (1) 
    {
        // Teste do servo
        // for (uint32_t count = SERVO_MIN_DEGREE; count < SERVO_MAX_DEGREE; count++) 
        // {
        //     // printf("Angle of rotation: %d\n", count);
        //     uint32_t angle = servo_per_degree_init(count);
        //     // printf("pulse width: %dus\n", angle);
        //     mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
        //     vTaskDelay(pdMS_TO_TICKS(50));     // Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
        //     fflush(stdout);
        // }

        // for (uint32_t count = SERVO_MAX_DEGREE; count > SERVO_MIN_DEGREE; count--) 
        // {
        //     // printf("Angle of rotation: %d\n", count);
        //     uint32_t angle = servo_per_degree_init(count);
        //     // printf("pulse width: %dus\n", angle);
        //     mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
        //     vTaskDelay(pdMS_TO_TICKS(50));     // Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
        //     fflush(stdout);
        // }

       vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
