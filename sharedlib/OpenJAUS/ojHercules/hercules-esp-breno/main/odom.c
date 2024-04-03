#include "odom.h"
#include "driver/pulse_cnt.h"
// #include "driver/adc.h"
// #include "driver/adc_common.h"


static const char* TAG = "ODOM module";

bool
pcnt_on_reach (pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* edata,
               void* user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callback
    xQueueSendFromISR (queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}

pcnt_unit_handle_t
encoder_setup (int sensor_a, int sensor_b)
{
    ESP_LOGI (TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK (pcnt_new_unit (&unit_config, &pcnt_unit));

    ESP_LOGI (TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK (pcnt_unit_set_glitch_filter (pcnt_unit, &filter_config));

    ESP_LOGI (TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = sensor_a,
        .level_gpio_num = sensor_b,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    
    ESP_ERROR_CHECK (
        pcnt_new_channel (pcnt_unit, &chan_a_config, &pcnt_chan_a));
    /*
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = PIN_ENCODER_B,
        .level_gpio_num = PIN_ENCODER_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK (
        pcnt_new_channel (pcnt_unit, &chan_b_config, &pcnt_chan_b));
    */

    ESP_LOGI (TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK (pcnt_channel_set_edge_action (
        pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_LEVEL_ACTION_HOLD));
    ESP_ERROR_CHECK (pcnt_channel_set_level_action (
        pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_LEVEL_ACTION_HOLD));
    /*
    ESP_ERROR_CHECK (pcnt_channel_set_edge_action (
        pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK (pcnt_channel_set_level_action (
        pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    */

    //QueueHandle_t queue = xQueueCreate (10, sizeof (int));
    ESP_LOGI (TAG, "enable pcnt unit");
    ESP_ERROR_CHECK (pcnt_unit_enable (pcnt_unit));
    ESP_LOGI (TAG, "clear pcnt unit");
    ESP_ERROR_CHECK (pcnt_unit_clear_count (pcnt_unit));
    ESP_LOGI (TAG, "start pcnt unit");
    ESP_ERROR_CHECK (pcnt_unit_start (pcnt_unit));

    return pcnt_unit;
}

void
right_encoder_task ( void )
{
    
    int pulse_count = 0;
    float current_velocity = 0;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = CALCULATE_FREQUENCY(TASK_ENCODER_FREQUENCY); //Task frequency in FreeRTOS ticks (OBS : task_frequency * xFrequency = FreeRTOS_TickRate)
    xLastWakeTime = xTaskGetTickCount ();
    pcnt_unit_handle_t pcnt_unit = encoder_setup (PIN_RIGHT_ENCODER_A,PIN_RIGHT_ENCODER_B);
    double meters_per_second_per_pulse
        = (TASK_ENCODER_FREQUENCY * 2 * PI * WHEEL_DIAMETER)
          / NUMBER_OF_ENCODER_LINES ;// *GEAR_RATIO);
    while (1)
        {
            ESP_ERROR_CHECK (pcnt_unit_get_count (pcnt_unit, &pulse_count));
            //ESP_LOGI (TAG, "Right Encoder Pulse Count: %d", pulse_count);
            current_velocity = pulse_count * meters_per_second_per_pulse;
            if (xSemaphoreTake (odomRightVelocityMutex, 1000 / portTICK_PERIOD_MS)){
                odom_right_velocity = current_velocity;
                xSemaphoreGive (odomRightVelocityMutex);
            }
            ESP_LOGI (TAG, "Right Motor Current_velocity: %.2f", current_velocity);
            ESP_ERROR_CHECK (pcnt_unit_clear_count(pcnt_unit));
            vTaskDelayUntil (&xLastWakeTime, xFrequency);
        }
    
}

void
left_encoder_task ( void )
{
    int pulse_count = 0;
    float current_velocity = 0;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = CALCULATE_FREQUENCY(TASK_ENCODER_FREQUENCY);
    xLastWakeTime = xTaskGetTickCount ();
    pcnt_unit_handle_t pcnt_unit = encoder_setup (PIN_LEFT_ENCODER_A,PIN_LEFT_ENCODER_B);
    double meters_per_second_per_pulse
        = (TASK_ENCODER_FREQUENCY * 2 * PI * WHEEL_DIAMETER)
          / NUMBER_OF_ENCODER_LINES ;//* GEAR_RATIO);
    while (1)
        {
            ESP_ERROR_CHECK (pcnt_unit_get_count (pcnt_unit, &pulse_count));
            //ESP_LOGI (TAG, "Left Encoder Pulse Count: %d", pulse_count);
            current_velocity = pulse_count * meters_per_second_per_pulse;
            if (xSemaphoreTake (odomLeftVelocityMutex, 1000 / portTICK_PERIOD_MS)){
                odom_left_velocity = current_velocity;
                xSemaphoreGive (odomLeftVelocityMutex);
            }
            ESP_LOGI (TAG, "Left Motor Current velocity: %.2f", current_velocity);
            ESP_ERROR_CHECK (pcnt_unit_clear_count(pcnt_unit));
            vTaskDelayUntil (&xLastWakeTime, xFrequency);
        }
}


// bool
// adc_calibration_init(void)
// {
//     esp_err_t ret;
//     bool cali_enable = false;

//     ret = esp_adc_cal_check_efuse(ADC_EXAMPLE_CALI_SCHEME);
//     if (ret == ESP_ERR_NOT_SUPPORTED) 
//     {
//         ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
//     } 
//     else if (ret == ESP_ERR_INVALID_VERSION) 
//     {
//         ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
//     } 
//     else if (ret == ESP_OK) 
//     {
//         cali_enable = true;
//         esp_adc_cal_characterize(ADC_UNIT_1, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
//     } 
//     else 
//     {
//         ESP_LOGE(TAG, "Invalid arg");
//     }

//     return cali_enable;
// }


void
steering_reading_task ( void )
{
    uint32_t voltage = 0;
    // adc_calibration_init();
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = CALCULATE_FREQUENCY(TASK_STEERING_FREQUENCY);
    xLastWakeTime = xTaskGetTickCount ();
    while (1)
        {
            // for (int i = 0; i < 200; i++)
            //     {
            //         uint32_t adc_reading
            //             = adc1_get_raw (PIN_SERVO_POTENTIOMETER);
            //         voltage += esp_adc_cal_raw_to_voltage (adc_reading,
            //                                                &adc1_chars);
            //         vTaskDelay (0);
            //     }
            // voltage /= 200;
            // // printf ("Voltage: %d mV\n", voltage);
            // if (xSemaphoreTake (odomSteeringMutex, 1000 / portTICK_PERIOD_MS)){
            //     odom_steering = voltage;
            //     xSemaphoreGive (odomSteeringMutex);
            // }
            // voltage = 0;
            vTaskDelayUntil (&xLastWakeTime, xFrequency);
        }
}
