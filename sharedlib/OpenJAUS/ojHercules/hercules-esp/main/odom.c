#include "odom.h"
#include "driver/pulse_cnt.h"
#include "esp_adc/adc_oneshot.h"
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


    if ( sensor_a == PIN_LEFT_ENCODER_A && sensor_b == PIN_LEFT_ENCODER_B )
    {
        // ESP_LOGI (TAG, "install pcnt unit");
        // pcnt_unit_config_t unit_config = {
        //     .high_limit = PCNT_HIGH_LIMIT,
        //     .low_limit = PCNT_LOW_LIMIT,
        // };
        // pcnt_unit_handle_t pcnt_unit = NULL;
        // ESP_ERROR_CHECK (pcnt_new_unit (&unit_config, &pcnt_unit));

        // ESP_LOGI (TAG, "set glitch filter");
        // pcnt_glitch_filter_config_t filter_config = {
        //     .max_glitch_ns = 1000,
        // };
        // ESP_ERROR_CHECK (pcnt_unit_set_glitch_filter (pcnt_unit, &filter_config));

        ESP_LOGI (TAG, "install pcnt channels");
        pcnt_chan_config_t chan_a_config = {
            .edge_gpio_num = sensor_a,
            .level_gpio_num = sensor_b,
        };
        pcnt_channel_handle_t pcnt_chan_a = NULL;
        
        ESP_ERROR_CHECK (
            pcnt_new_channel (pcnt_unit, &chan_a_config, &pcnt_chan_a));

        ESP_LOGI (TAG, "set edge and level actions for pcnt channels");
        ESP_ERROR_CHECK (pcnt_channel_set_edge_action (
            pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_LEVEL_ACTION_HOLD));
        ESP_ERROR_CHECK (pcnt_channel_set_level_action (
            pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_LEVEL_ACTION_HOLD));
    }
    else
    {
        // ESP_LOGI (TAG, "install pcnt unit");
        // pcnt_unit_config_t unit_config = {
        //     .high_limit = PCNT_HIGH_LIMIT,
        //     .low_limit = PCNT_LOW_LIMIT,
        // };
        // pcnt_unit_handle_t pcnt_unit = NULL;
        // ESP_ERROR_CHECK (pcnt_new_unit (&unit_config, &pcnt_unit));

        // ESP_LOGI (TAG, "set glitch filter");
        // pcnt_glitch_filter_config_t filter_config = {
        //     .max_glitch_ns = 1000,
        // };
        // ESP_ERROR_CHECK (pcnt_unit_set_glitch_filter (pcnt_unit, &filter_config));

        ESP_LOGI (TAG, "install pcnt channels");
        pcnt_chan_config_t chan_b_config = {
            .edge_gpio_num = sensor_b,
            .level_gpio_num = sensor_a,
        };
        pcnt_channel_handle_t pcnt_chan_b = NULL;
        ESP_ERROR_CHECK (
            pcnt_new_channel (pcnt_unit, &chan_b_config, &pcnt_chan_b));
        
        ESP_LOGI (TAG, "set edge and level actions for pcnt channels");
        ESP_ERROR_CHECK (pcnt_channel_set_edge_action (
            pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_LEVEL_ACTION_HOLD));
        ESP_ERROR_CHECK (pcnt_channel_set_level_action (
            pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_LEVEL_ACTION_HOLD));                              
    }


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
left_encoder_task ( void )
{
    
    int pulse_count = 0;
    float current_velocity = 0;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = CALCULATE_FREQUENCY(TASK_ENCODER_FREQUENCY); //Task frequency in FreeRTOS ticks (OBS : task_frequency * xFrequency = FreeRTOS_TickRate)
    xLastWakeTime = xTaskGetTickCount ();
    pcnt_unit_handle_t pcnt_unit = encoder_setup (PIN_LEFT_ENCODER_A,PIN_LEFT_ENCODER_B);
    double meters_per_second_per_pulse
        = (TASK_ENCODER_FREQUENCY * 2 * PI * WHEEL_DIAMETER)
          / NUMBER_OF_ENCODER_LINES ;// *GEAR_RATIO);
    while (1)
        {
            ESP_ERROR_CHECK (pcnt_unit_get_count (pcnt_unit, &pulse_count));
            ESP_LOGI (TAG, "Left Encoder Pulse Count: %d", pulse_count);
            current_velocity = pulse_count * meters_per_second_per_pulse;
            if (xSemaphoreTake (odomLeftVelocityMutex, 1000 / portTICK_PERIOD_MS)){
                odom_left_velocity = current_velocity;
                xSemaphoreGive (odomLeftVelocityMutex);
            }
            ESP_LOGD (TAG, "Left Motor Current_velocity: %.2f", current_velocity);
            ESP_ERROR_CHECK (pcnt_unit_clear_count(pcnt_unit));
            vTaskDelayUntil (&xLastWakeTime, xFrequency);
        }
    
}

void
right_encoder_task ( void )
{
    int pulse_count = 0;
    float current_velocity = 0;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = CALCULATE_FREQUENCY(TASK_ENCODER_FREQUENCY);
    xLastWakeTime = xTaskGetTickCount ();
    pcnt_unit_handle_t pcnt_unit = encoder_setup (PIN_RIGHT_ENCODER_A,PIN_RIGHT_ENCODER_B);
    double meters_per_second_per_pulse
        = (TASK_ENCODER_FREQUENCY * 2 * PI * WHEEL_DIAMETER)
          / NUMBER_OF_ENCODER_LINES ;//* GEAR_RATIO);
    while (1)
        {
            ESP_ERROR_CHECK (pcnt_unit_get_count (pcnt_unit, &pulse_count));
            ESP_LOGI (TAG, "Right Encoder Pulse Count: %d", pulse_count);
            current_velocity = pulse_count * meters_per_second_per_pulse;
            if (xSemaphoreTake (odomRightVelocityMutex, 1000 / portTICK_PERIOD_MS)){
                odom_right_velocity = current_velocity;
                xSemaphoreGive (odomRightVelocityMutex);
            }
            ESP_LOGD (TAG, "Right Motor Current velocity: %.2f", current_velocity);
            ESP_ERROR_CHECK (pcnt_unit_clear_count(pcnt_unit));
            vTaskDelayUntil (&xLastWakeTime, xFrequency);
        }
}


adc_oneshot_unit_handle_t
adc_setup(void)
{
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_POTENTIOMETER,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_POTENTIOMETER,
        .atten = ADC_ATTEN_POTENTIOMETER,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_POTENTIOMETER, &config));
    return adc_handle;
}


void
steering_reading_task ( void )
{
    int accumulator = 0;
    int adc_reading = 0;
    adc_oneshot_unit_handle_t adc_handle = adc_setup();
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = CALCULATE_FREQUENCY(TASK_STEERING_FREQUENCY);
    xLastWakeTime = xTaskGetTickCount ();
    while (1)
        {
            accumulator = 0;
            for (int i = 0; i < 64; i++)
                {
                    adc_oneshot_read(adc_handle, ADC_CHANNEL_POTENTIOMETER, &adc_reading);
                    accumulator += adc_reading;
                }
            accumulator /= 64;
            if (xSemaphoreTake (odomSteeringMutex, 1000 / portTICK_PERIOD_MS)){
                odom_steering = accumulator;
                xSemaphoreGive (odomSteeringMutex);
            }
            ESP_LOGD (TAG, "Angle measument: %d",accumulator);
            vTaskDelayUntil (&xLastWakeTime, xFrequency);
        }
}
