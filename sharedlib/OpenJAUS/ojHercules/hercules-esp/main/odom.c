#include "odom.h"
#include "driver/pulse_cnt.h"
#include "esp_adc/adc_oneshot.h"


static const char* TAG = "ODOM module";
const TickType_t xFrequencyTaskEncoder = CALCULATE_FREQUENCY(TASK_ENCODER_FREQUENCY);
const TickType_t xFrequencyTaskSteering = CALCULATE_FREQUENCY(TASK_STEERING_FREQUENCY);
const double meters_per_second_per_pulse = (TASK_ENCODER_FREQUENCY * PI * WHEEL_DIAMETER) / (PULSES_PER_REVOLUTION);

bool
pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* edata, void* user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callback
    xQueueSendFromISR (queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}

pcnt_unit_handle_t
encoder_setup(int sensor_a, int sensor_b)
{
    ESP_LOGI (TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK (pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI (TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK (pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    ESP_LOGI (TAG, "install pcnt channels");
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = sensor_a,
        .level_gpio_num = sensor_b,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    
    ESP_ERROR_CHECK (pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    ESP_LOGI (TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK (pcnt_channel_set_edge_action (pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_LEVEL_ACTION_HOLD));
    ESP_ERROR_CHECK (pcnt_channel_set_level_action (pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_LEVEL_ACTION_HOLD));

    //QueueHandle_t queue = xQueueCreate (10, sizeof (int));
    ESP_LOGI (TAG, "enable pcnt unit");
    ESP_ERROR_CHECK (pcnt_unit_enable (pcnt_unit));
    ESP_LOGI (TAG, "clear pcnt unit");
    ESP_ERROR_CHECK (pcnt_unit_clear_count (pcnt_unit));
    ESP_LOGI (TAG, "start pcnt unit");
    ESP_ERROR_CHECK (pcnt_unit_start (pcnt_unit));

    return pcnt_unit;
}

int
encoder_pulse_counter(pcnt_unit_handle_t *pcnt_unit)
{
    int pulse_count;
    ESP_ERROR_CHECK (pcnt_unit_get_count (*pcnt_unit, &pulse_count));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(*pcnt_unit));
    return pulse_count;
}

double
read_encoder(pcnt_unit_handle_t *pcnt_unit)
{
    int pulse_count = 0;
    pulse_count = encoder_pulse_counter(pcnt_unit);
    ESP_LOGD (TAG, "Encoder Pulse Count: %d", pulse_count);
    return (pulse_count * meters_per_second_per_pulse);
}

void
left_encoder_task()
{
    int pulse_count = 0;
    double current_velocity = 0;

    pcnt_unit_handle_t pcnt_unit = encoder_setup(PIN_LEFT_ENCODER_A,PIN_LEFT_ENCODER_B);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        current_velocity = read_encoder(&pcnt_unit);
        set_odom_left_velocity(current_velocity);
        vTaskDelayUntil (&xLastWakeTime, xFrequencyTaskEncoder);
    }
}

void
right_encoder_task()
{
    int pulse_count = 0;
    double current_velocity = 0;

    pcnt_unit_handle_t pcnt_unit = encoder_setup(PIN_RIGHT_ENCODER_A,PIN_RIGHT_ENCODER_B);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        current_velocity = read_encoder(&pcnt_unit);
        set_odom_right_velocity(current_velocity);
        vTaskDelayUntil (&xLastWakeTime, xFrequencyTaskEncoder);
    }
}
