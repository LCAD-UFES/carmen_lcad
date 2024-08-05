#include "can.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "math.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"

static const char* TAG = "CAN module";
const TickType_t xFrequencyTaskCan = CALCULATE_FREQUENCY(TASK_CAN_FREQUENCY);
const double angle_can_to_T_HIGH_coefficient = ((MIN_T_HIGH - MAX_T_HIGH) / (2*CAN_COMMAND_MAX));

int
send_can_message (uint32_t id, double data)
{
    twai_message_t message = {.data_length_code = 2};
    message.identifier = id;
    if (id == ODOM_VELOCITY_CAN_ID)
    {
        int16_t data_send = (int16_t) round(data * VELOCITY_CONVERSION_CONSTANT);
        message.data[0] = (uint8_t) (data_send & 0xFF);
        message.data[1] = (uint8_t) ((data_send >> 8) & 0xFF);
    }
    else if (id == ODOM_STEERING_CAN_ID)
    {
        uint16_t data_send = (uint16_t) round(data);
        message.data[0] = (uint8_t) (data_send & 0xFF);
        message.data[1] = (uint8_t) ((data_send >> 8) & 0xFF);
    }
    
    // Queue message for transmission
    if (twai_transmit (&message, pdMS_TO_TICKS (1000)) == ESP_OK)
    {
        ESP_LOGD (TAG, "Message queued for transmission");
        return 1;
    }
    else
    {
        ESP_LOGW (TAG, "Failed to queue message for transmission");
        return 0;
    }
}

int
can_setup ()
{
    ESP_LOGI(TAG, "Setting up CAN");
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT (PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 1;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ESP_LOGI(TAG, "Configurations set");

    // Install TWAI driver
    if (twai_driver_install (&g_config, &t_config, &f_config) == ESP_OK)
    {
        ESP_LOGI (TAG, "Driver installed");
    }
    else
    {
        ESP_LOGE (TAG, "Failed to install driver");
        return 0;
    }

    // Start TWAI driver
    if (twai_start () == ESP_OK)
    {
        ESP_LOGI (TAG, "Driver started");
    }
    else
    {
        ESP_LOGE (TAG, "Failed to start driver");
        return 0;
    }

    return 1;
}

void
config_servo_pin ()
{
    // Prepare and then apply the PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

     // Prepare and then apply the PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = LEDC_INITIAL_DUTY, // Set duty to medium angle
    };
    ledc_channel_config(&ledc_channel);

}

void
update_steering (int command_steering)
{
    if (command_steering > CAN_COMMAND_MAX)
        command_steering = CAN_COMMAND_MAX;
    else if (command_steering < -(CAN_COMMAND_MAX))
        command_steering = -(CAN_COMMAND_MAX);

    
    ESP_LOGI (TAG, "Steering command effort: %hu", command_steering);
    
    double target_T_HIGH = (command_steering * angle_can_to_T_HIGH_coefficient) + MEDIUM_T_HIGH + SERVO_BIAS;
    target_T_HIGH = target_limit_double(target_T_HIGH, MIN_T_HIGH, MAX_T_HIGH);
    int duty_cycle = ((target_T_HIGH / LEDC_PERIOD) * LEDC_MAX_DUTY);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty_cycle);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void
can_reading_task ()
{
    // variables for saving the values temporaly
    twai_message_t message;
    int16_t command_velocity_received;
    uint16_t command_steering_effort_received;
    int16_t command_step_motor_received;
    double current_command_steering = 0.0;
    
    if (!can_setup ())
    {
        ESP_LOGE (TAG, "Coudn't setup can\n");
        return;
    }
    config_servo_pin();

    // Task frequency control
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        // Receive message
        if (twai_receive (&message, pdMS_TO_TICKS (10000)) == ESP_OK)
        {
            ESP_LOGD (TAG, "Message received\n");
        }
        else
        {
            ESP_LOGW (TAG, "Failed to receive message\n");
            current_command_steering = 0;
            update_steering((int)current_command_steering); 
            continue;
        }

        // Update global command variables
        if (message.identifier == COMMAND_CAN_CAR_ID)
        {
            ESP_LOGD (TAG, "Command received for steering and velocity");

            command_velocity_received = (message.data[1] << 8) | message.data[0];
            command_steering_effort_received = (message.data[3] << 8) | message.data[2];

            set_command_velocity(command_velocity_received);

            ESP_LOGI (TAG, "CAN Steering command effort: %hi", command_steering_effort_received);
            if (command_steering_effort_received > ((CAM_LIMIT_MAX-1)/2)) {
                command_steering_effort_received -= (CAM_LIMIT_MAX);
            }
            current_command_steering += (command_steering_effort_received/10.0);
            update_steering((int)current_command_steering);           
            
            ESP_LOGD (TAG, "CAN Velocity command: %hi", command_velocity_received);
            ESP_LOGD (TAG, "CAN Steering command: %f", current_command_steering);
            set_reset_error_and_angle_counter(0);
            
        }
        else if (message.identifier == COMMAND_CAN_STEP_MOTOR_ID)
        {
            ESP_LOGD (TAG, "Command received for step motor");
            command_step_motor_received = (message.data[1] << 8) | message.data[0];
            set_command_step_motor(command_step_motor_received);
            ESP_LOGD (TAG, "CAN Step Motor command: %hi", command_step_motor);
        }                 
        else
        {
            ESP_LOGW (TAG, "Unknown message ID: %lu\n", message.identifier);
        }

        vTaskDelayUntil (&xLastWakeTime, xFrequencyTaskCan);
    }
}

adc_oneshot_unit_handle_t
adc_setup()
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

int
read_steering(adc_oneshot_unit_handle_t adc_handle)
{
    int steering = 0;
    int adc_reading = 0;
    for (int i = 0; i < 200; i++)
    {
        adc_oneshot_read(adc_handle, ADC_CHANNEL_POTENTIOMETER, &adc_reading);
        steering += adc_reading;
    }
    steering /= (200);
    steering -= POTENTIOMETER_BIAS;

    return (steering);
}

void
can_writing_task()
{
    // variables for saving the values temporaly
    double current_velocity;
    int current_steering;
    #if TWO_MOTORS
        double current_left_velocity;
        double current_right_velocity;
    #endif

    adc_oneshot_unit_handle_t adc_handle = adc_setup();

    TickType_t xLastWakeTime = xTaskGetTickCount ();
    while (1)
    {
        #if ONLY_LEFT_MOTOR
            current_velocity = get_odom_left_velocity();
        #elif ONLY_RIGHT_MOTOR
            current_velocity = get_odom_right_velocity();
        #else
            current_left_velocity = get_odom_left_velocity();
            current_right_velocity = get_odom_right_velocity();
            current_velocity = (current_left_velocity + current_right_velocity)/2;
        #endif
        current_steering = read_steering(adc_handle);

        // Send can messages
        send_can_message (ODOM_VELOCITY_CAN_ID, current_velocity);
        send_can_message (ODOM_STEERING_CAN_ID, current_steering);

        vTaskDelayUntil (&xLastWakeTime, xFrequencyTaskCan);
    }
}
