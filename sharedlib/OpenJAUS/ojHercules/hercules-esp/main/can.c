#include "can.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "math.h"

static const char* TAG = "CAN module";
const TickType_t xFrequencyTaskCan = CALCULATE_FREQUENCY(TASK_CAN_FREQUENCY);

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
can_reading_task ()
{
    // variables for saving the values temporaly
    twai_message_t message;
    int16_t command_velocity_received;
    int16_t command_steering_effort_received;
    int16_t command_step_motor_received;
    
    if (!can_setup ())
    {
        ESP_LOGE (TAG, "Coudn't setup can\n");
        return;
    }

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
            continue;
        }

        // Update global command variables
        if (message.identifier == COMMAND_CAN_CAR_ID)
        {
            ESP_LOGD (TAG, "Command received for steering and velocity");
            command_velocity_received = (message.data[1] << 8) | message.data[0];
            command_steering_effort_received = (message.data[3] << 8) | message.data[2];
            set_command_velocity(command_velocity_received);
            set_command_steering_effort(command_steering_effort_received);
            ESP_LOGD (TAG, "CAN Velocity command: %hi", command_velocity);
            ESP_LOGD (TAG, "CAN Steering command: %hi", command_steering);
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

void
can_writing_task ()
{
    // variables for saving the values temporaly
    double current_velocity;
    int current_steering;
    #if TWO_MOTORS
        double current_left_velocity;
        double current_right_velocity;
    #endif

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
        current_steering = get_odom_steering();

        // Send can messages
        send_can_message (ODOM_VELOCITY_CAN_ID, current_velocity);
        send_can_message (ODOM_STEERING_CAN_ID, current_steering);

        vTaskDelayUntil (&xLastWakeTime, xFrequencyTaskCan);
    }
}
