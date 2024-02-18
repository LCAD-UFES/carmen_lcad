#include "can.h"
#include "driver/gpio.h"
#include "driver/twai.h"

static const char* TAG = "CAN module";

int
send_can_message (twai_message_t* message, uint32_t id, int data)
{
    message->identifier = id;
    message->data[0] = data & 0xFF;
    message->data[1] = (data >> 8) & 0xFF;

    // Queue message for transmission
    if (twai_transmit (&message, pdMS_TO_TICKS (1000)) == ESP_OK)
        {
            ESP_LOGD (TAG, "Message queued for transmission\n");
            return 1;
        }
    else
        {
            ESP_LOGW (TAG, "Failed to queue message for transmission\n");
            return 0;
        }
}

int
can_setup ()
{
    // Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT (
        PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 1;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS ();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL ();

    // Install TWAI driver
    if (twai_driver_install (&g_config, &t_config, &f_config) == ESP_OK)
        {
            ESP_LOGI (TAG, "Driver installed\n");
        }
    else
        {
            ESP_LOGE (TAG, "Failed to install driver\n");
            return 0;
        }

    // Start TWAI driver
    if (twai_start () == ESP_OK)
        {
            ESP_LOGI (TAG, "Driver started\n");
        }
    else
        {
            ESP_LOGE (TAG, "Failed to start driver\n");
            return 0;
        }

    return 1;
}

void
can_reading_task (void* parameters)
{
    TaskParameters* task_parameters = (TaskParameters*)parameters;
    twai_message_t message;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = task_parameters->frequency;
    xLastWakeTime = xTaskGetTickCount ();
    while (1)
        {
            if (twai_receive (&message, pdMS_TO_TICKS (10000)) == ESP_OK)
                {
                    ESP_LOGD (TAG, "Message received\n");
                }
            else
                {
                    ESP_LOGW (TAG, "Failed to receive message\n");
                    continue;
                }

            if (message.identifier == ODOM_VELOCITY_CAN_ID)
                {
                    xSemaphoreTake (odomVelocityMutex, portMAX_DELAY);
                    odom_velocity = (message.data[1] << 8) | message.data[0];
                    xSemaphoreGive (odomVelocityMutex);
                    ESP_LOGD (TAG, "Velocity: %d\n", odom_velocity);
                }
            else if (message.identifier == ODOM_STEERING_CAN_ID)
                {
                    xSemaphoreTake (odomSteeringMutex, portMAX_DELAY);
                    odom_steering = (message.data[1] << 8) | message.data[0];
                    xSemaphoreGive (odomSteeringMutex);
                    ESP_LOGD (TAG, "Steering: %d\n", odom_steering);
                }
            else
                {
                    ESP_LOGW (TAG, "Unknown message ID: %lu\n",
                              message.identifier);
                }
            xTaskDelayUntil (&xLastWakeTime, xFrequency);
        }
}

void
can_writing_task (void* parameters)
{
    TaskParameters* task_parameters = (TaskParameters*)parameters;
    twai_message_t message;
    message.extd = 0;
    message.data_length_code = 2;
    int current_velocity;
    int current_steering;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = task_parameters->frequency;
    xLastWakeTime = xTaskGetTickCount ();
    while (1)
        {
            // Get current velocity
            xSemaphoreTake (odomVelocityMutex, portMAX_DELAY);
            current_velocity = odom_velocity;
            xSemaphoreGive (odomVelocityMutex);

            // Get current steering angle
            xSemaphoreTake (odomSteeringMutex, portMAX_DELAY);
            current_steering = odom_steering;
            xSemaphoreGive (odomSteeringMutex);

            // Send can messages
            send_can_message (&message, ODOM_VELOCITY_CAN_ID,
                              current_velocity);
            send_can_message (&message, ODOM_STEERING_CAN_ID,
                              current_steering);

            xTaskDelayUntil (&xLastWakeTime, xFrequency);
        }
}
