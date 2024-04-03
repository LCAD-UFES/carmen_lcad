#include "can.h"
#include "driver/gpio.h"
#include "driver/twai.h"

static const char* TAG = "CAN module";

int
send_can_message (uint32_t id, int data)
{
    twai_message_t message = {.data_length_code = 2};
    message.identifier = id;
    int16_t data_send = data;
    if (id == ODOM_VELOCITY_CAN_ID)
        {
            data = (int16_t) (data * VELOCITY_CONVERSION_CONSTANT);
        }
    else if (id == ODOM_STEERING_CAN_ID)
        {
            data = (int16_t) (data * 100);
        }
    message.data[0] = (uint8_t) (data_send & 0xFF);
    message.data[1] = (uint8_t) ((data_send >> 8) & 0xFF);
    ESP_LOGD (TAG, "ID: %lu", message.identifier);
    ESP_LOGD (TAG, "Data0: %u", message.data[0]);
    ESP_LOGD (TAG, "Data1: %u", message.data[1]);
    
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
can_setup ( void )
{
    ESP_LOGI(TAG, "Setting up CAN");
    // Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT (
        PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
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
    twai_message_t message;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = CALCULATE_FREQUENCY(TASK_CAN_FREQUENCY);
    xLastWakeTime = xTaskGetTickCount ();
    int command_velocity_received;
    int command_steering_received;
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
            if (message.identifier == COMMAND_CAN_ID)
                {
                    ESP_LOGD (TAG, "Command received");
                    command_velocity_received = (message.data[1] << 8) | message.data[0];
                    command_steering_received = (message.data[3] << 8) | message.data[2];
                    if (xSemaphoreTake (commandVelocityMutex, 1000 / portTICK_PERIOD_MS)){
                        command_velocity = command_velocity_received;
                        xSemaphoreGive (commandVelocityMutex);
                    }
                    else
                    {
                        ESP_LOGE (TAG, "Failed to take command velocity mutex");
                        continue;
                    }
                    if (xSemaphoreTake (commandSteeringMutex, 1000 / portTICK_PERIOD_MS)){
                        command_steering = command_steering_received;
                        xSemaphoreGive (commandSteeringMutex);
                    }
                    else
                    {
                        ESP_LOGE (TAG, "Failed to take command steering mutex");
                        continue;
                    }
                    ESP_LOGD (TAG, "Velocity: %d", command_velocity);
                    ESP_LOGD (TAG, "Steering: %d", command_steering);
                }                    
            else
                {
                    ESP_LOGW (TAG, "Unknown message ID: %lu\n",
                              message.identifier);
               }
            vTaskDelayUntil (&xLastWakeTime, xFrequency);
        }
}

void
can_writing_task ()
{
    int current_velocity;
    int current_steering;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = CALCULATE_FREQUENCY(TASK_CAN_FREQUENCY);
    xLastWakeTime = xTaskGetTickCount ();
    while (1)
        {
            // Get current velocity
            xSemaphoreTake (odomRightVelocityMutex, portMAX_DELAY);
            xSemaphoreTake (odomLeftVelocityMutex, portMAX_DELAY);
            current_velocity = (odom_right_velocity + odom_left_velocity) / 2;
            xSemaphoreGive (odomRightVelocityMutex);
            xSemaphoreGive (odomLeftVelocityMutex);

            // Get current steering angle
            xSemaphoreTake (odomSteeringMutex, portMAX_DELAY);
            current_steering = odom_steering;
            xSemaphoreGive (odomSteeringMutex);

            // Send can messages
            send_can_message (ODOM_VELOCITY_CAN_ID,
                              current_velocity);
            send_can_message (ODOM_STEERING_CAN_ID,
                              current_steering);

            vTaskDelayUntil (&xLastWakeTime, xFrequency);
        }
}
