#include "test.h"
#include "control.h"
#include "driver/pulse_cnt.h"

static const char* TAG = "Test module";


/*Task that prints 'Hello World' repeatedly for testing the ESP32.*/
void
test_task (void* parameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount ();
    const TickType_t xFrequency = CALCULATE_FREQUENCY(TASK_TEST_FREQUENCY);
    while (1)
        {
            ESP_LOGI (TAG, "Hello World");
            vTaskDelayUntil (&xLastWakeTime, xFrequency);
        }
}


/* Task that writes fake velocity and steering values to the odom_velocity
 * and odom_steering global variables.*/
void
fake_odometry_task ()
{
    int current_velocity = 0;
    int current_steering = 0;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = CALCULATE_FREQUENCY(TASK_TEST_FREQUENCY);
    xLastWakeTime = xTaskGetTickCount ();
    while (1)
        {
            current_velocity += 1;
            current_steering += 1;
            current_velocity = current_velocity % 100;
            current_steering = current_steering % 100;
            if (xSemaphoreTake (odomRightVelocityMutex, 1000 / portTICK_PERIOD_MS))
                {
                    odom_right_velocity = current_velocity;
                    xSemaphoreGive (odomRightVelocityMutex);
                }
            else
                {
                    ESP_LOGE (TAG, "Failed to take odom right velocity mutex");
                    continue;
                }
            if (xSemaphoreTake (odomLeftVelocityMutex, 1000 / portTICK_PERIOD_MS))
                {
                    odom_left_velocity = current_velocity;
                    xSemaphoreGive (odomLeftVelocityMutex);
                }
            else
                {
                    ESP_LOGE (TAG, "Failed to take odom left velocity mutex");
                    continue;
                }
            if (xSemaphoreTake (odomSteeringMutex, 1000 / portTICK_PERIOD_MS))
                {
                    odom_steering = current_steering;
                    xSemaphoreGive (odomSteeringMutex);
                }
            else
                {
                    ESP_LOGE (TAG, "Failed to take odom steering mutex");
                    continue;
                }

            ESP_LOGI (TAG, "Velocity created: %d", current_velocity);
            ESP_LOGI (TAG, "Steering created: %d", current_steering);
            vTaskDelayUntil (&xLastWakeTime, xFrequency);
        }
}


/*Task that writes fake velocity and steering values to the
 * command_velocity and command_steering global variables.*/
void
fake_commands_task ()
{

    int current_velocity = 0;
    int current_steering = 0;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10;
    xLastWakeTime = xTaskGetTickCount ();
    while (1)
        {
            current_velocity += CAN_COMMAND_MAX / 100;
            // current_steering += CAN_COMMAND_MAX / 100;
            current_velocity = current_velocity % CAN_COMMAND_MAX;
            current_steering = current_steering % CAN_COMMAND_MAX;
            if (xSemaphoreTake (commandVelocityMutex,
                                1000 / portTICK_PERIOD_MS))
                {
                    command_velocity = current_velocity;
                    xSemaphoreGive (commandVelocityMutex);
                }
            else
                {
                    ESP_LOGE (TAG, "Failed to take command velocity mutex");
                    continue;
                }
            if (xSemaphoreTake (commandSteeringMutex,
                                1000 / portTICK_PERIOD_MS))
                {
                    command_steering = current_steering;
                    xSemaphoreGive (commandSteeringMutex);
                }
            else
                {
                    ESP_LOGE (TAG, "Failed to take command steering mutex");
                    continue;
                }

            ESP_LOGI (TAG, "Velocity command created: %d", current_velocity);
            ESP_LOGI (TAG, "Steering command created: %d", current_steering);
            vTaskDelayUntil (&xLastWakeTime, xFrequency);
        }
}


void
fake_step_motor_task (void)
{
    config_step_motor_pins();
    int num_steps = 4700;
    int current_steps = 0;
    int step_to_move = num_steps - current_steps;
    gpio_set_level(PIN_A4988_EN, 1);
    while (1)
    {
        step_to_move = num_steps - current_steps;
        if (step_to_move == 0)
        {
            gpio_set_level(PIN_A4988_EN, 1);
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }
        if (step_to_move > 0)
        {
            gpio_set_level(PIN_A4988_DIR, 1);
            gpio_set_level(PIN_A4988_EN, 0);
            gpio_set_level(PIN_A4988_STEP, 0);
            vTaskDelay(pdMS_TO_TICKS(STEP_MOTOR_HALF_PERIOD));
            gpio_set_level(PIN_A4988_STEP, 1);
            current_steps++;
            ESP_LOGD (TAG, "Step %d", current_steps);
        }
        if (step_to_move < 0)
        {
            gpio_set_level(PIN_A4988_DIR, 0);
            gpio_set_level(PIN_A4988_EN, 0);
            gpio_set_level(PIN_A4988_STEP, 0);
            vTaskDelay(pdMS_TO_TICKS(STEP_MOTOR_HALF_PERIOD));
            gpio_set_level(PIN_A4988_STEP, 1);
            current_steps--;
            ESP_LOGD (TAG, "Step %d", current_steps);
        }
        vTaskDelay(pdMS_TO_TICKS(STEP_MOTOR_HALF_PERIOD));
    }
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
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = sensor_a,
        .level_gpio_num = sensor_b,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    
    ESP_ERROR_CHECK (
        pcnt_new_channel (pcnt_unit, &chan_config, &pcnt_chan));

    ESP_LOGI (TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK (pcnt_channel_set_edge_action (
        pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_LEVEL_ACTION_HOLD));
    ESP_ERROR_CHECK (pcnt_channel_set_level_action (
        pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_LEVEL_ACTION_HOLD));


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
measure_encoder_task ()
{
    pcnt_unit_handle_t pcnt_unit = encoder_setup(PIN_LEFT_ENCODER_A, PIN_LEFT_ENCODER_B);
    int pulse_count = 0;
    float current_velocity = 0;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = CALCULATE_FREQUENCY(0.2);
    xLastWakeTime = xTaskGetTickCount ();
    while (1)
        {
            ESP_ERROR_CHECK (pcnt_unit_get_count (pcnt_unit, &pulse_count));
            ESP_LOGI (TAG, "Left Encoder Pulse Count: %d", pulse_count);
            vTaskDelayUntil (&xLastWakeTime, xFrequency);
        }
}