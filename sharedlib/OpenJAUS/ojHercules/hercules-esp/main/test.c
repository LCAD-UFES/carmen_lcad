#include "test.h"
#include "control.h"

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
fake_step_motor_task ( void )
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