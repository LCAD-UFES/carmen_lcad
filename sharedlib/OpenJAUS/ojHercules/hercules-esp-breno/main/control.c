#include "control.h"
#include "driver/ledc.h"
#include "math.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
static const char* TAG = "CONTROL module";

void
motor_control_setup()
{
    // Setup PWM control
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = DUTY_RESOLUTION,
        .freq_hz = 1000,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    ledc_channel_config_t pwm_left_channel = {
        .gpio_num = PIN_MOTOR_LEFT,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config_t pwm_right_channel = {
        .gpio_num = PIN_MOTOR_RIGHT,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&pwm_left_channel);
    ledc_channel_config(&pwm_right_channel);

    // Setup motor direction control
    gpio_config_t motor_direction_config = {
        .pin_bit_mask = (1ULL << PIN_MOTOR_LEFT_DIRECTION) | (1ULL << PIN_MOTOR_RIGHT_DIRECTION),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&motor_direction_config);
}

void
motor_task (void* parameters)
{
    motor_control_setup();

    // Control variables to be used in the task
    int velocity_can = 0;
    int steering_can = 0;
    double velocity_pwm = 0;
    double left_to_right_difference = 0;    
    int command_velocity_left = 0;
    int command_velocity_right = 0;

    double angle_can_to_rad = MAX_ANGLE / CAN_COMMAND_MAX;
    double velocity_can_to_pwm = pow(2, DUTY_RESOLUTION) / (CAN_COMMAND_MAX * (1 + WHEEL_SPACING * tan(MAX_ANGLE) / (2 * AXLE_SPACING)));
    double lef_to_right_difference_constant = WHEEL_SPACING / (2 * AXLE_SPACING);


    // Task frequency control
    int frequency = ((TaskParameters*)parameters)->frequency;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = frequency;
    xLastWakeTime = xTaskGetTickCount ();

    while(1) {
        // Read the global variables of command
        if (xSemaphoreTake (commandVelocityMutex, 1000 / portTICK_PERIOD_MS)){
            velocity_can = command_velocity;
            xSemaphoreGive (commandVelocityMutex);
        }
        else
        {
            ESP_LOGE (TAG, "Failed to take command velocity mutex");
            continue;
        }
        if (xSemaphoreTake (commandSteeringMutex, 1000 / portTICK_PERIOD_MS)){
            steering_can = command_steering;
            xSemaphoreGive (commandSteeringMutex);
        }
        else
        {
            ESP_LOGE (TAG, "Failed to take command steering mutex");
            continue;
        }
        ESP_LOGD (TAG, "Velocity: %d", velocity_can);
        ESP_LOGD (TAG, "Steering: %d", steering_can);

        // Calculate the velocity difference between the left and right motors
        velocity_pwm = velocity_can * velocity_can_to_pwm;
        ESP_LOGD (TAG, "Velocity PWM: %f", velocity_pwm);
        left_to_right_difference = steering_can * lef_to_right_difference_constant * angle_can_to_rad;
        command_velocity_right = round(velocity_pwm * (1 + left_to_right_difference));
        command_velocity_left = round(velocity_pwm * (1 - left_to_right_difference));
        ESP_LOGD (TAG, "Velocity left: %d", command_velocity_left);
        ESP_LOGD (TAG, "Velocity right: %d", command_velocity_right);

        // Set the direction of the motors
        if (velocity_pwm < 0) {
            gpio_set_level(PIN_MOTOR_LEFT_DIRECTION, 1);
            gpio_set_level(PIN_MOTOR_RIGHT_DIRECTION, 1);
        } else {
            gpio_set_level(PIN_MOTOR_LEFT_DIRECTION, 0);
            gpio_set_level(PIN_MOTOR_RIGHT_DIRECTION, 0);
        }

        // Set the duty cycle of the PWM signals
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, command_velocity_left);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, command_velocity_right);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);

        vTaskDelayUntil (&xLastWakeTime, xFrequency);
    }   
}

void
servo_task (void* parameters)
{
    // CAN task implementation using the global variables velocity and
    // velocityMutex
}