#include "control.h"
#include "driver/ledc.h"
#include "math.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
static const char* TAG = "CONTROL module";

void
motor_control_setup( void )
{
    // Setup PWM control
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = DUTY_RESOLUTION,
        .freq_hz = 18 * 1000, //kHz
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    ledc_channel_config_t pwm_left_channel = {
        .gpio_num = PIN_MOTOR_LEFT_PWM,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 100,
        .hpoint = 0
    };
    ledc_channel_config_t pwm_right_channel = {
        .gpio_num = PIN_MOTOR_RIGHT_PWM,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .duty = 100,
        .hpoint = 0
    };
    ledc_channel_config(&pwm_left_channel);
    ledc_channel_config(&pwm_right_channel);

    // Setup motor direction control
    gpio_config_t motor_direction_config = {
        .pin_bit_mask = (1ULL << PIN_MOTOR_DRIVE) | (1ULL << PIN_MOTOR_REVERSE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&motor_direction_config);
}

void
motor_task ( void )
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
    double left_to_right_difference_constant = WHEEL_SPACING / (2 * AXLE_SPACING);


    // Task frequency control
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = CALCULATE_FREQUENCY(TASK_MOTOR_FREQUENCY);
    xLastWakeTime = xTaskGetTickCount ();

    while(1) 
    {
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
        // ESP_LOGD (TAG, "Velocity: %d", velocity_can);
        // ESP_LOGD (TAG, "Steering: %d", steering_can);

        // Calculate the velocity difference between the left and right motors
        velocity_pwm = velocity_can * velocity_can_to_pwm;
        // ESP_LOGI (TAG, "Velocity PWM: %f", velocity_pwm);
        left_to_right_difference = steering_can * left_to_right_difference_constant * angle_can_to_rad;
        command_velocity_right = round(velocity_pwm * (1 + left_to_right_difference));
        command_velocity_left = round(velocity_pwm * (1 - left_to_right_difference));
        ESP_LOGD (TAG, "Velocity left: %d", command_velocity_left);
        ESP_LOGD (TAG, "Velocity right: %d", command_velocity_right);

        // Set the direction of the motors
        if (velocity_pwm < 0) {
            gpio_set_level(PIN_MOTOR_DRIVE, 0);
            gpio_set_level(PIN_MOTOR_REVERSE, 1);
        } else {
            gpio_set_level(PIN_MOTOR_DRIVE, 1);
            gpio_set_level(PIN_MOTOR_REVERSE, 0);
        }

        // Set the duty cycle of the PWM signals
        
        if (velocity_pwm < 0)
        {
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, -command_velocity_left);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, -command_velocity_right);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
        }
        else
        {
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, command_velocity_left);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, command_velocity_right);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
        }
        ESP_LOGD (TAG, "Duty cycle left: %d", command_velocity_left);
        ESP_LOGD (TAG, "Duty cycle right: %d", command_velocity_right);

        vTaskDelayUntil (&xLastWakeTime, xFrequency);
    }   
}

double target_limit_double(double insert,double low,double high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}

void
config_servo_pin( void )
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
servo_task ( void )
{   
    double received_command_steering = MEDIUM_T_HIGH; 
    double target_T_HIGH = 0;
    int duty_cycle = 0;

    double angle_can_to_T_HIGH_coefficient = ((MAX_T_HIGH - MIN_T_HIGH) / (2*CAN_COMMAND_MAX));

    // Task frequency control
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = CALCULATE_FREQUENCY(TASK_SERVO_FREQUENCY);
    xLastWakeTime = xTaskGetTickCount ();

    config_servo_pin();
    
    while (1)
    {
        if( xSemaphoreTake( commandSteeringMutex, 1000 / portTICK_PERIOD_MS))
        {
            received_command_steering = command_steering;
            xSemaphoreGive(commandSteeringMutex);
        }

        target_T_HIGH = (received_command_steering * angle_can_to_T_HIGH_coefficient) + MEDIUM_T_HIGH;
        target_T_HIGH = target_limit_double(target_T_HIGH, MIN_T_HIGH, MAX_T_HIGH);

        duty_cycle = (target_T_HIGH/LEDC_PERIOD)*LEDC_MAX_DUTY;

        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty_cycle);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

        vTaskDelayUntil (&xLastWakeTime, xFrequency);
    }
}

void
step_motor_task ( void )
{
    gpio_set_level(PIN_A4988_EN, 1);

    int current_step_motor_command = 0;
    int num_steps = 0;
    double step_motor_can_to_steps = NUM_STEPS_0_TO_100 / CAN_COMMAND_MAX;

    // Task frequency control
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = CALCULATE_FREQUENCY(TASK_STEP_MOTOR_FREQUENCY);
    xLastWakeTime = xTaskGetTickCount ();
    
    while (1)
    {
        if( xSemaphoreTake( commandStepMotorMutex, 1000 / portTICK_PERIOD_MS))
        {
            received_command_step_motor = command_step_motor;
            xSemaphoreGive(commandStepMotorMutex);
        }

        num_steps = (received_command_step_motor - current_step_motor_command) * step_motor_can_to_steps;
        if (num_steps < 0)
        {
            gpio_set_level(PIN_A4988_DIR, 0);
            num_steps = -num_steps;
        }
        else
        {
            gpio_set_level(PIN_A4988_DIR, 1);
        }
        gpio_set_level(PIN_A4988_EN, 0);
        for (int i = 0; i < num_steps; i++)
        {
            gpio_set_level(PIN_A4988_STEP, 0);
            vTaskDelay(pdMS_TO_TICKS(STEP_MOTOR_HALF_PERIOD));
            gpio_set_level(PIN_A4988_STEP, 1);
            vTaskDelay(pdMS_TO_TICKS(STEP_MOTOR_HALF_PERIOD));
        }
        gpio_set_level(PIN_A4988_EN, 1);

        vTaskDelayUntil (&xLastWakeTime, xFrequency);
    }
}