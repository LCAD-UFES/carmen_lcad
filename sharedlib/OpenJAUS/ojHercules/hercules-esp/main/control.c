#include "control.h"
// #include "esp_log.h "
#include "driver/ledc.h"
#include "driver/rmt.h"
#include "driver/rmt_tx.h"
#include "stepper_motor_encoder.h"
#include "math.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
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
config_step_motor_pins( void )
{
    // Setup step motor control
    gpio_config_t step_motor_config = {
        .pin_bit_mask = (1ULL << PIN_A4988_EN) | (1ULL << PIN_A4988_DIR), // | (1ULL << PIN_A4988_STEP),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&step_motor_config);
}

void init_rmt() {
    
}


void
step_motor_task ( void )
{
    config_step_motor_pins();
 
    rmt_channel_handle_t motor_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_REF_TICK, // select clock source
        .gpio_num = PIN_A4988_STEP,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &motor_chan));

    gpio_set_level(PIN_A4988_EN, 0);
    gpio_set_level(PIN_A4988_DIR, 1);

    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t uniform_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder));

    stepper_motor_curve_encoder_config_t accel_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = 500,
        .start_freq_hz = 500,
        .end_freq_hz = 1500,
    };
    rmt_encoder_handle_t accel_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&accel_encoder_config, &accel_motor_encoder));
    
    ESP_ERROR_CHECK(rmt_enable(motor_chan));
    rmt_transmit_config_t tx_config = {};

    const static uint32_t accel_samples = 500;
    const static uint32_t uniform_speed_hz = 1500;
    ESP_ERROR_CHECK(rmt_transmit(motor_chan, accel_motor_encoder, &accel_samples, sizeof(accel_samples), &tx_config));
    for (int i = 0; i < 3; i++)
    {
        // vTaskDelay(pdMS_TO_TICKS(100));
        ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, 1000));
    }

    // gpio_set_level(PIN_A4988_EN, 1);

    // int received_command_step_motor = 0;
    // int current_steps = 0;
    // int num_steps = 0;
    // double step_motor_can_to_steps = NUM_STEPS_0_TO_100 / CAN_COMMAND_MAX;

    // // Task frequency control
    // TickType_t xLastWakeTime;
    // const TickType_t xFrequency = CALCULATE_FREQUENCY(TASK_STEP_MOTOR_FREQUENCY);
    // xLastWakeTime = xTaskGetTickCount ();
    
    // while (1)
    // {
    //     if( xSemaphoreTake(commandStepMotorMutex, 1000 / portTICK_PERIOD_MS))
    //     {
    //         command_step_motor = 2000;
    //         received_command_step_motor = command_step_motor * step_motor_can_to_steps;
    //         xSemaphoreGive(commandStepMotorMutex);
    //     }
    //     num_steps = received_command_step_motor - current_steps;
    //     if (num_steps == 0)
    //     {
    //         gpio_set_level(PIN_A4988_EN, 1);
    //         vTaskDelayUntil(&xLastWakeTime, xFrequency);
    //         continue;
    //     }
    //     if (num_steps > 0)
    //     {
    //         for (int i = 0; i < MIN(num_steps, STEPS_PER_CYCLE); i++)
    //         {
    //             gpio_set_level(PIN_A4988_DIR, 1);
    //             gpio_set_level(PIN_A4988_EN, 0);
    //             gpio_set_level(PIN_A4988_STEP, 0);
    //             vTaskDelay(pdMS_TO_TICKS(STEP_MOTOR_HALF_PERIOD));
    //             gpio_set_level(PIN_A4988_STEP, 1);
    //             vTaskDelay(pdMS_TO_TICKS(1.0 * STEP_MOTOR_HALF_PERIOD));
    //             // ESP_LOGD (TAG, "Step %d", current_steps);
    //             // int teste = 0;
    //             // teste += 2;
    //             // teste -= 2;
    //         }
    //         current_steps += MIN(num_steps, STEPS_PER_CYCLE);
    //     }
    //     if (num_steps < 0)
    //     {
    //         for (int i = 0; i < MAX(num_steps, -STEPS_PER_CYCLE); i--)
    //         {
    //             gpio_set_level(PIN_A4988_DIR, 0);
    //             gpio_set_level(PIN_A4988_EN, 0);
    //             gpio_set_level(PIN_A4988_STEP, 0);
    //             vTaskDelay(pdMS_TO_TICKS(STEP_MOTOR_HALF_PERIOD/2));
    //             gpio_set_level(PIN_A4988_STEP, 1);
    //             vTaskDelay(pdMS_TO_TICKS(STEP_MOTOR_HALF_PERIOD));
    //             // ESP_LOGD (TAG, "Step %d", current_steps);
    //         }
    //         current_steps += MAX(num_steps, -STEPS_PER_CYCLE);
    //     }
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }


    // ESP_LOGI(TAG, "Initialize EN + DIR GPIO");
    // gpio_config_t en_dir_gpio_config = {
    //     .pin_bit_mask = (1ULL << PIN_A4988_EN) | (1ULL << PIN_A4988_DIR),
    //     .mode = GPIO_MODE_OUTPUT,
    //     .pull_up_en = GPIO_PULLUP_DISABLE,
    //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
    //     .intr_type = GPIO_INTR_DISABLE
    // };
    // ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config));
    
    // ESP_LOGI(TAG, "Create RMT TX channel");
    // rmt_channel_handle_t motor_chan = NULL;
    // rmt_tx_channel_config_t tx_chan_config = {
    //     .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
    //     .gpio_num = PIN_A4988_STEP,
    //     .mem_block_symbols = 64,
    //     .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
    //     .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    // };
    // ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &motor_chan));


    // ESP_LOGI(TAG, "Create motor encoders");
    // stepper_motor_curve_encoder_config_t accel_encoder_config = {
    //     .resolution = STEP_MOTOR_RESOLUTION_HZ,
    //     .sample_points = 500,
    //     .start_freq_hz = 500,
    //     .end_freq_hz = 1500,
    // };
    // rmt_encoder_handle_t accel_motor_encoder = NULL;
    // ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&accel_encoder_config, &accel_motor_encoder));

    // stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
    //     .resolution = STEP_MOTOR_RESOLUTION_HZ,
    // };
    // rmt_encoder_handle_t uniform_motor_encoder = NULL;
    // ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder));

    // stepper_motor_curve_encoder_config_t decel_encoder_config = {
    //     .resolution = STEP_MOTOR_RESOLUTION_HZ,
    //     .sample_points = 500,
    //     .start_freq_hz = 1500,
    //     .end_freq_hz = 500,
    // };
    // rmt_encoder_handle_t decel_motor_encoder = NULL;
    // ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&decel_encoder_config, &decel_motor_encoder));

    // ESP_LOGI(TAG, "Enable RMT channel");
    // ESP_ERROR_CHECK(rmt_enable(motor_chan));

    // ESP_LOGI(TAG, "Spin motor for 6000 steps: 500 accel + 5000 uniform + 500 decel");
    // rmt_transmit_config_t tx_config = {};

    // const static uint32_t accel_samples = 500;
    // const static uint32_t uniform_speed_hz = 1500;
    // const static uint32_t decel_samples = 500;

    // while (1) {
    //     // acceleration phase
    //     ESP_ERROR_CHECK(rmt_transmit(motor_chan, accel_motor_encoder, &accel_samples, sizeof(accel_samples), &tx_config));

    //     // uniform phase
    //     ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));

    //     // deceleration phase
    //     ESP_ERROR_CHECK(rmt_transmit(motor_chan, decel_motor_encoder, &decel_samples, sizeof(decel_samples), &tx_config));
    //     // wait all transactions finished
    //     ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));

    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
}