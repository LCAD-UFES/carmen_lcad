#include "control.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "driver/rmt.h"
#include "driver/rmt_tx.h"
#include "stepper_motor_encoder.h"
#include <math.h>
#include <string.h>
#include <inttypes.h>

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define CLAMP(min, val, max) MIN(MAX(min, val), max)
static const char* TAG = "CONTROL module";
const TickType_t xFrequencyTaskMotor = CALCULATE_FREQUENCY(TASK_MOTOR_FREQUENCY);
const TickType_t xFrequencyTaskServo = CALCULATE_FREQUENCY(TASK_SERVO_FREQUENCY);
const TickType_t xFrequencyTaskStepMotor = CALCULATE_FREQUENCY(TASK_STEP_MOTOR_FREQUENCY);
const double angle_can_to_T_HIGH_coefficient = ((MAX_T_HIGH - MIN_T_HIGH) / (2*CAN_COMMAND_MAX));
const double angle_can_to_rad = MAX_ANGLE / CAN_COMMAND_MAX;    
const double left_to_right_difference_constant = WHEEL_SPACING / (2 * AXLE_SPACING);
const double velocity_can_to_pwm = (MOTOR_MAX_PWM) / (CAN_COMMAND_MAX * (1 + WHEEL_SPACING * tan(MAX_ANGLE) / (2 * AXLE_SPACING)));

typedef struct PID
{
    double kp;
    double ki;
    double kd;
    double h; // Feedback gain
    double error_t_1;
    double integral_t;
    double integral_t_1;
    double u_t;
    double previous_t;
} PID;

typedef struct ConfigStepMotor
{
    rmt_channel_handle_t motor_chan;
    rmt_tx_channel_config_t tx_chan_config;
    stepper_motor_uniform_encoder_config_t accel_encoder_config; 
    stepper_motor_uniform_encoder_config_t uniform_encoder_config;
    stepper_motor_uniform_encoder_config_t decel_encoder_config;
    rmt_encoder_handle_t decel_motor_encoder;
    rmt_transmit_config_t tx_config;
} ConfigStepMotor;

void
motor_control_setup ()
{
    // Setup PWM control
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = MOTOR_DUTY_RESOLUTION,
        .freq_hz = 18 * 1000, //kHz
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK
    };
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
    ledc_timer_config(&pwm_timer);
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

double
get_time_sec ()
{
    int64_t t = esp_timer_get_time();
	double t_seconds = (double) t / 1000000.0;
	return (t_seconds);
}

int
motor_pid (PID *pid, int velocity_can, double current_velocity)
{
	// http://en.wikipedia.org/wiki/PID_controller -> Discrete implementation
	if (pid->previous_t == 0.0)
	{
		pid->previous_t = get_time_sec();
		return (0.0);
	}
	double t = get_time_sec();
	double delta_t = t - pid->previous_t;

	double error_t = velocity_can - current_velocity * pid->h;
    pid->integral_t = pid->integral_t + error_t * delta_t;
	double derivative_t = (error_t - pid->error_t_1) / delta_t;

	pid->u_t = MOTOR_PID_KP * error_t +
		       MOTOR_PID_KI * pid->integral_t +
		       MOTOR_PID_KD * derivative_t;

	pid->error_t_1 = error_t;

	// Anti windup
	if ((pid->u_t < -MOTOR_MAX_PWM) || (pid->u_t > MOTOR_MAX_PWM))
		pid->integral_t = pid->integral_t_1;
	pid->integral_t_1 = pid->integral_t;

	pid->previous_t = t;

	return CLAMP(-MOTOR_MAX_PWM, (int) round(pid->u_t), MOTOR_MAX_PWM);
}

int 
deadzone_correction (int velocity)
{
    if (velocity > 0)
    {
        return (MOTOR_DEAD_ZONE);
    }
    else if (velocity < 0)
    {
        return (-MOTOR_DEAD_ZONE);
    }
    else
    {
        return 0;
    }
}

void
set_motor_direction (int* left_pwm, int* right_pwm)
{
    if (*(left_pwm) < 0 || *(right_pwm) < 0) //Backwards
    {
        *(left_pwm)= -*(left_pwm);
        *(right_pwm)= -*(right_pwm);
        gpio_set_level(PIN_MOTOR_DRIVE, 0);
        gpio_set_level(PIN_MOTOR_REVERSE, 1);
    }
    else //Forward
    {
        gpio_set_level(PIN_MOTOR_DRIVE, 1);
        gpio_set_level(PIN_MOTOR_REVERSE, 0);
    }
}

void
motor_task ()
{
    motor_control_setup();
    
    // setting up PID for left and right motor
    PID left_pid = {
        .kp = MOTOR_PID_KP,
        .ki = MOTOR_PID_KI,
        .kd = MOTOR_PID_KD,
        .h = CAN_COMMAND_MAX / MAX_VELOCITY,
        .error_t_1 = 0.0,
        .integral_t = 0.0,
        .integral_t_1 = 0.0,
        .u_t = 0.0,
        .previous_t = 0.0
    };
    PID right_pid;
    memcpy(&right_pid, &left_pid, sizeof(PID));
    
    // Control variables to be used in the task
    int velocity_can = 0;
    int steering_can = 0;
    double left_to_right_difference = 0;
    int command_velocity_left = 0;
    int command_velocity_right = 0;
    int left_pwm = 0;
    int right_pwm = 0;
    #if MOTOR_USE_PID
        double left_current_velocity = 0;
        double right_current_velocity = 0;
    #endif

    // Task frequency control
    // TickType_t xLastWakeTime;
    // const TickType_t xFrequency = CALCULATE_FREQUENCY(TASK_MOTOR_FREQUENCY);
    // xLastWakeTime = xTaskGetTickCount ();
    TickType_t xLastWakeTime = xTaskGetTickCount ();

    while(1) 
    {
        // Read the global variables of command
        velocity_can = get_command_velocity();
        // if (xSemaphoreTake (commandVelocityMutex, 1000 / portTICK_PERIOD_MS)){
        //     velocity_can = command_velocity;
        //     xSemaphoreGive (commandVelocityMutex);
        // }
        // else
        // {
        //     ESP_LOGE (TAG, "Failed to take command velocity mutex");
        //     continue;
        // }
        steering_can = get_command_steering();
        // if (xSemaphoreTake (commandSteeringMutex, 1000 / portTICK_PERIOD_MS)){
        //     steering_can = command_steering;
        //     xSemaphoreGive (commandSteeringMutex);
        // }
        // else
        // {
        //     ESP_LOGE (TAG, "Failed to take command steering mutex");
        //     continue;
        // }
        ESP_LOGD (TAG, "CAN Velocity command: %d", velocity_can);
        ESP_LOGD (TAG, "CAN Steering command: %d", steering_can);

        velocity_can += deadzone_correction(velocity_can);
        // if (velocity_can > 0)
        // {
        //     velocity_can += MOTOR_DEAD_ZONE;
        // }
        // else if (velocity_can < 0)
        // {
        //     velocity_can -= MOTOR_DEAD_ZONE;
        // }
        left_to_right_difference = steering_can * left_to_right_difference_constant * angle_can_to_rad;
        command_velocity_right = round(velocity_can * (1 + left_to_right_difference));
        command_velocity_left = round(velocity_can * (1 - left_to_right_difference));

        ESP_LOGD (TAG, "Command velocity left: %d, Command velocity right: %d", command_velocity_left, command_velocity_right);

        #if MOTOR_USE_PID
        left_current_velocity = get_odom_left_velocity();
        // if (xSemaphoreTake (odomLeftVelocityMutex, 1000 / portTICK_PERIOD_MS)){
        //     left_current_velocity = odom_left_velocity;
        //     xSemaphoreGive (commandVelocityMutex);
        // }
        left_pwm = motor_pid(&left_pid, command_velocity_left, left_current_velocity);

        right_current_velocity = get_odom_right_velocity();
        // if (xSemaphoreTake (odomRightVelocityMutex, 1000 / portTICK_PERIOD_MS)){
        //     right_current_velocity = odom_right_velocity;
        //     xSemaphoreGive (commandVelocityMutex);
        // }
        right_pwm = motor_pid(&right_pid, command_velocity_right, right_current_velocity);
        #else
        left_pwm = command_velocity_left * velocity_can_to_pwm;
        right_pwm = command_velocity_right * velocity_can_to_pwm;
        #endif

        set_motor_direction(&left_pwm,&right_pwm);

        //if (left_pwm < 0)
        //{
        //   left_pwm = -left_pwm;
        //    gpio_set_level(PIN_MOTOR_DRIVE, 0);
        //    gpio_set_level(PIN_MOTOR_REVERSE, 1);
        //}
        //else
        //{
        //    gpio_set_level(PIN_MOTOR_DRIVE, 1);
        //    gpio_set_level(PIN_MOTOR_REVERSE, 0);
        //}
        //if (right_pwm < 0)
        //{
        //    right_pwm = -right_pwm;
        //}

        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, left_pwm);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, right_pwm);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);

        ESP_LOGD(TAG, "Left PWM: %d, Right PWM: %d", left_pwm, right_pwm);

        vTaskDelayUntil (&xLastWakeTime, xFrequencyTaskMotor);
    }   
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

int
calculate_dutyCycle (double T_High)
{
    return((T_High/LEDC_PERIOD)*LEDC_MAX_DUTY);
}

void
servo_task ()
{   
    // Variables used for controlling the servo
    int received_command_steering = MEDIUM_T_HIGH; 
    double converted_received_command_steering = (double)received_command_steering;
    double target_T_HIGH = 0;
    int duty_cycle = 0;

    // Task frequency control
    // TickType_t xLastWakeTime;
    // const TickType_t xFrequency = CALCULATE_FREQUENCY(TASK_SERVO_FREQUENCY);
    // xLastWakeTime = xTaskGetTickCount ();
    TickType_t xLastWakeTime = xTaskGetTickCount ();

    config_servo_pin();
    
    while (1)
    {
        received_command_steering = get_command_steering();
        converted_received_command_steering = (double)received_command_steering;
        // if( xSemaphoreTake( commandSteeringMutex, 1000 / portTICK_PERIOD_MS))
        // {
        //     received_command_steering = command_steering;
        //     xSemaphoreGive(commandSteeringMutex);
        // }
        
        target_T_HIGH = (converted_received_command_steering * angle_can_to_T_HIGH_coefficient) + MEDIUM_T_HIGH + SERVO_BIAS;
        target_T_HIGH = target_limit_double(target_T_HIGH, MIN_T_HIGH, MAX_T_HIGH);

        duty_cycle = calculate_dutyCycle(target_T_HIGH);

        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty_cycle);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

        vTaskDelayUntil (&xLastWakeTime, xFrequencyTaskServo);
    }
}

ConfigStepMotor
config_step_motor ()
{
    ConfigStepMotor config;
    
    config->motor_chan = NULL;
    config->tx_chan_config = {
        .clk_src = RMT_CLK_SRC_REF_TICK, // select clock source
        .gpio_num = PIN_A4988_STEP,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&(config->tx_chan_config), &(config->motor_chan)));

    gpio_set_level(PIN_A4988_EN, 0);
    gpio_set_level(PIN_A4988_DIR, 1);

    return config;
}

void
config_step_motor_pins ()
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

void
step_motor_task ()
{
    config_step_motor_pins();

    ConfigStepMotor config = config_step_motor();
 
    // rmt_channel_handle_t motor_chan = NULL;
    // rmt_tx_channel_config_t tx_chan_config = {
    //     .clk_src = RMT_CLK_SRC_REF_TICK, // select clock source
    //     .gpio_num = PIN_A4988_STEP,
    //     .mem_block_symbols = 64,
    //     .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
    //     .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    // };
    // ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &motor_chan));

    //gpio_set_level(PIN_A4988_EN, 0);
    //gpio_set_level(PIN_A4988_DIR, 1);

    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .freq_hz = STEP_MOTOR_INITIAL_SPEED_HZ,
        .sample_points = 0,
    };
    rmt_encoder_handle_t uniform_motor_encoder = NULL;

    stepper_motor_curve_encoder_config_t accel_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = 500,
        .start_freq_hz = STEP_MOTOR_INITIAL_SPEED_HZ,
        .end_freq_hz = 1500,
    };
    rmt_encoder_handle_t accel_motor_encoder = NULL;

    stepper_motor_curve_encoder_config_t decel_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = 500,
        .start_freq_hz = 1500,
        .end_freq_hz = STEP_MOTOR_INITIAL_SPEED_HZ,
    };
    rmt_encoder_handle_t decel_motor_encoder = NULL;
    
    ESP_ERROR_CHECK(rmt_enable(motor_chan));
    rmt_transmit_config_t tx_config = {};

    int received_command_step_motor = 0;
    int current_steps = 0;
    int num_steps = 0;
    int transient_steps = 0;
    int uniform_steps = 0;
    double step_motor_can_to_steps = NUM_STEPS_0_TO_100 / CAN_COMMAND_MAX;
    uint32_t accel_samples;
    uint32_t uniform_speed_hz;
    uint32_t decel_samples;

    // Task frequency control
    // TickType_t xLastWakeTime;
    // const TickType_t xFrequency = CALCULATE_FREQUENCY(TASK_STEP_MOTOR_FREQUENCY);
    // xLastWakeTime = xTaskGetTickCount ();
    TickType_t xLastWakeTime = xTaskGetTickCount ();
    
    while (1)
    {
        received_command_step_motor = get_command_step_motor();
        received_command_step_motor = received_command_step_motor*step_motor_can_to_steps;
        // if( xSemaphoreTake(commandStepMotorMutex, 1000 / portTICK_PERIOD_MS))
        // {
        //     received_command_step_motor = command_step_motor * step_motor_can_to_steps;
        //     xSemaphoreGive(commandStepMotorMutex);
        // }
        num_steps = received_command_step_motor - current_steps;

        if (num_steps == 0)
        {
            gpio_set_level(PIN_A4988_EN, 1);
            vTaskDelayUntil(&xLastWakeTime, xFrequencyTaskStepMotor);
            continue;
        }


        gpio_set_level(PIN_A4988_EN, 0);
        if (num_steps > 0)
        {
            gpio_set_level(PIN_A4988_DIR, 1);
        }
        else // if num_steps < 0
        {
            num_steps = -num_steps;
            gpio_set_level(PIN_A4988_DIR, 0);
        }
        
        // Do the necessary calculations
        transient_steps = MIN(num_steps, STEP_MOTOR_MAX_TRANSIENT_STEPS);
        accel_samples = transient_steps / 2;
        decel_samples = accel_samples;
        uniform_speed_hz = STEP_MOTOR_INITIAL_SPEED_HZ + STEP_MOTOR_ACCEL_HZ_PER_S * accel_samples;
        uniform_steps = num_steps - accel_samples - decel_samples;

        // Prepare acceleration phase
        accel_encoder_config.sample_points = accel_samples;
        accel_encoder_config.end_freq_hz = uniform_speed_hz;

        // Prepare desacceleration phase
        decel_encoder_config.sample_points = decel_samples;
        decel_encoder_config.start_freq_hz = uniform_speed_hz;

        // Setup both phases
        ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&accel_encoder_config, &accel_motor_encoder));
        ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&decel_encoder_config, &decel_motor_encoder));
        
        // Setup uniform phase, if necessary
        if (uniform_steps)
        {
            uniform_encoder_config.sample_points = uniform_steps;
            uniform_encoder_config.freq_hz = uniform_speed_hz;
            ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder));
        }

        // Execute acceleration phase
        ESP_ERROR_CHECK(rmt_transmit(motor_chan, accel_motor_encoder, &accel_samples, sizeof(accel_samples), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, 10000));
        // Execute uniform phase, if necessary
        if (uniform_steps)
        {
            ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, 10000));
        }
        // Execute desacceleration phase
        ESP_ERROR_CHECK(rmt_transmit(motor_chan, decel_motor_encoder, &decel_samples, sizeof(decel_samples), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, 10000));

        printf("accel_steps: %"PRIu32", uniform_steps: %d, decel_steps: %"PRIu32"", accel_samples, uniform_steps, decel_samples);

        current_steps = received_command_step_motor;
        ESP_LOGD(TAG, "Current steps: %d", current_steps);
        
        vTaskDelayUntil (&xLastWakeTime, xFrequencyTaskStepMotor);
    }
}