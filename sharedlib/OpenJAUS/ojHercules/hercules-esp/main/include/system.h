#ifndef SYSTEM_H
#define SYSTEM_H


#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_log.h"


// Single Motor
#define ONLY_LEFT_MOTOR 0
#define ONLY_RIGHT_MOTOR 0
#define TWO_MOTORS (~(ONLY_LEFT_MOTOR | ONLY_RIGHT_MOTOR))

// CAR measurements
#define WHEEL_DIAMETER 0.262f   
#define GEAR_RATIO 30.0
#define WHEEL_SPACING 0.155f
#define AXLE_SPACING 0.143f
#define MAX_ANGLE 0.30f
#define NUMBER_OF_ENCODER_LINES 500.0
#define PULSES_PER_REVOLUTION 9014.9f
#define MAX_VELOCITY 3.0

// Utils
#define CALCULATE_FREQUENCY(f) pdMS_TO_TICKS(1000 / ((float) f))
#define PI 3.14159265359f


// Pins
// All GPIO's that don't mess with ESP32 boot : 4,13, 14, 16-33, 34-39(Input Only)
// GPIO's that arent beeing used : 16,17,20,22,23,24,28 // Input Only : 34,36-39 
#define PIN_SERVO GPIO_NUM_18 
#define PIN_POTENTIOMETER GPIO_NUM_34 
#define ADC_CHANNEL_POTENTIOMETER  ADC1_CHANNEL_6
#define PIN_CAN_TX GPIO_NUM_16 // change pin later (PWM PIN)
#define PIN_CAN_RX GPIO_NUM_4 // change pin later (PWM PIN)
#define PIN_RIGHT_ENCODER_A GPIO_NUM_17
#define PIN_RIGHT_ENCODER_B GPIO_NUM_5
#define PIN_LEFT_ENCODER_A GPIO_NUM_26
#define PIN_LEFT_ENCODER_B GPIO_NUM_14   
#define PIN_MOTOR_DRIVE GPIO_NUM_32
#define PIN_MOTOR_REVERSE GPIO_NUM_13
#define PIN_MOTOR_LEFT_PWM GPIO_NUM_25
#define PIN_MOTOR_RIGHT_PWM GPIO_NUM_33
#define PIN_A4988_DIR GPIO_NUM_22
#define PIN_A4988_STEP GPIO_NUM_23
#define PIN_A4988_EN GPIO_NUM_19


//Task Frequencies in Hz
#define TASK_CAN_FREQUENCY 100
#define TASK_ENCODER_FREQUENCY 50
#define TASK_MOTOR_FREQUENCY 50
#define TASK_SERVO_FREQUENCY 1
#define TASK_STEERING_FREQUENCY 1
#define TASK_STEP_MOTOR_FREQUENCY 1
#define TASK_TEST_FREQUENCY 1


// Global variables
extern double odom_left_velocity;
extern double odom_right_velocity;
extern int odom_steering;
extern int command_velocity;
extern int command_steering;
extern int command_step_motor;
extern SemaphoreHandle_t odomLeftVelocityMutex;
extern SemaphoreHandle_t odomRightVelocityMutex;
extern SemaphoreHandle_t odomSteeringMutex;
extern SemaphoreHandle_t commandVelocityMutex;
extern SemaphoreHandle_t commandSteeringMutex;
extern SemaphoreHandle_t commandStepMotorMutex;


// Functions to properly interact with the global variables using the Mutexes
int get_odom_steering();
double get_odom_left_velocity();
double get_odom_right_velocity();
int get_command_steering();
int get_command_velocity();
int get_command_step_motor();

void set_odom_steering(int new_odom_steering);
void set_odom_left_velocity(double new_left_velocity);
void set_odom_right_velocity(double new_odom_right_velocity);
void set_command_velocity(int new_command_velocity);
void set_command_steering(int new_command_steering);
void set_command_step_motor(int new_command_step_motor);

// Limiting Functions
double target_limit_double(double insert,double low,double high);

// CAN params
#define ODOM_VELOCITY_CAN_ID 0x425
#define ODOM_STEERING_CAN_ID 0x80
#define COMMAND_CAN_CAR_ID 0x100
#define COMMAND_CAN_STEP_MOTOR_ID 0x90
#define CAN_COMMAND_MAX (100 * 256)
#define VELOCITY_CONVERSION_CONSTANT 5000.0 // Must match carmen value in oj main.c
#define CAM_LIMIT_MAX 65536


// Motors
#define MOTOR_DUTY_RESOLUTION 8
#define MOTOR_USE_PID 0
#define MOTOR_PID_KP 100.0
#define MOTOR_PID_KI 0.0
#define MOTOR_PID_KD 0.0
#define MOTOR_MAX_PWM ((1 << MOTOR_DUTY_RESOLUTION) - 1)
#define MOTOR_DEAD_ZONE 2560


// Encoders
#define PCNT_HIGH_LIMIT 20000
#define PCNT_LOW_LIMIT -20000


// Steering potentiometer
#define ADC_ATTEN_POTENTIOMETER         ADC_ATTEN_DB_11
#define ADC_BITWIDTH_POTENTIOMETER     ADC_WIDTH_BIT_12
#define ADC_UNIT_POTENTIOMETER         ADC_UNIT_1


// Sterring Parameters
#define MIN_T_HIGH 1000.0 // in us
#define MAX_T_HIGH 2100.0 // in us
#define MEDIUM_T_HIGH ((MAX_T_HIGH + MIN_T_HIGH) / 2) // in us


// Step Motor
#define NUM_STEPS_0_TO_100 4700.0
#define STEP_MOTOR_HALF_PERIOD 1000.0 // 5.0 // in ms
#define STEP_MOTOR_RESOLUTION_HZ 100000.0
#define STEPS_PER_CYCLE 4000
#define STEP_MOTOR_INITIAL_SPEED_HZ 75
#define STEP_MOTOR_ACCEL_HZ_PER_S 50
#define STEP_MOTOR_MAX_SPEED_HZ 1500
#define STEP_MOTOR_MAX_TRANSIENT_STEPS (STEP_MOTOR_MAX_SPEED_HZ - STEP_MOTOR_INITIAL_SPEED_HZ) / STEP_MOTOR_ACCEL_HZ_PER_S


// Servo
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          PIN_SERVO // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_11_BIT // Set duty resolution to 11 bits
// duty goes from 0 to (2^11 - 1)
#if (LEDC_DUTY_RES == LEDC_TIMER_11_BIT)
#define LEDC_MAX_DUTY 2047
#endif
#define LEDC_FREQUENCY          100 // Frequency in Hertz.
#define LEDC_PERIOD (1000000/LEDC_FREQUENCY) // in us
#define LEDC_INITIAL_DUTY       ((MEDIUM_T_HIGH/LEDC_PERIOD)*LEDC_MAX_DUTY) // Set duty to (1500us/10000us). ((2 ** 11) - 1) 
#define SERVO_BIAS -35

#endif /* SYSTEM_H */
