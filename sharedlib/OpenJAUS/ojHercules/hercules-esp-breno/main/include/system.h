#ifndef SYSTEM_H
#define SYSTEM_H


#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_log.h"


// Pins
// All GPIO's that don't mess with ESP32 boot : 4,13, 16-33, 34-39(Input Only)
// GPIO's that arent beeing used : 16,17,20,22,23,24,28 // Input Only : 34,36-39 
#define PIN_SERVO GPIO_NUM_18 
#define PIN_POTENTIOMETER GPIO_NUM_34 
#define ADC_CHANNEL_POTENTIOMETER  ADC1_CHANNEL_6
#define PIN_CAN_TX GPIO_NUM_16 // change pin later (PWM PIN)
#define PIN_CAN_RX GPIO_NUM_4 // change pin later (PWM PIN)
#define PIN_LEFT_ENCODER_A GPIO_NUM_14
#define PIN_LEFT_ENCODER_B GPIO_NUM_26
#define PIN_RIGHT_ENCODER_A GPIO_NUM_5
#define PIN_RIGHT_ENCODER_B GPIO_NUM_17   
#define PIN_MOTOR_DRIVE GPIO_NUM_32
#define PIN_MOTOR_REVERSE GPIO_NUM_13
#define PIN_MOTOR_LEFT_PWM GPIO_NUM_25
#define PIN_MOTOR_RIGHT_PWM GPIO_NUM_33


// Functions
#define CALCULATE_FREQUENCY(f) pdMS_TO_TICKS(1000 / ((float) f))


//Task Frequencies in Hz
#define TASK_CAN_FREQUENCY 100
#define TASK_ENCODER_FREQUENCY 1
#define TASK_MOTOR_FREQUENCY 1
#define TASK_SERVO_FREQUENCY 10
#define TASK_STEERING_FREQUENCY 1
#define TASK_TEST_FREQUENCY 1


// Global variables
extern double odom_left_velocity;
extern double odom_right_velocity;
extern int odom_steering;
extern int command_velocity;
extern int command_steering;// Angle Odometry Parameters
extern SemaphoreHandle_t odomLeftVelocityMutex;
extern SemaphoreHandle_t odomRightVelocityMutex;
extern SemaphoreHandle_t odomSteeringMutex;
extern SemaphoreHandle_t commandVelocityMutex;
extern SemaphoreHandle_t commandSteeringMutex;


// CAN params
#define ODOM_VELOCITY_CAN_ID 0x425
#define ODOM_STEERING_CAN_ID 0x80
#define COMMAND_CAN_ID 0x100
#define CAN_COMMAND_MAX 32767
#define VELOCITY_CONVERSION_CONSTANT 5000.0 // Must match carmen value in oj main.c


// Motors
#define DUTY_RESOLUTION 8


// Encoders
#define PCNT_HIGH_LIMIT 20000
#define PCNT_LOW_LIMIT -20000


// Steering potentiometer
#define ADC_ATTEN_POTENTIOMETER         ADC_ATTEN_DB_11
#define ADC_BITWIDTH_POTENTIOMETER     ADC_WIDTH_BIT_12
#define ADC_UNIT_POTENTIOMETER         ADC_UNIT_1
#define MIN_ANGLE_RESISTANCE 10500 // em ohms
#define MEDIUM_RESISTANCE 5350 // em ohms
#define MAX_ANGLE_RESISTANCE 200 // em ohms


// Sterring Parameters
#define MIN_ANGLE_T_HIGH 2100 // em us
#define MEDIUM_ANGLE_T_HIGH 1550 // em us
#define MAX_ANGLE_T_HIGH 1000 // em us
#define MAX_ANGLE 0.35 // em radianos, calcular com sentido anti-horário (curva para esquerda), talvez valor errado
#define MEDIUM_ANGLE 0.0 // em radianos, posição neutra
#define MIN_ANGLE (-0.35) // em radianos, calcular com sentido anti-horário (curva para direita), talvez valor errado
#define LINEAR_COEFFICIENT ((MAX_ANGLE-MEDIUM_ANGLE)/(MAX_ANGLE_T_HIGH-MEDIUM_ANGLE_T_HIGH)) // usado para estabelecer a relação entre ângulo e T_HIGH


// PWM
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
#define LEDC_PERIOD 1000000/LEDC_FREQUENCY // em us
#define LEDC_INITIAL_DUTY       ((MEDIUM_ANGLE_T_HIGH/LEDC_PERIOD)*LEDC_MAX_DUTY*0.5) // Set duty to (1500us/10000us). ((2 ** 11) - 1) 


// CAR measurements
#define WHEEL_DIAMETER 0.065
#define PI 3.14159265359f
#define GEAR_RATIO 30.0
#define WHEEL_SPACING 0.155f
#define AXLE_SPACING 0.143f
//#define MAX_ANGLE 0.30f
#define NUMBER_OF_ENCODER_LINES 500


#endif /* SYSTEM_H */
