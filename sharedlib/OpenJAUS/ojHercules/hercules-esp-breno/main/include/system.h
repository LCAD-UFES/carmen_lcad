#ifndef SYSTEM_H
#define SYSTEM_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Functions
#define CALCULATE_FREQUENCY(f) pdMS_TO_TICKS(1000 / ((float) f))

// Pins
#define PIN_SERVO GPIO_NUM_21 // pino do servo
#define PIN_CAN_TX GPIO_NUM_26
#define PIN_CAN_RX GPIO_NUM_27
#define PIN_ENCODER_A GPIO_NUM_5
#define PIN_ENCODER_B GPIO_NUM_4    
#define PIN_MOTOR_LEFT GPIO_NUM_2
#define PIN_MOTOR_RIGHT GPIO_NUM_15
#define PIN_MOTOR_LEFT_DIRECTION GPIO_NUM_0
#define PIN_MOTOR_RIGHT_DIRECTION GPIO_NUM_5

// Structs
typedef struct TaskParameters
{
    TickType_t frequency;
} TaskParameters;

// Global variables
extern double odom_velocity;
extern int odom_steering;
extern int command_velocity;
extern int command_steering;// Angle Odometry Parameters
extern int command_steering;
extern SemaphoreHandle_t odomVelocityMutex;
extern SemaphoreHandle_t odomSteeringMutex;
extern SemaphoreHandle_t commandVelocityMutex;
extern SemaphoreHandle_t commandSteeringMutex;

#define MAX_ANGLE 0.35 // em radianos, calcular com sentido anti-horário (curva para esquerda), talvez valor errado
#define MEDIUM_ANGLE 0.0 // em radianos, posição neutra
#define MIN_ANGLE (-0.35) // em radianos, calcular com sentido anti-horário (curva para direita), talvez valor errado

#define LINEAR_COEFFICIENT ((MAX_ANGLE-MEDIUM_ANGLE)/(MAX_ANGLE_T_HIGH-MEDIUM_ANGLE_T_HIGH))frequency = CALCULATE_FREQUENCY(1) ;
extern static TaskParameters motor_task_parameters = { .frequency = CALCULATE_FREQUENCY(100) };
extern static TaskParameters servo_task_parameters = { .frequency = CALCULATE_FREQUENCY(1) };
extern static TaskParameters encoder_task_parameters = { .frequency = CALCULATE_FREQUENCY(1) };
extern static TaskParameters steering_reading_parameters = { .frequency = CALCULATE_FREQUENCY(1) };
extern static TaskParameters fake_odometry_task_parameters = { .frequency = CALCULATE_FREQUENCY(1) };
extern static TaskParameters fake_commands_task_parameters = { .frequency = CALCULATE_FREQUENCY(10) };
extern static TaskParameters can_reading_task_parameters = { .frequency = CALCULATE_FREQUENCY(1)};
extern static TaskParameters can_writing_task_parameters = { .frequency = CALCULATE_FREQUENCY(1) };

// CAN params
#define ODOM_VELOCITY_CAN_ID 0x425
#define ODOM_STEERING_CAN_ID 0x80
#define COMMAND_CAN_ID 0x100
#define CAN_COMMAND_MAX 32767
#define VELOCITY_CONVERSION_CONSTANT 5000.0 // Must match carmen value in oj main.c

// Motors
#define DUTY_RESOLUTION 14

// Encoders
#define PCNT_HIGH_LIMIT 20000
#define PCNT_LOW_LIMIT -20000

// Steering potentiometer
#define ADC_ANGLE_POTENTIOMETER ADC1_CHANNEL_7
#define MIN_ANGLE_RESISTANCE 10500 // em ohms
#define MEDIUM_RESISTANCE 5350 // em ohms
#define MAX_ANGLE_RESISTANCE 200 // em ohms

// Sterring Parameters
#define MIN_ANGLE_T_HIGH 2100 // em us
#define MEDIUM_ANGLE_T_HIGH 1550 // em us
#define MAX_ANGLE_T_HIGH 1000 // em us

//#define MAX_ANGLE 0.35 // em radianos, calcular com sentido anti-horário (curva para esquerda), talvez valor errado
#define MEDIUM_ANGLE 0.0 // em radianos, posição neutra
//#define MIN_ANGLE (-0.35) // em radianos, calcular com sentido anti-horário (curva para direita), talvez valor errado

#define LINEAR_COEFFICIENT ((MAX_ANGLE-MEDIUM_ANGLE)/(MAX_ANGLE_T_HIGH-MEDIUM_ANGLE_T_HIGH)) // usado para estabelecer a relação entre ângulo e T_HIGH

// PWM
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          SERVO_PIN // Define the output GPIO
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

// ADC Attenuation
#define ADC_ATTEN ADC_ATTEN_DB_11

// ADC Calibration
#if CONFIG_IDF_TARGET_ESP32
#define ADC_CALI_SCHEME ESP_ADC_CAL_VAL_EFUSE_VREF
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_CALI_SCHEME ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32C3
#define ADC_CALI_SCHEME ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_CALI_SCHEME ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif


#endif /* SYSTEM_H */
