#ifndef SYSTEM_H
#define SYSTEM_H

#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

// Pins
#define PIN_CAN_TX GPIO_NUM_1
#define PIN_CAN_RX GPIO_NUM_2
#define PIN_ENCODER_A GPIO_NUM_3
#define PIN_ENCODER_B GPIO_NUM_4

// Structs
typedef struct TaskParameters
{
    uint32_t frequency;
} TaskParameters;

// Global variables
static int odom_velocity;
static int odom_steering;
static int command_velocity;
static int command_steering;
static SemaphoreHandle_t odomVelocityMutex;
static SemaphoreHandle_t odomSteeringMutex;
static SemaphoreHandle_t commandVelocityMutex;
static SemaphoreHandle_t commandSteeringMutex;
static TaskParameters can_reading_task_parameters = { .frequency = 30 };
static TaskParameters can_writing_task_parameters = { .frequency = 30 };
static TaskParameters motor_task_parameters = { .frequency = 30 };
static TaskParameters servo_task_parameters = { .frequency = 30 };
static TaskParameters encoder_task_parameters = { .frequency = 30 };
static TaskParameters steering_reading_parameters = { .frequency = 30 };

// CAN message IDs
#define ODOM_VELOCITY_CAN_ID 0x425
#define ODOM_STEERING_CAN_ID 0x80

// Encoders
#define PCNT_HIGH_LIMIT 100
#define PCNT_LOW_LIMIT -100

// Steering potentiometer
#define ADC_ANGLE_POTENTIOMETER ADC1_CHANNEL_7

// CAR measurements
#define WHEEL_DIAMETER 0.065
#define PI 3.14159265359f
#define GEAR_RATIO 30.0
#define WHEEL_SPACING 0.155f
#define AXLE_SPACING 0.143f
#define NUMBER_OF_ENCODER_LINES 500
#define ENCODER_PRECISION

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
