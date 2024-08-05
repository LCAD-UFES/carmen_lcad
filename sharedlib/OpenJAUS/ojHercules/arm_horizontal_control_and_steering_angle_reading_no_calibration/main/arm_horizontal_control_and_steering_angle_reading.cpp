#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "driver/adc_common.h"
#include "esp_adc_cal.h"

#include "Arduino.h"
#include "driver/mcpwm.h"

#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/can.h"

#include "driver/pcnt.h"
#include "esp_attr.h"

// GENERAL CONSTANTS

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
#define TASK_SERVO_FREQUENCY 50
#define TASK_STEERING_FREQUENCY 50
#define TASK_STEP_MOTOR_FREQUENCY 1
#define TASK_TEST_FREQUENCY 1
#define TASK_RESET_ERROR_AND_ANGLE_FREQUENCY 1

// CAR measurements
#define WHEEL_DIAMETER 0.262f
#define GEAR_RATIO 30.0
#define WHEEL_SPACING 0.155f
#define AXLE_SPACING 0.143f
#define MAX_ANGLE 0.30f
#define NUMBER_OF_ENCODER_LINES 500.0
#define PULSES_PER_REVOLUTION 9014.9f
#define MAX_VELOCITY 3.0

// CAN params
#define ODOM_VELOCITY_CAN_ID 		0x425
#define ODOM_STEERING_CAN_ID 		0x80
#define COMMAND_CAN_CAR_ID 			0x100
#define COMMAND_CAN_STEP_MOTOR_ID 	0x90
#define CAN_COMMAND_MAX 			(100 * 256)
#define VELOCITY_CONVERSION_CONSTANT (CAN_COMMAND_MAX / MAX_VELOCITY) // Must match carmen value in oj main.c
#define CAM_LIMIT_MAX 				65536

// end GENERAL CONSTANTS


// MOTORS

#define LEFT_ENCODER 	PCNT_UNIT_0
#define RIGHT_ENCODER	PCNT_UNIT_1

#define PCNT_H_LIM_VAL      20000
#define PCNT_L_LIM_VAL     -20000

const double meters_per_second_per_pulse = (TASK_ENCODER_FREQUENCY * PI * WHEEL_DIAMETER) / (PULSES_PER_REVOLUTION);

// end MOTORS


// ADC
// ADC Channels
#define ADC_ANGLE_POTENTIOMETER     ADC1_CHANNEL_6

// ADC Attenuation
#define ADC_EXAMPLE_ATTEN           ADC_ATTEN_DB_11

// ADC Calibration
#if CONFIG_IDF_TARGET_ESP32
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32C3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif

// end ADC


//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH    1100 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH    1900 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE        180 //Maximum angle in degree upto which servo can rotate
#define SERVO_MIN_DEGREE        0 //Minimum angle in degree upto which servo can rotate
#define SERVO_PULSE_GPIO        (18)   // GPIO connects to the PWM signal line

static esp_adc_cal_characteristics_t adc1_chars;

static const char *TAG = "Stepper Motor, angle measurement and steering servo control";

double current_steering_angle = (double) (SERVO_MIN_DEGREE + (SERVO_MAX_DEGREE + SERVO_MIN_DEGREE) / 2.0);

// CAN

#define NO_OF_ITERS                     3
#define CAN_RECEIVE_TASK_PRIORITY       9
#define CAN_SEND_ODOMETRY_TASK_PRIORITY 9
#define TX_GPIO_NUM                     16
#define RX_GPIO_NUM                     4
#define EXAMPLE_TAG                     "CAN Listen Only"

static const can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
static const can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
//Set TX queue length to 0 due to listen only mode
static const can_general_config_t g_config = {.mode = CAN_MODE_NORMAL,
                                              .tx_io = (gpio_num_t) TX_GPIO_NUM, .rx_io = (gpio_num_t) RX_GPIO_NUM,
                                              .clkout_io = CAN_IO_UNUSED, .bus_off_io = CAN_IO_UNUSED,
                                              .tx_queue_len = 5, .rx_queue_len = 5,
                                              .alerts_enabled = CAN_ALERT_NONE, .clkout_divider = 0, .intr_flags = ESP_INTR_FLAG_LEVEL1};

static SemaphoreHandle_t rx_sem;

can_message_t can_steering_and_velocity_efforts;
can_message_t can_odometry_velocity;
can_message_t can_odometry_steering;

// end CAN

uint32_t voltage = 0;
double odom_left_velocity = 0.0;
double odom_right_velocity = 0.0;


static void
encoder_setup(int sensor_a, int sensor_b, pcnt_unit_t unit)
{
	/* Prepare configuration for the PCNT unit */
	pcnt_config_t pcnt_config =	{ };
	// Set PCNT input signal and control GPIOs
	pcnt_config.pulse_gpio_num = sensor_a;
	pcnt_config.ctrl_gpio_num = sensor_b;
	pcnt_config.channel = PCNT_CHANNEL_0;
	pcnt_config.unit = unit;
	// What to do on the positive / negative edge of pulse input?
	pcnt_config.pos_mode = PCNT_COUNT_INC;   // Count up on the positive edge
	pcnt_config.neg_mode = PCNT_COUNT_DIS;   // Keep the counter value on the negative edge
	// What to do when control input is low or high?
	pcnt_config.lctrl_mode = PCNT_MODE_REVERSE; // Reverse counting direction if low
	pcnt_config.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode if high
	// Set the maximum and minimum limit values to watch
	pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;
	pcnt_config.counter_l_lim = PCNT_L_LIM_VAL;

    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(unit, 100);
    pcnt_filter_enable(unit);

//    /* Set threshold 0 and 1 values and enable events to watch */
//    pcnt_set_event_value(unit, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
//    pcnt_event_enable(unit, PCNT_EVT_THRES_1);
//    pcnt_set_event_value(unit, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
//    pcnt_event_enable(unit, PCNT_EVT_THRES_0);
//    /* Enable events on zero, maximum and minimum limit values */
//    pcnt_event_enable(unit, PCNT_EVT_ZERO);
//    pcnt_event_enable(unit, PCNT_EVT_H_LIM);
//    pcnt_event_enable(unit, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);

//    /* Install interrupt service and add isr callback handler */
//    pcnt_isr_service_install(0);
//    pcnt_isr_handler_add(unit, pcnt_example_intr_handler, (void *)unit);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(unit);
}


double
read_encoder(pcnt_unit_t unit)
{
	int16_t pulse_count;
    pcnt_get_counter_value(unit, &pulse_count);

    return (pulse_count * meters_per_second_per_pulse);
}


static void
left_encoder_task(void *arg)
{
    encoder_setup(PIN_LEFT_ENCODER_A, PIN_LEFT_ENCODER_B, LEFT_ENCODER);

    while (1)
    {
    	odom_left_velocity = read_encoder(LEFT_ENCODER);
		vTaskDelay((1000 / 40) / portTICK_PERIOD_MS);
    }
}


static void
right_encoder_task(void *arg)
{
    encoder_setup(PIN_RIGHT_ENCODER_A, PIN_RIGHT_ENCODER_B, RIGHT_ENCODER);

    while (1)
    {
    	odom_right_velocity = read_encoder(RIGHT_ENCODER);
		vTaskDelay((1000 / 40) / portTICK_PERIOD_MS);
    }
}


static bool
adc_init(void)
{
    esp_err_t ret;
    bool cali_enable = false;

    ret = esp_adc_cal_check_efuse(ADC_EXAMPLE_CALI_SCHEME);
    if (ret == ESP_ERR_NOT_SUPPORTED) 
    {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    } 
    else if (ret == ESP_ERR_INVALID_VERSION) 
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } 
    else if (ret == ESP_OK) 
    {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
    } 
    else 
    {
        ESP_LOGE(TAG, "Invalid arg");
    }

    // Angle potentiometer ADC config
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_ANGLE_POTENTIOMETER, ADC_EXAMPLE_ATTEN));

    return (cali_enable);
}


uint32_t
get_voltage(uint32_t voltage)
{
	for (int i = 0; i < 200; i++)
	{
		uint32_t adc_reading = adc1_get_raw(ADC_ANGLE_POTENTIOMETER);
		voltage += esp_adc_cal_raw_to_voltage(adc_reading, &adc1_chars);
		vTaskDelay(0);
	}
	voltage /= 200;

	return (voltage);
}


static uint32_t 
servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}


void 
init_steering_servo_control(void)
{
    // 1. mcpwm gpio initialization
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PULSE_GPIO);  // Set GPIO 18 as PWM0A, to which servo is connected

    // 2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 100; // frequency = 100Hz, i.e. for this servo motor, time period should be 10ms
    pwm_config.cmpr_a = 0;      // duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;      // duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    // Configure PWM0A & PWM0B with above settings
}


void
set_steering_servo_angle(uint32_t angle)
{
    uint32_t angle_in_us = servo_per_degree_init(angle);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle_in_us);
}


void
set_steering_effort(short int steering_effort)
{
    current_steering_angle += (double) steering_effort / 10.0;
    if (current_steering_angle > SERVO_MAX_DEGREE)
        current_steering_angle = SERVO_MAX_DEGREE;
    else if (current_steering_angle < SERVO_MIN_DEGREE)
        current_steering_angle = SERVO_MIN_DEGREE;
    
    int new_angle = round(current_steering_angle);
    set_steering_servo_angle(new_angle);
}


static void
can_send_odometry_task(void *arg)
{
    double current_velocity;
    int16_t data_send;

	while (1)
	{
		voltage = get_voltage(voltage);
		data_send = (uint16_t) voltage;
        can_odometry_steering.data[0] = (uint8_t) (data_send & 0xFF);
        can_odometry_steering.data[1] = (uint8_t) ((data_send >> 8) & 0xFF);
		can_transmit(&can_odometry_steering, pdMS_TO_TICKS(1000));

		current_velocity = (odom_left_velocity + odom_right_velocity) / 2.0;
        data_send = (int16_t) round(current_velocity * VELOCITY_CONVERSION_CONSTANT);
        can_odometry_velocity.data[0] = (uint8_t) (data_send & 0xFF);
        can_odometry_velocity.data[1] = (uint8_t) ((data_send >> 8) & 0xFF);
		can_transmit(&can_odometry_velocity, pdMS_TO_TICKS(1000));

//		vTaskDelayUntil(&xLastWakeTime, xFrequencyTaskCan);

		vTaskDelay((1000 / 40) / portTICK_PERIOD_MS);
	}
}


static void
can_receive_task(void *arg)
{
	xSemaphoreTake(rx_sem, portMAX_DELAY);

	set_steering_effort(0); // init steering

	while (1)
	{
		can_message_t rx_msg;
		can_receive(&rx_msg, portMAX_DELAY);

		if (rx_msg.identifier == COMMAND_CAN_CAR_ID)
		{
//            command_velocity_received = (message.data[1] << 8) | message.data[0];
            short int steering_effort_received = (rx_msg.data[3] << 8) | rx_msg.data[2];
            set_steering_effort(steering_effort_received);
		}

		if (rx_msg.identifier == COMMAND_CAN_STEP_MOTOR_ID)
		{
//            int command_step_motor_received = (rx_msg.data[1] << 8) | rx_msg.data[0];
//            set_command_step_motor(command_step_motor_received);
		}
	}

	xSemaphoreGive(rx_sem);
	vTaskDelete(NULL);
}


void
configure_CAN_messages()                        						// flag types
{                                                           			// CAN_MSG_FLAG_EXTD			Message is in Extended Frame Format (29bit ID)
																		// CAN_MSG_FLAG_DLC_NON_COMP	Message’s Data length code is larger than 8. This will break compliance with CAN2.0B
	can_steering_and_velocity_efforts.identifier = COMMAND_CAN_CAR_ID;	// CAN_MSG_FLAG_RTR				Message is a Remote Transmit Request
	can_steering_and_velocity_efforts.flags = CAN_MSG_FLAG_SS;  		// CAN_MSG_FLAG_SS				Transmit message using Single Shot Transmission (Message will not be retransmitted upon error or loss of arbitration)
	can_steering_and_velocity_efforts.data_length_code = 4;     		// CAN_MSG_FLAG_SELF			Transmit message using Self Reception Request (Transmitted message will also received by the same node)

	can_odometry_velocity.identifier = ODOM_VELOCITY_CAN_ID;			// CAN_MSG_FLAG_RTR				Message is a Remote Transmit Request
	can_odometry_velocity.flags = CAN_MSG_FLAG_SS;   					// CAN_MSG_FLAG_SS				Transmit message using Single Shot Transmission (Message will not be retransmitted upon error or loss of arbitration)
	can_odometry_velocity.data_length_code = 2;      					// CAN_MSG_FLAG_SELF
    
    can_odometry_steering.identifier = ODOM_STEERING_CAN_ID;			// CAN_MSG_FLAG_RTR				Message is a Remote Transmit Request
	can_odometry_steering.flags = CAN_MSG_FLAG_SS;   					// CAN_MSG_FLAG_SS				Transmit message using Single Shot Transmission (Message will not be retransmitted upon error or loss of arbitration)
	can_odometry_steering.data_length_code = 2;      					// CAN_MSG_FLAG_SELF			Transmit message using Self Reception Request (Transmitted message will also received by the same node)
}


extern "C" void 
app_main(void)
{
	initArduino();

	configure_CAN_messages();

    adc_init();

    vTaskDelay(pdMS_TO_TICKS(1000));
//    step_motor_setup();

    init_steering_servo_control();

    // CAN bus setup
    rx_sem = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(can_receive_task, "can_receive_task", 				4096, NULL, CAN_RECEIVE_TASK_PRIORITY, NULL, 0);
//    xTaskCreatePinnedToCore(motor_task, "motor_task", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(can_send_odometry_task, "can_send_odometry_task", 	4096, NULL, CAN_SEND_ODOMETRY_TASK_PRIORITY, NULL, 0);
    xTaskCreatePinnedToCore(left_encoder_task, "left_encoder_task", 			4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(right_encoder_task, "right_encoder_task", 			4096, NULL, 5, NULL, 1);
//    xTaskCreatePinnedToCore(step_motor_task, "step_motor_task", 				4096, NULL, 5, NULL, 1);

    // Install and start CAN driver
    ESP_ERROR_CHECK(can_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");
    ESP_ERROR_CHECK(can_start());
    ESP_LOGI(EXAMPLE_TAG, "Driver started");

    xSemaphoreGive(rx_sem);                     //Start RX task


    while (1) 
    {
        // Teste do servo
        // for (uint32_t count = SERVO_MIN_DEGREE; count < SERVO_MAX_DEGREE; count++) 
        // {
        //     // printf("Angle of rotation: %d\n", count);
        //     uint32_t angle = servo_per_degree_init(count);
        //     // printf("pulse width: %dus\n", angle);
        //     mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
        //     vTaskDelay(pdMS_TO_TICKS(50));     // Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
        //     fflush(stdout);
        // }

        // for (uint32_t count = SERVO_MAX_DEGREE; count > SERVO_MIN_DEGREE; count--) 
        // {
        //     // printf("Angle of rotation: %d\n", count);
        //     uint32_t angle = servo_per_degree_init(count);
        //     // printf("pulse width: %dus\n", angle);
        //     mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
        //     vTaskDelay(pdMS_TO_TICKS(50));     // Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
        //     fflush(stdout);
        // }
        printf("Voltage: %d mV\n", voltage);
        fflush(stdout);

    	vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelay(pdMS_TO_TICKS(100));				// Wait 100ms
    xSemaphoreTake(rx_sem, portMAX_DELAY);      // Wait for RX task to complete

    //Stop and uninstall CAN driver
    ESP_ERROR_CHECK(can_stop());
    ESP_LOGI(EXAMPLE_TAG, "Driver stopped");
    ESP_ERROR_CHECK(can_driver_uninstall());
    ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

    //Cleanup
    vSemaphoreDelete(rx_sem);
    // end CAN bus setup
}
