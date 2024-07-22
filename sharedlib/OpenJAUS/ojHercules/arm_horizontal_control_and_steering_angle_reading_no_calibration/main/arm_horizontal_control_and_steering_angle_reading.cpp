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

void step_motor_setup();
void step_motor_task(void *arg);
void motor_task(void *arg);
void serial_task(void *arg);

// CAN bus
#define NO_OF_ITERS                     3
#define RX_TASK_PRIO                    9
#define TX_GPIO_NUM                     16
#define RX_GPIO_NUM                     4
#define EXAMPLE_TAG                     "CAN Listen Only"



//ADC Channels
#define ADC_ANGLE_POTENTIOMETER          ADC1_CHANNEL_6

//ADC Attenuation
#define ADC_EXAMPLE_ATTEN           ADC_ATTEN_DB_11

//ADC Calibration
#if CONFIG_IDF_TARGET_ESP32
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32C3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH    1100 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH    1900 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE        180 //Maximum angle in degree upto which servo can rotate
#define SERVO_MIN_DEGREE        0 //Minimum angle in degree upto which servo can rotate
#define SERVO_PULSE_GPIO        (18)   // GPIO connects to the PWM signal line

static esp_adc_cal_characteristics_t adc1_chars;

static const char *TAG = "Stepper Motor, angle measurement and steering servo control";

double current_steering_angle = (double) (SERVO_MIN_DEGREE + (SERVO_MAX_DEGREE + SERVO_MIN_DEGREE) / 2.0);

// begin CAN bus setup

static const can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
static const can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
//Set TX queue length to 0 due to listen only mode
static const can_general_config_t g_config = {.mode = CAN_MODE_NORMAL,
                                              .tx_io = (gpio_num_t) TX_GPIO_NUM, .rx_io = (gpio_num_t) RX_GPIO_NUM,
                                              .clkout_io = CAN_IO_UNUSED, .bus_off_io = CAN_IO_UNUSED,
                                              .tx_queue_len = 5, .rx_queue_len = 5,
                                              .alerts_enabled = CAN_ALERT_NONE, .clkout_divider = 0};
static SemaphoreHandle_t rx_sem;

can_message_t can_steering_and_velocity_efforts;
can_message_t can_odometry_velocity;
can_message_t can_odometry_steering;

// end CAN bus setup

static bool adc_calibration_init(void)
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

    return cali_enable;
}


static void 
adc1_task(void *arg)
{
    uint32_t voltage = 0;
    while (1) 
    {
        for (int i = 0; i < 200; i++)
        {
            uint32_t adc_reading = adc1_get_raw(ADC_ANGLE_POTENTIOMETER);
            voltage += esp_adc_cal_raw_to_voltage(adc_reading, &adc1_chars);
            vTaskDelay(0);
        }
        voltage /= 200;
        printf("Voltage: %d mV\n", voltage);
        fflush(stdout);
        vTaskDelay((1000 / 40) / portTICK_PERIOD_MS);
        voltage = 0;
    }
}

/**
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
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
set_steering_effort(int steering_effort)
{
    current_steering_angle += (double) steering_effort / 10.0;
    if (current_steering_angle > SERVO_MAX_DEGREE)
        current_steering_angle = SERVO_MAX_DEGREE;
    else if (current_steering_angle < SERVO_MIN_DEGREE)
        current_steering_angle = SERVO_MIN_DEGREE;
    
    int new_angle = round(current_steering_angle);
    set_steering_servo_angle(new_angle);
}

void
set_velocity_command(int velocity_command)
{
    ESP_LOGI(TAG, "Velocity command: %d", velocity_command);
}


static void
can_receive_task(void *arg)
{
	xSemaphoreTake(rx_sem, portMAX_DELAY);

	// float *break_effort = (float *) arg;
	while (1)
	{
		can_message_t rx_msg;
		can_receive(&rx_msg, portMAX_DELAY);

		// if (rx_msg.identifier == CAN_ID_SAFE_STOP_AND_MANUAL_OVERRIDE)
		// {
		// 	g_manual_override_and_safe_stop = rx_msg.data[0];
		// 	// Manual Override eh o bit 1 de g_manual_override_and_safe_stop
		// 	if (g_manual_override_and_safe_stop & 0x2)
		// 		*break_effort = 0.0;

		// 	// Safe Stop eh o bit 0 de g_manual_override_and_safe_stop
		// 	if (g_manual_override_and_safe_stop & 0x1)
		// 		*break_effort = 100.0;
		// }

		// if ((g_manual_override_and_safe_stop == 0) && (rx_msg.identifier == CAN_ID_SET_BREAK_EFFORT))
		// {
		// 	float byte_effort = (float) rx_msg.data[1] / 2.0; // transforma para um valor de 0.0 a 100.0. Ver pd.c em ojIARASim
		// 	*break_effort = (float) byte_effort;

		// 	static float previous_break_effort = 0.0;
		// 	if (*break_effort != previous_break_effort)
		// 		previous_break_effort = *break_effort;
		// }
	}

	xSemaphoreGive(rx_sem);
	vTaskDelete(NULL);
}

void
configure_CAN_messages()                        // flag types
{                                                           // CAN_MSG_FLAG_EXTD			Message is in Extended Frame Format (29bit ID)
															// CAN_MSG_FLAG_DLC_NON_COMP	Message’s Data length code is larger than 8. This will break compliance with CAN2.0B
	can_steering_and_velocity_efforts.identifier = 0x100;       // CAN_MSG_FLAG_RTR				Message is a Remote Transmit Request
	can_steering_and_velocity_efforts.flags = CAN_MSG_FLAG_SS;  // CAN_MSG_FLAG_SS				Transmit message using Single Shot Transmission (Message will not be retransmitted upon error or loss of arbitration)
	can_steering_and_velocity_efforts.data_length_code = 4;     // CAN_MSG_FLAG_SELF			Transmit message using Self Reception Request (Transmitted message will also received by the same node)

	can_odometry_velocity.identifier = 0x425;					// CAN_MSG_FLAG_RTR				Message is a Remote Transmit Request
	can_odometry_velocity.flags = CAN_MSG_FLAG_SS;   			// CAN_MSG_FLAG_SS				Transmit message using Single Shot Transmission (Message will not be retransmitted upon error or loss of arbitration)
	can_odometry_velocity.data_length_code = 2;      			// CAN_MSG_FLAG_SELF
    
    can_odometry_steering.identifier = 0x80;					// CAN_MSG_FLAG_RTR				Message is a Remote Transmit Request
	can_odometry_steering.flags = CAN_MSG_FLAG_SS;   			// CAN_MSG_FLAG_SS				Transmit message using Single Shot Transmission (Message will not be retransmitted upon error or loss of arbitration)
	can_odometry_steering.data_length_code = 2;      			// CAN_MSG_FLAG_SELF			Transmit message using Self Reception Request (Transmitted message will also received by the same node)
}

extern "C" void 
app_main(void)
{
    configure_CAN_messages();
	initArduino();

    adc_calibration_init();

    vTaskDelay(pdMS_TO_TICKS(1000));
    step_motor_setup();

    init_steering_servo_control();

    // Angle potentiometer ADC config
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_ANGLE_POTENTIOMETER, ADC_EXAMPLE_ATTEN));


    xTaskCreatePinnedToCore(adc1_task, "adc1_task", 1024 * 2, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(step_motor_task, "step_motor_task", 1024 * 2, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(can_receive_task, "CAN_rx", 4096, NULL, RX_TASK_PRIO, NULL, 0);
    xTaskCreatePinnedToCore(motor_task, "motor_task", 1024 * 2, NULL, 2, NULL, 0);
    // xTaskCreatePinnedToCore(can_send_odometry_task, "CAN_rx", 4096, NULL, RX_TASK_PRIO, NULL, 0);


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

       vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
