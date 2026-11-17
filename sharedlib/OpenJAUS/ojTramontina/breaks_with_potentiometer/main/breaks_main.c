#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"
#include "esp_bit_defs.h"
#include "driver/timer.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "esp_timer.h"

#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/can.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"


// Breaks
#define GPIO_PWM0A_OUT 			2		// Set GPIO 2 as PWM0A
#define GPIO_PWM0B_OUT 			16		// Set GPIO 16 as PWM0B
#define	HALL2_EXTENDING			BIT18	// GPIO 18
#define	HALL1_TRANSITION		BIT19 	// GPIO 19

#define MAX_VOLTAGE_SET		(2960)
#define MIN_VOLTAGE_SET		(1300)

#define BREAKS_PWM_FREQUENCY	50
#define BREAKS_PWM_RANGE		100.0

#define BREAKS_PID_Kp 			1.0
#define BREAKS_PID_Ki 			0.5
#define BREAKS_PID_Kd 			0.05
#define BREAKS_MIN_ut			15.0
#define BREAKS_MAX_ut			100.0

#define	PUSHBREAKS			 	4  	// GPIO  4, pino  7
#define	PULLBREAKS				27 	// GPIO 27, pino 13
// end Breaks

// CAN bus
#define NO_OF_ITERS                     3
#define RX_TASK_PRIO                    9
#define TX_GPIO_NUM                     21
#define RX_GPIO_NUM                     22
#define EXAMPLE_TAG                     "CAN Listen Only"

#define CAN_ID_SET_BREAK_EFFORT         		0x480	// Ver pd.c em ojIARASim
#define CAN_ID_SAFE_STOP_AND_MANUAL_OVERRIDE	0x600

static const can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
static const can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
//Set TX queue length to 0 due to listen only mode
static const can_general_config_t g_config = {.mode = CAN_MODE_NORMAL,
                                              .tx_io = TX_GPIO_NUM, .rx_io = RX_GPIO_NUM,
                                              .clkout_io = CAN_IO_UNUSED, .bus_off_io = CAN_IO_UNUSED,
                                              .tx_queue_len = 0, .rx_queue_len = 5,
                                              .alerts_enabled = CAN_ALERT_NONE,
                                              .clkout_divider = 0};
static SemaphoreHandle_t rx_sem;
// end CAN bus


#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   256          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2 -> o Potenciomentro vai no GPIO34
static const adc_atten_t atten = ADC_ATTEN_DB_11; // max = 3.9V -> https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/adc.html
static const adc_unit_t unit = ADC_UNIT_1;

static int g_manual_override_and_safe_stop = 0;


static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

static void
mcpwm_gpio_initialize(void)
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
}


static void
brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}


static void
brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
}


static void
brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}


double
ojGetTimeSec()
{
	int64_t t = esp_timer_get_time();
	double t_seconds = (double) t / 1000000.0;

	return (t_seconds);
}


int
gpioRead(int gpio_port)
{
	int status = REG_READ(GPIO_IN_REG) & gpio_port;

	return (status);
}


int g_voltage = 0;
int g_previous_hall_transition;
int g_previous_hall_extending;

int
hall_transition(int gpio_port)
{
	int current = gpioRead(gpio_port);

	if (gpio_port == HALL1_TRANSITION)
	{
		if (current != g_previous_hall_transition)
		{
			g_previous_hall_transition = current;
			if (current)
				return (1);	// rising edge
			else
				return (2);	// falling edge
		}
	}
	else
	{
		if (current != g_previous_hall_extending)
		{
			g_previous_hall_extending = current;
			if (current)
				return (1);	// rising edge
			else
				return (2);	// falling edge
		}
	}
	return (0);
}


void
update_voltage()
{
	int hall_transition_transition = hall_transition(HALL1_TRANSITION);
	int hall_extending_transition = hall_transition(HALL2_EXTENDING);

	if (hall_transition_transition == 1) // changed to high (a rising edge)
	{
		if (gpioRead(HALL2_EXTENDING))
			g_voltage++;
		else
			g_voltage--;
	}
	else if (hall_transition_transition == 2) // changed to low (a falling edge)
	{
		if (gpioRead(HALL2_EXTENDING))
			g_voltage--;
		else
			g_voltage++;
	}
	else if (hall_extending_transition == 2) // changed to low (a falling edge)
	{
		if (gpioRead(HALL1_TRANSITION))
			g_voltage++;
		else
			g_voltage--;
	}
	else if (hall_extending_transition == 1)
	{
		if (gpioRead(HALL1_TRANSITION))
			g_voltage--;
		else
			g_voltage++;
	}
}


void
ojSleepMsec(double miliseconds)
{
	double t = ojGetTimeSec();
    while ((ojGetTimeSec() - t) < (miliseconds / 1000.0))
		update_voltage();
}


void
gpioPWM(int push_pull, double pwm)
{
	if (push_pull == PUSHBREAKS)
		brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, pwm);
	else
		brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, pwm);
}


void
apply_break_effort(double ut)
{
	static int push = 0;

	if (ut > BREAKS_MIN_ut)
	{
		if (!push)
		{
			gpioPWM(PULLBREAKS, 0);
			ojSleepMsec(1);
			push = 1;
		}
		double pwm = ut * (double) BREAKS_PWM_RANGE / BREAKS_MAX_ut;
		if (pwm > BREAKS_PWM_RANGE)
			pwm = BREAKS_PWM_RANGE;
		gpioPWM(PUSHBREAKS, pwm);
	}
	else if (ut < -BREAKS_MIN_ut)
	{
		if (push)
		{
			gpioPWM(PUSHBREAKS, 0);
			ojSleepMsec(1);
			push = 0;
		}
		double pwm = -ut * (double) BREAKS_PWM_RANGE / BREAKS_MAX_ut;
		if (pwm > BREAKS_PWM_RANGE)
			pwm = BREAKS_PWM_RANGE;
		gpioPWM(PULLBREAKS, pwm);
	}
	else
		brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
}


double
saturate(double X, double Y, double Z)
{
	if (Y < X)
		return (X);
	else if (Y > Z)
		return (Z);
	return (Y);
}


double
breaks_pid(double desired_voltage, double current_voltage, int manual_override)
{
	// http://en.wikipedia.org/wiki/PID_controller -> Discrete implementation
	static double 	error_t_1 = 0.0;	// error in time t-1
	static double 	integral_t = 0.0;
	static double 	integral_t_1 = 0.0;
	static double 	u_t = 0.0;			// u(t)	-> actuation in time t
	static double	previous_t = 0.0;

	if (previous_t == 0.0)
	{
		previous_t = ojGetTimeSec();
		return (0.0);
	}
	double t = ojGetTimeSec();
	double delta_t = t - previous_t;

//	if (delta_t < (0.7 * (1.0 / 40.0)))
//		return (u_t);

	double error_t = desired_voltage - current_voltage;

	if (manual_override == 0)
		integral_t = integral_t + error_t * delta_t;
	else
		integral_t = integral_t_1 = 0.0;

	double derivative_t = (error_t - error_t_1) / delta_t;

	u_t = BREAKS_PID_Kp * error_t +
		  BREAKS_PID_Ki * integral_t +
		  BREAKS_PID_Kd * derivative_t;

	error_t_1 = error_t;

	// Anti windup
	if ((u_t < -BREAKS_MAX_ut) || (u_t > BREAKS_MAX_ut))
		integral_t = integral_t_1;
	integral_t_1 = integral_t;

	previous_t = t;

	u_t = saturate(-BREAKS_MAX_ut, u_t, BREAKS_MAX_ut);

	return (u_t);
}


double
get_voltage_from_break_effort(double break_effort)
{
	double voltage = MIN_VOLTAGE_SET + (break_effort * (double) (MAX_VOLTAGE_SET - MIN_VOLTAGE_SET) / 100.0); // 100.0 eh o maximo de break_effort

	return (voltage);
}


double
get_breaks_voltage()
{
    uint32_t adc_reading = 0;
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
        if (unit == ADC_UNIT_1) // Usamos a ADC_UNIT_1
        {
            adc_reading += adc1_get_raw((adc1_channel_t) channel);
        }
        else
        {
            int raw;
            adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES;
    //Convert adc_reading to voltage in mV
    int voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

    return (voltage);
}


void
update_breaks(double g_break_effort)
{
	int desired_voltage = get_voltage_from_break_effort(g_break_effort);
    g_voltage = get_breaks_voltage();
	double ut = breaks_pid(desired_voltage, g_voltage, g_manual_override_and_safe_stop);
	apply_break_effort(ut);
}


void
init_breaks()
{
	// Init breaks PWM
	mcpwm_gpio_initialize();

	// Disable GPIO_NUM_18 para desabilitar entrada da versao com HALL sensor
	gpio_set_direction(GPIO_NUM_18, GPIO_MODE_INPUT);
	gpio_set_pull_mode(GPIO_NUM_18, GPIO_FLOATING);

    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

	// Init breaks pontentiometer sensor (https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/adc.html)
    if (unit == ADC_UNIT_1) // Usamos a ADC_UNIT_1
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    }
    else
        adc2_config_channel_atten((adc2_channel_t) channel, atten);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    //Continuously sample ADC1
//    while (1)
//    {
//    	int voltage = get_breaks_voltage();
//        printf("Voltage: %dmV\n", voltage);
//        vTaskDelay(pdMS_TO_TICKS(100));
//    }

    printf("Configuring Initial Parameters of mcpwm...\n");
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = BREAKS_PWM_FREQUENCY;
    pwm_config.cmpr_a = 0;    	// duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    	// duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    // Configure PWM0A & PWM0B with above settings

    double t;
    g_voltage = get_breaks_voltage();
	printf("g_voltage %d\n\r", g_voltage);
	int previous_g_voltage;
	// Volta ao minimo do curso do atuador linear
	brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 80.0);
	do
	{
		previous_g_voltage = g_voltage;
		ojSleepMsec(500.0);
	    g_voltage = get_breaks_voltage();
	} while (previous_g_voltage != g_voltage);
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
	printf("g_voltage %d\n\r", g_voltage);

	// Vai para o minimo desejado de curso (MIN_VOLTAGE_SET)
	double initial_time = ojGetTimeSec();
	t = initial_time;
	while ((t - initial_time) < 3.0)
	{
		ojSleepMsec(5.0);
	    g_voltage = get_breaks_voltage();
		double ut = breaks_pid(MIN_VOLTAGE_SET, g_voltage, 0);
		apply_break_effort(ut);
		t = ojGetTimeSec();
	}
	printf("\ng_voltage %d\n\r", g_voltage);
}


static void
set_break_effort(void *desired_break_effort)
{
	double break_effort;

	while (1)
	{
		break_effort = (double) (*((float *) desired_break_effort));

		update_breaks(break_effort);
		ojSleepMsec(5.0);
	}
}


static void
can_receive_task(void *arg)
{
	xSemaphoreTake(rx_sem, portMAX_DELAY);

	float *break_effort = (float *) arg;
	while (1)
	{
		can_message_t rx_msg;
		can_receive(&rx_msg, portMAX_DELAY);

		if (rx_msg.identifier == CAN_ID_SAFE_STOP_AND_MANUAL_OVERRIDE)
		{
			g_manual_override_and_safe_stop = rx_msg.data[0];
			// Manual Override eh o bit 1 de g_manual_override_and_safe_stop
			if (g_manual_override_and_safe_stop & 0x2)
				*break_effort = 0.0;

			// Safe Stop eh o bit 0 de g_manual_override_and_safe_stop
			if (g_manual_override_and_safe_stop & 0x1)
				*break_effort = 100.0;
		}

		if ((g_manual_override_and_safe_stop == 0) && (rx_msg.identifier == CAN_ID_SET_BREAK_EFFORT))
		{
			float byte_effort = (float) rx_msg.data[1] / 2.0; // transforma para um valor de 0.0 a 100.0. Ver pd.c em ojIARASim
			*break_effort = (float) byte_effort;

			static float previous_break_effort = 0.0;
			if (*break_effort != previous_break_effort)
				previous_break_effort = *break_effort;
		}
	}

	xSemaphoreGive(rx_sem);
	vTaskDelete(NULL);
}


void
app_main(void)
{
	init_breaks();

    float break_effort = 0.0;
    xTaskCreatePinnedToCore(set_break_effort, "set_break_effort", 2048, &break_effort, 5, NULL, 1);

    // CAN bus setup
    rx_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(can_receive_task, "CAN_rx", 4096, &break_effort, RX_TASK_PRIO, NULL, 0);

    //Install and start CAN driver
    ESP_ERROR_CHECK(can_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");
    ESP_ERROR_CHECK(can_start());
    ESP_LOGI(EXAMPLE_TAG, "Driver started");

    xSemaphoreGive(rx_sem);                     //Start RX task

	printf("starting break_effort = %f\n", break_effort);
	vTaskDelay(2000 / portTICK_RATE_MS);
    int cnt = 0;
    while (1)
    {
		cnt++;

		vTaskDelay(100 / portTICK_RATE_MS);
		printf("break_effort = %f, g_voltage = %d\n", break_effort, g_voltage);
    }

    vTaskDelay(pdMS_TO_TICKS(100));				//Wait 100ms
    xSemaphoreTake(rx_sem, portMAX_DELAY);      //Wait for RX task to complete

    //Stop and uninstall CAN driver
    ESP_ERROR_CHECK(can_stop());
    ESP_LOGI(EXAMPLE_TAG, "Driver stopped");
    ESP_ERROR_CHECK(can_driver_uninstall());
    ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

    //Cleanup
    vSemaphoreDelete(rx_sem);
    // end CAN bus setup
}
