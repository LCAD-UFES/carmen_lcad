#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <sys/time.h>

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
#include "esp_sleep.h"

#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/can.h"

#include "driver/timer.h"

#include "driver/dac.h"
#include "driver/adc.h"

#include "esp_adc_cal.h"

#include "driver/periph_ctrl.h"

#include <rom/ets_sys.h>

#include "esp_wifi.h"

#define ESP_INTR_FLAG_DEFAULT 0

#define LOW  0
#define HIGH 1

#define DEBOUNCE_FACTOR 1000

// Writable pins: 2, 4, 16, 17, 5, 18, 19, 23, 27, 26, 25, 33, 32
// Readable pins: 15, 13, 12, ...

// CAN bus
#define NO_OF_ITERS                     3
#define RX_TASK_PRIO                    9
#define TX_GPIO_NUM                     21
#define RX_GPIO_NUM                     22
#define EXAMPLE_TAG                     "CAN"

#define THROTTLE_AND_BREAK_EFFORT_CAN_ID 0x480	// Ver pd.c em ojIARASim
#define GEAR_CAN_ID                      0x405
#define SIGNALS_CAN_ID					 0x400

// GPIO PINS DEFINITIONS
#define YELLOW_BUTTON_PIN  			GPIO_NUM_13

#define GEAR_RELAY_UP				GPIO_NUM_5
#define GEAR_RELAY_DOWN  			GPIO_NUM_18

#define MANUAL_OVERRIDE_RELAY		GPIO_NUM_23

#define DIGITAL_IO_SERIAL_OUT		GPIO_NUM_2
#define DIGITAL_IO_CLOCK			GPIO_NUM_32
#define DIGITAL_IO_SERIAL_LATCH		GPIO_NUM_33
#define DIGITAL_IO_SERIAL_IN		GPIO_NUM_39

// Throttle
#define	RELAY_THROTTLE				GPIO_NUM_4

// Brakes
#define MAX_VOLTAGE_SET				(2380)
#define MIN_VOLTAGE_SET				(1200)

#define IBT_2_SIGNAL1				GPIO_NUM_16
#define IBT_2_SIGNAL2				GPIO_NUM_17
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   256          //Multisampling

#define	PUSHBRAKES			 	0
#define	PULLBRAKES				1

#define BRAKES_PWM_FREQUENCY	50
#define BRAKES_PWM_RANGE		100.0

#define BRAKES_PID_Kp 			1.0
#define BRAKES_PID_Ki 			0.5
#define BRAKES_PID_Kd 			0.05
#define BRAKES_MIN_ut			15.0
#define BRAKES_MAX_ut			100.0

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC2_CHANNEL_3;     // GPIO15 ADC2 -> o Potenciomentro vai no GPIO15
static const adc_atten_t atten = ADC_ATTEN_DB_11; // max = 3.9V -> https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/adc.html
static const adc_unit_t unit = ADC_UNIT_2;

int g_voltage = 0; // in mV
// End Brakes

// begin CAN bus setup
static const can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();
static const can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
//Set TX queue length to 0 due to listen only mode
static const can_general_config_t g_config = {.mode = CAN_MODE_NORMAL,
                                              .tx_io = TX_GPIO_NUM, .rx_io = RX_GPIO_NUM,
                                              .clkout_io = CAN_IO_UNUSED, .bus_off_io = CAN_IO_UNUSED,
                                              .tx_queue_len = 5, .rx_queue_len = 5,
                                              .alerts_enabled = CAN_ALERT_NONE, .clkout_divider = 0};

//static const can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, CAN_MODE_NORMAL);

static SemaphoreHandle_t rx_sem;
static SemaphoreHandle_t digital_io_sem;

can_message_t manual_override_and_safe_stop;
can_message_t digital_io_input;
can_message_t tramontina_speed_can_message;

// end CAN bus setup

int safe_stop_button = LOW;
int manual_override_button = LOW;

int autonomous = 0;

uint64_t zero_dot_two_seconds_counter = 0;

enum
{
	NEUTRAL,
	GOING_TO_DRIVE,
	DRIVE,
	GOING_TO_REVERSE,
	REVERSE,
} GEAR_SM_STATE;

int gear_sm_state = NEUTRAL;
uint64_t previous_gear_sm_time = 0;

enum
{
	MANUAL_OVERRIDE,
	GOING_TO_ALLOW_AUTONOMY,
	GOING_TO_ALLOW_AUTONOMY2,
	ALLOW_AUTONOMY
} MANUAL_OVERRIDE_SM_STATE;

int manual_override_state = MANUAL_OVERRIDE;
uint64_t previous_manual_override_sm_time = 0;

int resquested_bits_to_send = 0;

// speed Tramontina
#define SPEED_SIGNAL_A				GPIO_NUM_34
#define SPEED_SIGNAL_B				GPIO_NUM_35

#define	NUM_PULSE_COUNTERS			5
volatile int signalB = 0; // Current value of signal B (0 or 1)
volatile int pulseCount = 10000;
// volatile int pulseCounter[NUM_PULSE_COUNTERS];
// volatile unsigned int pulseCounterIndex = 0;
static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;
int int_speed;


void
examine_can_errors_and_try_to_restart()
{
	uint32_t alerts;
	can_read_alerts(&alerts, pdMS_TO_TICKS(1000));

	if (alerts & CAN_ALERT_ABOVE_ERR_WARN)
		printf("Surpassed Error Warning Limit\n");

	if (alerts & CAN_ALERT_ERR_PASS)
		printf("Entered Error Passive state\n");

	if (alerts & CAN_ALERT_TX_FAILED)
		printf("Tramission failed\n");

	if (alerts & CAN_ALERT_RX_QUEUE_FULL)
		printf("Receive queue full\n");

	if (alerts & CAN_ALERT_ARB_LOST)
		printf("A transmission lost arbitration\n");

	if (alerts & CAN_ALERT_BUS_RECOVERED)
		printf("Bus Recovered!\n");

	if (alerts & CAN_ALERT_BUS_ERROR)
		printf("A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus!\n");

    can_status_info_t status_info;
    esp_err_t error = can_get_status_info(&status_info);
    if (error != ESP_OK)
		printf("*** can_get_status_info() error %s\n", esp_err_to_name(error));
    else
    {
    	printf("\nstatus_info.state %d\n", status_info.state);
    	printf("status_info.msgs_to_tx %d\n", status_info.msgs_to_tx);
    	printf("status_info.msgs_to_rx %d\n", status_info.msgs_to_rx);
    	printf("status_info.tx_error_counter %d\n", status_info.tx_error_counter);
    	printf("status_info.rx_error_counter %d\n", status_info.rx_error_counter);
    	printf("status_info.tx_failed_count %d\n", status_info.tx_failed_count);
    	printf("status_info.rx_missed_count %d\n", status_info.rx_missed_count);
    	printf("status_info.arb_lost_count %d\n", status_info.arb_lost_count);
    	printf("status_info.bus_error_count %d\n", status_info.bus_error_count);
    }

	printf("Initiate bus recovery\n");
	can_driver_uninstall();
	ESP_ERROR_CHECK(can_driver_install(&g_config, &t_config, &f_config));
	can_start();
	printf("bus recovery complete\n\n");
}


void
set_throttle_pedal_effort(int int_throttle_effort)
{
	if ((gear_sm_state != DRIVE) && (gear_sm_state != REVERSE)) // O Tramontina da erro 41 se acelerar sem estar com a marcha DRIVE ou REVERSE
		int_throttle_effort = 0;

	dac_output_voltage(DAC_CHANNEL_1, int_throttle_effort);			// o hardware converte de 0V to 3.3V para 0V to 5V
	if (int_throttle_effort == 0)
		gpio_set_level(RELAY_THROTTLE, HIGH);	// Active low
	else
		gpio_set_level(RELAY_THROTTLE, LOW);	// Active low
}


void
initialize_manual_override_throttle_and_gear()
{
	gpio_set_direction(YELLOW_BUTTON_PIN, GPIO_MODE_INPUT);
	gpio_set_pull_mode(YELLOW_BUTTON_PIN, GPIO_PULLUP_ONLY);

	gpio_set_direction(MANUAL_OVERRIDE_RELAY, GPIO_MODE_OUTPUT);
	gpio_set_level(MANUAL_OVERRIDE_RELAY, HIGH);

	// Gear Relays
	gpio_set_direction(GEAR_RELAY_UP, GPIO_MODE_OUTPUT);
	gpio_set_direction(GEAR_RELAY_DOWN, GPIO_MODE_OUTPUT);

	gpio_set_level(GEAR_RELAY_UP, HIGH);
	gpio_set_level(GEAR_RELAY_DOWN, HIGH);

	// Tramontina speed
	gpio_set_direction(SPEED_SIGNAL_A, GPIO_MODE_INPUT);
    gpio_set_intr_type(SPEED_SIGNAL_A, GPIO_INTR_POSEDGE);
	gpio_set_direction(SPEED_SIGNAL_B, GPIO_MODE_INPUT);

	set_throttle_pedal_effort(0);

	printf("Throttle and Gear Driver Initialized!\n");
}


void
send_bit_serially(int bit)
{
	if (bit)
		gpio_set_level(DIGITAL_IO_SERIAL_OUT, LOW); // active low!
	else
		gpio_set_level(DIGITAL_IO_SERIAL_OUT, HIGH);
	usleep(1000);

	gpio_set_level(DIGITAL_IO_CLOCK, HIGH);
	usleep(1000);
	gpio_set_level(DIGITAL_IO_CLOCK, LOW);
}


void
send_bits_serially(int bits, int size)
{
    xSemaphoreTake(digital_io_sem, portMAX_DELAY);      // Wait for RX task to complete

    usleep(1000);
	for (int i = size - 1; i >= 0; i--)
	{
		int bit = 0;
		if (bits & (1 << i))
			bit = 1;
		send_bit_serially(bit);
	}
	gpio_set_level(DIGITAL_IO_SERIAL_LATCH, HIGH);
	usleep(1000);
	gpio_set_level(DIGITAL_IO_SERIAL_LATCH, LOW);
	usleep(1000);

    xSemaphoreGive(digital_io_sem);
}


int
get_bit_serially()
{
	int bit = gpio_get_level(DIGITAL_IO_SERIAL_IN);
	usleep(1000);

	gpio_set_level(DIGITAL_IO_CLOCK, HIGH);
	usleep(1000);
	gpio_set_level(DIGITAL_IO_CLOCK, LOW);

	return (bit);
}


int
get_bits_serially(int size)
{
    xSemaphoreTake(digital_io_sem, portMAX_DELAY);      // Wait for RX task to complete

	usleep(1000);
	gpio_set_level(DIGITAL_IO_CLOCK, LOW);
	gpio_set_level(DIGITAL_IO_SERIAL_OUT, LOW);
	usleep(1000);
	gpio_set_level(DIGITAL_IO_CLOCK, HIGH);
	usleep(1000);
	gpio_set_level(DIGITAL_IO_CLOCK, LOW);
	usleep(1000);
	gpio_set_level(DIGITAL_IO_SERIAL_OUT, HIGH);
	usleep(1000);

	int bits = 0;
	for (int i = size - 1; i >= 0; i--)
	{
		int bit = get_bit_serially();
		bits |= bit << i;
	}

    xSemaphoreGive(digital_io_sem);

	return (bits);
}


void
initialize_digital_io()
{
	gpio_set_direction(DIGITAL_IO_SERIAL_OUT, GPIO_MODE_OUTPUT);
	gpio_set_direction(DIGITAL_IO_CLOCK, GPIO_MODE_OUTPUT);
	gpio_set_direction(DIGITAL_IO_SERIAL_LATCH, GPIO_MODE_OUTPUT);
	gpio_set_direction(DIGITAL_IO_SERIAL_IN, GPIO_MODE_INPUT);

    xSemaphoreTake(digital_io_sem, portMAX_DELAY);      // Wait for RX task to complete
	resquested_bits_to_send = 0;
    xSemaphoreGive(digital_io_sem);

	printf("IO Driver Initialized!\n");
}


void
set_safe_stop_mode()
{
	autonomous = 0;
	manual_override_state = MANUAL_OVERRIDE;

	gpio_set_level(MANUAL_OVERRIDE_RELAY, HIGH);

	gpio_set_level(GEAR_RELAY_UP, HIGH);
	gpio_set_level(GEAR_RELAY_DOWN, HIGH);

	gpio_set_level(RELAY_THROTTLE, HIGH);	// Active low

	xSemaphoreTake(digital_io_sem, portMAX_DELAY);      // Wait for RX task to complete
	resquested_bits_to_send = 0x3 | 0x4 | 0x8; // setas, parking lights e high beam
    xSemaphoreGive(digital_io_sem);
}


void
set_manual_override_mode()
{
	autonomous = 0; // global

	gpio_set_level(MANUAL_OVERRIDE_RELAY, HIGH);

	gpio_set_level(GEAR_RELAY_UP, HIGH);
	gpio_set_level(GEAR_RELAY_DOWN, HIGH);

	gpio_set_level(RELAY_THROTTLE, HIGH);	// Active low

    xSemaphoreTake(digital_io_sem, portMAX_DELAY);      // Wait for RX task to complete
	resquested_bits_to_send = 0;
    xSemaphoreGive(digital_io_sem);
}


void
set_autonomous_mode()
{
	autonomous = 1;

	gpio_set_level(MANUAL_OVERRIDE_RELAY, LOW);
}


void
gear_state_machine(int request)
{
	switch (gear_sm_state)
	{
	case NEUTRAL:
		// Action
		gpio_set_level(GEAR_RELAY_DOWN, HIGH);	// aciona em LOW
		gpio_set_level(GEAR_RELAY_UP, HIGH);	// aciona em LOW
		if ((zero_dot_two_seconds_counter - previous_gear_sm_time) > 1)
		{
			// State change
			if (request != NEUTRAL)
			{
				previous_gear_sm_time = zero_dot_two_seconds_counter;
				if (request == DRIVE)
					gear_sm_state = GOING_TO_DRIVE;
				else if (request == REVERSE)
					gear_sm_state = GOING_TO_REVERSE;
			}
		}
		break;
	case GOING_TO_DRIVE:
		// Action
		gpio_set_level(GEAR_RELAY_UP, LOW);	// aciona em LOW
		gpio_set_level(GEAR_RELAY_DOWN, HIGH);	// aciona em LOW
		// State change
		if ((zero_dot_two_seconds_counter - previous_gear_sm_time) > 1)
		{
			previous_gear_sm_time = zero_dot_two_seconds_counter;
			gear_sm_state = DRIVE;
		}
		else if (request != DRIVE)
		{
			previous_gear_sm_time = zero_dot_two_seconds_counter;
			gear_sm_state = NEUTRAL;
		}
		break;
	case DRIVE:
		// Action
		gpio_set_level(GEAR_RELAY_UP, LOW);	// aciona em LOW
		gpio_set_level(GEAR_RELAY_DOWN, HIGH);	// aciona em LOW
		// State change
		if ((request == NEUTRAL) || (request == REVERSE))
		{
			previous_gear_sm_time = zero_dot_two_seconds_counter;
			gear_sm_state = NEUTRAL;
		}
		break;
	case GOING_TO_REVERSE:
		// Action
		gpio_set_level(GEAR_RELAY_UP, HIGH);	// aciona em LOW
		gpio_set_level(GEAR_RELAY_DOWN, LOW);	// aciona em LOW
		// State change
		if ((zero_dot_two_seconds_counter - previous_gear_sm_time) > 1)
		{
			previous_gear_sm_time = zero_dot_two_seconds_counter;
			gear_sm_state = REVERSE;
		}
		else if (request != REVERSE)
		{
			previous_gear_sm_time = zero_dot_two_seconds_counter;
			gear_sm_state = NEUTRAL;
		}
		break;
	case REVERSE:
		// Action
		gpio_set_level(GEAR_RELAY_UP, HIGH);	// aciona em LOW
		gpio_set_level(GEAR_RELAY_DOWN, LOW);	// aciona em LOW
		// State change
		if (request != REVERSE)
		{
			previous_gear_sm_time = zero_dot_two_seconds_counter;
			gear_sm_state = NEUTRAL;
		}
		break;
	}
}


void
can_message_handler(void *arg)
{
	xSemaphoreTake(rx_sem, portMAX_DELAY);

	float *break_effort = (float *) arg;
	while (1)
	{
		can_message_t rx_msg;
		esp_err_t can_state = can_receive(&rx_msg, pdMS_TO_TICKS(1000));
		if (can_state != ESP_OK)
		{
			printf("*** can_receive(&rx_msg, pdMS_TO_TICKS(1000)) error!\n");
			examine_can_errors_and_try_to_restart();
		}

		if (rx_msg.identifier == THROTTLE_AND_BREAK_EFFORT_CAN_ID)
		{
			if (autonomous)
			{
				float throttle_effort = ((float) rx_msg.data[0] / 2.0) / 100.0;
				int int_throttle_effort = (int) ceil(throttle_effort * 255.0);
				set_throttle_pedal_effort(int_throttle_effort);	// throttle convertido para um valor entre 0 e 255

				*break_effort = (float) rx_msg.data[1] / 2.0; // transforma para um valor de 0.0 a 100.0. Ver pd.c em ojIARASim

				// printf("rx_msg.data[0] %d, throttle_effort %f, rx_msg.data[1] %d, break_effort %f\n", rx_msg.data[0], throttle_effort, rx_msg.data[0], *break_effort);
			}
			else
			{
				set_throttle_pedal_effort(0);
				*break_effort = 0.0;
			}
		}

		if (rx_msg.identifier == GEAR_CAN_ID)
		{
			if (autonomous)
			{
				if (rx_msg.data[0] == 0x03)  // Drive
					gear_state_machine(DRIVE);
				else if (rx_msg.data[0] == 0x01)  // Reverse
					gear_state_machine(REVERSE);
				else if (rx_msg.data[0] == 0x02)  // Neutral
					gear_state_machine(NEUTRAL);
				// printf("rx_msg.data[0] %d\n", rx_msg.data[0]);
			}
		}

		if (rx_msg.identifier == SIGNALS_CAN_ID)
		{
			// Bits 0-2: Lights: 0: Off, 1: Parking Lights, 2: On;
			// Bit 3: Lights: High beans (T: On, F: Off), only if Lights Status = 2;
			// Bit 4: Lights: Fog lights (T: On, F: Off), only if Lights Status = 2;
			// Bits 5-6: Turn Signals: 0: Off, 1: Left, 2: Right, 3: Flashes
			// Bit 8: Horn: 0: Off, 1: On
			// Bits 9-10: Windshield Wipers: 0: Off, 1: On Slow, 2: On Fast
			int signals_raw = rx_msg.data[0] | (rx_msg.data[1] << 8);

			int signals = (signals_raw >> 5) & 0x3; // Turn Signals
			if (signals_raw & 0x1) // Parking Lights
				signals |= 0x4;
			if ((signals_raw & 0x2) && (signals_raw & 0x8)) // High beans
				signals |= 0x8;
			signals |= ((signals_raw >> 8) & 0x1) << 4; // Horn
			signals |= ((signals_raw >> 9) & 0x3) << 5; // Windshield Wipers

			// printf("signals_raw = 0x%x, signals = 0x%x\n", signals_raw, signals);
			if (autonomous)
			{
			    xSemaphoreTake(digital_io_sem, portMAX_DELAY);      // Wait for RX task to complete
				resquested_bits_to_send = signals;
			    xSemaphoreGive(digital_io_sem);
			}
		}
	}

	xSemaphoreGive(rx_sem);
	vTaskDelete(NULL);
}


int
gpio_get_level_with_debounce(gpio_num_t pin, int pin_state) // Deals with debounce effect that occours in the button state transition
{
	int pin_cont = 0;

	for (int i = 0; i < 1000; i++)
	{
		if (pin_state != gpio_get_level(pin))
			pin_cont = pin_cont + 1;
	}

	if (pin_cont >= 800 && pin_state != gpio_get_level(pin))
	{
		pin_state = gpio_get_level(pin);
//		printf("DEBOUNCED!\n");
	}

	return (pin_state);
}


void
send_manual_override_and_safe_stop_CAN_message(int manual_override_yellow_button)
{
	manual_override_and_safe_stop.data[0] = 0;

	if (manual_override_yellow_button == HIGH)
		manual_override_and_safe_stop.data[0] = manual_override_and_safe_stop.data[0] | 2;  // Turn on the first bit of the data byte (00000001)

	if (safe_stop_button == HIGH)
		manual_override_and_safe_stop.data[0] = manual_override_and_safe_stop.data[0] | 1;  // Turn on the first bit of the data byte (00000010)

	esp_err_t error = can_transmit(&manual_override_and_safe_stop, pdMS_TO_TICKS(1000));
//	printf("t m\n");
	if (error != ESP_OK)
	{
		printf("*** can_transmit(&manual_override_and_safe_stop, pdMS_TO_TICKS(1000)) error!\n");
		examine_can_errors_and_try_to_restart();
	}
}


int
gpio_get_digital_io()
{
	// Bits:
	// 0 - Turn Left
	// 1 - Turn Right
	// 2 - Parking Lights (Farol Baixo)
	// 3 - High Beans (Farol Alto)
	// 4 - Horn
	// 5 - Windshield Wipers Slow
	// 6 - Windshield Wipers Fast
	// 7 - Unused
	// 8 - Front Right Door
	// 9 - Front Left Door
	// 10 - Back Right Door
	// 11 - Back Left Door
	int bits = get_bits_serially(16);
	// printf("io bits 0x%4x\n", bits);

	return (bits);
}


void
send_digital_io_in_CAN_message(int digital_io)
{
//	printf("digital_io = 0x%04x\n", digital_io);

	//	Using part of the Ford Escape Hybrid 0x431 CAN message (see src/can_dump/CAN_COMMANDS_IDENTIFIED.txt)
	//	431  00 00 00 00 00 00 00 10 - Seta para esquerda (acionada por humano)
	//	431  00 00 00 00 00 00 00 20 - Seta para direita (acionada por humano)
	digital_io_input.data[7] = ((digital_io >> 0) & 0x3) << 4;

	//	431  00 00 01 00 00 00 00 00 - Porta esquerda (trazeira) aberta (acionada por humano)
	//	431  00 00 02 00 00 00 00 00 - Porta direita (carona) aberta (acionada por humano)
	//	431  00 00 04 00 00 00 00 00 - Porta esquerda (motorista) aberta (acionada por humano)
	digital_io_input.data[2] = ((digital_io >> 11) & 0x1) | (((digital_io >> 8) & 0x3) << 1);

	//	431  00 00 00 80 00 00 00 00 - Porta direita (trazeira) aberta (acionada por humano)
	digital_io_input.data[3] = ((digital_io >> 10) & 0x1) << 7;


	//	431  01 00 00 00 00 00 00 00 - Farol Baixo
	//	431  02 00 00 00 00 00 00 00 - Farol Alto
	digital_io_input.data[0] = (digital_io >> 2) & 0x3;

	//	431  00 01 00 00 00 00 00 00 - Buzina
	digital_io_input.data[1] = (digital_io >> 4) & 0x1;

	//	431  00 00 00 00 01 00 00 00 - Limpador de parabrisa lento
	//	431  00 00 00 00 02 00 00 00 - Limpador de parabrisa rapido
	digital_io_input.data[4] = (digital_io >> 5) & 0x3;

	esp_err_t error = can_transmit(&digital_io_input, pdMS_TO_TICKS(1000));
//	printf("t d\n");
	if (error != ESP_OK)
	{
		printf("*** can_transmit(&digital_io_input, pdMS_TO_TICKS(1000)) error\n");
		examine_can_errors_and_try_to_restart();
	}
}


int
manual_override_state_machine(int button)
{
	if (button == LOW)
		return (0);
	else
		return (1);
}


void 
manual_override_safe_stop_and_digital_io_input_handler(void *parameter)
{
	while (1)
	{
		manual_override_button = gpio_get_level_with_debounce(YELLOW_BUTTON_PIN, manual_override_button);
		int manual_override_yellow_button_state = manual_override_state_machine(manual_override_button);

//		printf("safe_stop_button %d, manual_override_button %d\n", safe_stop_button, manual_override_button);

		if (safe_stop_button)
			set_safe_stop_mode();
		else if (manual_override_yellow_button_state)
			set_manual_override_mode();
		else
			set_autonomous_mode();

		send_manual_override_and_safe_stop_CAN_message(manual_override_yellow_button_state);

		usleep(100000);    // 0.2s to operate at about 5Hz
		int digital_io = gpio_get_digital_io();
		send_digital_io_in_CAN_message(digital_io);

		usleep(100000);    // 0.2s to operate at about 5Hz
		zero_dot_two_seconds_counter++;
    }
}


void
send_bits_to_non_essential_signals(void *parameter)
{
	static int bits_to_send = -1;

	while (1)
	{
	    xSemaphoreTake(digital_io_sem, portMAX_DELAY);      // Wait for RX task to complete
	    int there_are_bits_to_send = 0;
		if (bits_to_send != resquested_bits_to_send)
		{
			bits_to_send = resquested_bits_to_send;
			there_are_bits_to_send = 1;
		}
	    xSemaphoreGive(digital_io_sem);

		if (there_are_bits_to_send)
		{
//			printf("bits_to_send 0x%x\n", bits_to_send);
			send_bits_serially(bits_to_send, 7);
			there_are_bits_to_send = 0;
		}

		usleep(5000);    // 0.2s to operate at about 5Hz
    }
}


void
configure_CAN_messages_to_trasnmit()                        // flag types
{                                                           // CAN_MSG_FLAG_EXTD			Message is in Extended Frame Format (29bit ID)
															// CAN_MSG_FLAG_DLC_NON_COMP	Message’s Data length code is larger than 8. This will break compliance with CAN2.0B
	manual_override_and_safe_stop.identifier = 0x600;       // CAN_MSG_FLAG_RTR				Message is a Remote Transmit Request
	manual_override_and_safe_stop.flags = CAN_MSG_FLAG_SS;  // CAN_MSG_FLAG_SS				Transmit message using Single Shot Transmission (Message will not be retransmitted upon error or loss of arbitration)
	manual_override_and_safe_stop.data_length_code = 1;     // CAN_MSG_FLAG_SELF			Transmit message using Self Reception Request (Transmitted message will also received by the same node)

	digital_io_input.identifier = 0x431;					// CAN_MSG_FLAG_RTR				Message is a Remote Transmit Request
	digital_io_input.flags = CAN_MSG_FLAG_SS;   			// CAN_MSG_FLAG_SS				Transmit message using Single Shot Transmission (Message will not be retransmitted upon error or loss of arbitration)
	digital_io_input.data_length_code = 8;      			// CAN_MSG_FLAG_SELF			Transmit message using Self Reception Request (Transmitted message will also received by the same node)

	tramontina_speed_can_message.identifier = 0x201;
	tramontina_speed_can_message.flags = CAN_MSG_FLAG_SS;
	tramontina_speed_can_message.data_length_code = 2;
}


static void IRAM_ATTR
tramontina_speed_pulse_interrupt_handler(void *not_used)
{
	UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

	signalB = gpio_get_level(SPEED_SIGNAL_B);
	if (signalB == 1)
		pulseCount++;
	else
		pulseCount--;
	// pulseCounter[pulseCounterIndex]++;

	taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}


#define TRAMONTINA_SPEED_CONSTANT 1.0

void
publish_tramontina_speed(void *parameter)
{
	while (1)
	{
		usleep(25000); // 40Hz

		// int_speed = 0;

		// for (int i = 0; i < NUM_PULSE_COUNTERS; i++)
		// 	int_speed += pulseCounter[i];

		taskENTER_CRITICAL(&my_spinlock);
		// pulseCounterIndex++;
		// if (pulseCounterIndex >= NUM_PULSE_COUNTERS)
		// 	pulseCounterIndex = 0;
		// pulseCounter[pulseCounterIndex] = 0;

		int_speed = pulseCount;
		pulseCount = 10000;
        taskEXIT_CRITICAL(&my_spinlock);

		// int_speed = round((double) int_speed / (double) NUM_PULSE_COUNTERS);
		tramontina_speed_can_message.data[0] = int_speed & 0xFF;
		tramontina_speed_can_message.data[1] = (int_speed >> 8) & 0xFF;

		esp_err_t error = can_transmit(&tramontina_speed_can_message, pdMS_TO_TICKS(1000));
		if (error != ESP_OK)
		{
			printf("*** can_transmit(&tramontina_speed_can_message, pdMS_TO_TICKS(1000)) error!\n");
			examine_can_errors_and_try_to_restart();
		}
    }
}


void
init_sincos_cosine_signal()
{
    dac_cw_config_t cw;
    cw.en_ch = DAC_CHANNEL_2;	/*!< Enable the cosine wave generator of DAC channel. */
    cw.scale = DAC_CW_SCALE_1;	/*!< Set the amplitude of the cosine wave generator output. */
    cw.phase = DAC_CW_PHASE_0;	/*!< Set the phase of the cosine wave generator output. */
    cw.freq = 10000;			/*!< Set frequency of cosine wave generator output. Range: 130(130Hz) ~ 55000(100KHz). */
    cw.offset = 1;				/*!< Set the voltage value of the DC component of the cosine wave generator output.
                                     Note: Unreasonable settings can cause waveform to be oversaturated. Range: -128 ~ 127. */
    dac_cw_generator_config(&cw);
    dac_output_enable(DAC_CHANNEL_2);
//    dac_output_voltage(DAC_CHANNEL_2, 200);
    dac_cw_generator_enable();
}


static void
check_efuse(void)
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


static void
print_char_val_type(esp_adc_cal_value_t val_type)
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
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, IBT_2_SIGNAL1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, IBT_2_SIGNAL2);
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


void
ojSleepMsec(double miliseconds)
{
	double t = ojGetTimeSec();
    while ((ojGetTimeSec() - t) < (miliseconds / 1000.0))
    	ets_delay_us(10);			//Stalls execution for #uS
}


void
gpioPWM(int push_pull, double pwm)
{
	if (push_pull == PUSHBRAKES)
		brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, pwm);
	else
		brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, pwm);
}


void
apply_break_effort(double ut)
{
	static int push = 0;

	if (ut > BRAKES_MIN_ut)
	{
		if (!push)
		{
			gpioPWM(PULLBRAKES, 0);
			ojSleepMsec(1);
			push = 1;
		}
		double pwm = ut * (double) BRAKES_PWM_RANGE / BRAKES_MAX_ut;
		if (pwm > BRAKES_PWM_RANGE)
			pwm = BRAKES_PWM_RANGE;
		gpioPWM(PUSHBRAKES, pwm);
	}
	else if (ut < -BRAKES_MIN_ut)
	{
		if (push)
		{
			gpioPWM(PUSHBRAKES, 0);
			ojSleepMsec(1);
			push = 0;
		}
		double pwm = -ut * (double) BRAKES_PWM_RANGE / BRAKES_MAX_ut;
		if (pwm > BRAKES_PWM_RANGE)
			pwm = BRAKES_PWM_RANGE;
		gpioPWM(PULLBRAKES, pwm);
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

	u_t = BRAKES_PID_Kp * error_t +
		  BRAKES_PID_Ki * integral_t +
		  BRAKES_PID_Kd * derivative_t;

	error_t_1 = error_t;

	// Anti windup
	if ((u_t < -BRAKES_MAX_ut) || (u_t > BRAKES_MAX_ut))
		integral_t = integral_t_1;
	integral_t_1 = integral_t;

	previous_t = t;

	u_t = saturate(-BRAKES_MAX_ut, u_t, BRAKES_MAX_ut);

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
    // Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
        if (unit == ADC_UNIT_1) // Usamos a ADC_UNIT_2
        {
            adc_reading += adc1_get_raw((adc1_channel_t) channel);
        }
        else
        {
            int raw;
            adc2_get_raw((adc2_channel_t) channel, ADC_WIDTH_BIT_12, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES;
    // Convert adc_reading to voltage in mV
    int voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

    return (voltage);
}


void
update_breaks(double g_break_effort)
{
	int desired_voltage = get_voltage_from_break_effort(g_break_effort);
    g_voltage = get_breaks_voltage();
	double ut = breaks_pid(desired_voltage, g_voltage, autonomous);
	apply_break_effort(ut);
}


void
set_break_effort(void *desired_break_effort)
{
	double break_effort;

	while (1)
	{
		break_effort = (double) (*((float *) desired_break_effort));

		update_breaks(break_effort);
		// ojSleepMsec(5.0);
		// g_voltage += 1;
		// float *x = (float *) desired_break_effort;
		// *x += 1.0;
		usleep(5000);    // 0.2s to operate at about 5Hz
	}
}


void
init_breaks()
{
	// Init breaks PWM
	mcpwm_gpio_initialize();

    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

	// Init breaks pontentiometer sensor (https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/adc.html)
    if (unit == ADC_UNIT_1) // Usamos a ADC_UNIT_2
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    }
    else
        adc2_config_channel_atten((adc2_channel_t) channel, atten);

    // Characterize ADC
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
    pwm_config.frequency = BRAKES_PWM_FREQUENCY;
    pwm_config.cmpr_a = 0;    	// duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    	// duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    // Configure PWM0A & PWM0B with above settings

    double t;
    g_voltage = get_breaks_voltage();
	printf("0 - g_voltage %d\n\r", g_voltage);
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
	printf("1 - g_voltage %d\n\r", g_voltage);

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
	printf("\n2 - g_voltage %d\n\r", g_voltage);
}


void
app_main(void)
{
	esp_wifi_stop();
    esp_wifi_deinit();

	configure_CAN_messages_to_trasnmit();

	// Init Tramontina Throttle
	dac_output_enable(DAC_CHANNEL_1);	// Sinal de Throttle em substituicao pontenciomentro acelerador GPIO_25
	gpio_set_direction(RELAY_THROTTLE, GPIO_MODE_OUTPUT);
	gpio_set_level(RELAY_THROTTLE, HIGH);	// Active low
	printf("Throttle Initialized!\n");

	// Necessario para o freio
	init_sincos_cosine_signal();
	printf("SINCOS Cossine Signal Initialized!\n");

    digital_io_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(digital_io_sem);

    // Init Tramontina speed
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SPEED_SIGNAL_A, tramontina_speed_pulse_interrupt_handler, (void *) SPEED_SIGNAL_A);

    float break_effort = 0.0;

	initialize_manual_override_throttle_and_gear();
	initialize_digital_io();
    vTaskDelay(pdMS_TO_TICKS(100));		// Wait 100ms

    // CAN bus setup
    rx_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(can_message_handler, "can_message_handler", 4096, &break_effort, 9, NULL, 0);

	xTaskCreatePinnedToCore(publish_tramontina_speed, "publish_tramontina_speed", 4096, NULL, 9, NULL, 0);

	xTaskCreatePinnedToCore(manual_override_safe_stop_and_digital_io_input_handler, "manual_override_safe_stop_and_digital_io_input_handler", 4096, NULL, 9, NULL, 1);

	xTaskCreatePinnedToCore(send_bits_to_non_essential_signals, "send_bits_to_non_essential_signals", 4096, NULL, 9, NULL, 1);

    xTaskCreatePinnedToCore(set_break_effort, "set_break_effort", 4096, &break_effort, 9, NULL, 1);

    // Init Tramontina brakes
    init_breaks();
	printf("Brakes Initialized!\n");

	// Install and start CAN driver
    ESP_ERROR_CHECK(can_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");
    ESP_ERROR_CHECK(can_start());
    ESP_LOGI(EXAMPLE_TAG, "Driver started");
	// Reconfigure alerts to detect Error Passive and Bus-Off error states
	uint32_t alerts_to_enable = CAN_ALERT_ABOVE_ERR_WARN | CAN_ALERT_ERR_PASS | CAN_ALERT_BUS_OFF | CAN_ALERT_TX_FAILED | CAN_ALERT_RX_QUEUE_FULL | CAN_ALERT_ARB_LOST | CAN_ALERT_BUS_RECOVERED | CAN_ALERT_BUS_ERROR;
	if (can_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK)
	    printf("Alerts reconfigured\n");
	else
	    printf("Failed to reconfigure alerts\n");

    xSemaphoreGive(rx_sem);                     //Start RX task

	vTaskDelay(1000 / portTICK_RATE_MS);

    int cnt = 0;
    while (1)
    {
		cnt++;

		printf("g_voltage %d, break_effort %f\n\r", g_voltage, break_effort);

	    vTaskDelay(pdMS_TO_TICKS(1000/2));		// Wait 1000ms/2
    }

    vTaskDelay(pdMS_TO_TICKS(100));				// Wait 100ms
    xSemaphoreTake(rx_sem, portMAX_DELAY);      // Wait for RX task to complete

    //Stop and uninstall CAN driver
    ESP_ERROR_CHECK(can_stop());
    ESP_LOGI(EXAMPLE_TAG, "Driver stopped");
    ESP_ERROR_CHECK(can_driver_uninstall());
    ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

    // Cleanup
    vSemaphoreDelete(digital_io_sem);
    vSemaphoreDelete(rx_sem);
    // end CAN bus setup
}
