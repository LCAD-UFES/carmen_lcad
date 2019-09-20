#include <ctype.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>

#include <jaus.h>
#include <openJaus.h>
#include <torc.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <ncurses.h>
#include <termios.h>
#include <unistd.h>

#include <pthread.h>

#include <pigpio.h>

#define CLEAR "clear"

#include "pd.h"
#include "vss.h"
#include "mpd.h"
#include "sd.h"
#include "can_utils.h"

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#define DESOUZA_GUIDOLINI_CONSTANT 					0.0022
#define robot_distance_between_front_and_rear_axles	2.625
#define robot_understeer_coeficient					0.0015

#define FRONT_RIGHT	0
#define FRONT_LEFT	1
#define BACK_RIGHT	2
#define BACK_LEFT	3
// Metros por revolucao do pneu ~= 2.2375 m
#define METERS_BY_ODOMETRY_COUNT	0.0255
#define MAX_ODOMETER 				10000
#define ODOMETER_FIFO_SIZE 			12

#define DEFAULT_STRING_LENGTH 		128
#define KEYBOARD_LOCK_TIMEOUT_SEC	60.0

// Breaks
#define	PUSHBREAKS			 4 // GPIO  4, pino  7
#define	PULLBREAKS			27 // GPIO 27, pino 13
#define	HALL2_EXTENDING		22 // GPIO 22, pino 15
#define	HALL1_TRANSITION	23 // GPIO 23, pino 16, signal leads when extending

// main loop and interface
static int mainRunning = FALSE;
static int verbose = FALSE; // Se verdadeiro, printf() funciona; caso contrario, nao.
static int keyboardLock = FALSE;
int interface_active = FALSE;

// Operating specific console handles
static struct termios newTermio;
static struct termios storedTermio;

OjCmpt pd;
OjCmpt vss;
OjCmpt mpd;
OjCmpt sd;

int in_can_sockfd = -1;
int out_can_sockfd = -1;

double front_left_speed[WHEEL_SPEED_MOVING_AVERAGE_SIZE];
double front_right_speed[WHEEL_SPEED_MOVING_AVERAGE_SIZE];
double back_left_speed[WHEEL_SPEED_MOVING_AVERAGE_SIZE];
double back_right_speed[WHEEL_SPEED_MOVING_AVERAGE_SIZE];

double car_speed = 0.0;
double speed_signal = 1.0;
double steering_angle = 0.0;
unsigned int manual_override_and_safe_stop;
int turn_signal;
int door_signal;

int calibrate_steering_wheel_zero_angle = 0;
int calibrate_steering_wheel_zero_torque = 0;
int steering_angle_sensor = 20000.0 + 30.0;
int steering_angle_auxiliary_sensor = 0xFFFC;
int steering_angle_sensor_zero = 20000.0 + 30.0;

int steering_wheel_zero_torque = -30;


// Refresh screen in curses mode
void updateScreen(int keyboardLock, int keyPress)
{
	int row = 0;
	int col = 0;
	char string[256] = {0};
	static int lastChoice = '1';
	JausAddress address;

	if(!keyboardLock && keyPress != -1 && keyPress != 27 && keyPress != 12) // 27 = ESC, 12 = Ctrl+l
		lastChoice = keyPress;

	clear();

	mvprintw(row,35,"Keyboard Lock:	%s", keyboardLock?"ON, Press ctrl+L to unlock":"OFF, Press ctrl+L to lock");

	mvprintw(row++,0,"+---------------------------+");
	mvprintw(row++,0,"|           Menu            |");
	mvprintw(row++,0,"|                           |");
	mvprintw(row++,0,"| 1. Primitive Driver & VSS |");
	mvprintw(row++,0,"|                           |");
	mvprintw(row++,0,"| ESC to Exit               |");
	mvprintw(row++,0,"+---------------------------+");

	row = 2;
	col = 40;
	switch(lastChoice)
	{
		case '1':
			mvprintw(row++,col,"== Primitive Driver ==");
			mvprintw(row++,col,"PD State Machine Running: %s", (ojCmptIsRunning(pd))? "true": "false");
			mvprintw(row++,col,"PD Update Rate:	%5.2f", ojCmptGetRateHz(pd));
			address = ojCmptGetAddress(pd);
			jausAddressToString(address, string);
			jausAddressDestroy(address);
			mvprintw(row++,col,"PD Address:\t%s", string);
			mvprintw(row++,col,"PD State:\t%s", jausStateGetString(ojCmptGetState(pd)));

			row++;
			if(ojCmptHasController(pd))
			{
				address = ojCmptGetControllerAddress(pd);
				jausAddressToString(address, string);
				jausAddressDestroy(address);
				mvprintw(row++,col,"PD Controller:	%s", string);
			}
			else
			{
				mvprintw(row++,col,"PD Controller:	None");
			}
			mvprintw(row++,col,"PD Controller SC:	%s", pdGetControllerScStatus(pd)?"Active":"Inactive");
			mvprintw(row++,col,"PD Controller State:	%s", jausStateGetString(pdGetControllerState(pd)));

			row++;
			mvprintw(row++,col,"PD Prop Effort X: %0.0lf", pdGetWrenchEffort(pd)? pdGetWrenchEffort(pd)->propulsiveLinearEffortXPercent:-1.0);
			mvprintw(row++,col,"PD Rstv Effort X: %0.0lf", pdGetWrenchEffort(pd)? pdGetWrenchEffort(pd)->resistiveLinearEffortXPercent:-1.0);
			mvprintw(row++,col,"PD Rtat Effort Z: %0.0lf", pdGetWrenchEffort(pd)? pdGetWrenchEffort(pd)->propulsiveRotationalEffortZPercent:-1.0);

			row++;
			mvprintw(row++,col,"PD Main Propulsion: %s", pdGetDiscreteDevices(pd)? ((pdGetDiscreteDevices(pd)->mainPropulsion)?"on":"off"):" ");
			mvprintw(row++,col,"PD Main FuelSupply: %s", pdGetDiscreteDevices(pd)? ((pdGetDiscreteDevices(pd)->mainFuelSupply)?"on":"off"):" ");
			mvprintw(row++,col,"PD Gear: %d", pdGetDiscreteDevices(pd)? pdGetDiscreteDevices(pd)->gear:-1);
			mvprintw(row++,col,"PD Horn: %s", pdGetDiscreteDevices(pd)? ((pdGetDiscreteDevices(pd)->horn)?"on":"off"):" ");
			mvprintw(row++,col,"PD Parking Break %s", pdGetDiscreteDevices(pd)? ((pdGetDiscreteDevices(pd)->parkingBrake)?"on":"off"):" ");

//			break;
//
//		case '2':
			row++;
			mvprintw(row++,col,"== Velocity State Sensor ==");
			mvprintw(row++,col,"VSS Update Rate:  %6.2f", ojCmptGetRateHz(vss));
			address = ojCmptGetAddress(vss);
			jausAddressToString(address, string );
			jausAddressDestroy(address);
			mvprintw(row++,col,"VSS Address:\t    %s", string);
			mvprintw(row++,col,"VSS State:\t    %s", jausStateGetString(ojCmptGetState(vss)));
			mvprintw(row++,col,"VSS SC State:\t    %s", vssGetScActive(vss)? "Active" : "Inactive");
			break;

		default:
			mvprintw(row++,col,"NONE.");
			break;
	}

	move(24,0);
	refresh();
}

void parseUserInput(char input)
{
	switch(input)
	{
		case 12: // 12 == 'ctrl + L'
			keyboardLock = !keyboardLock;
			break;

		case 27: // 27 = ESC
			if(!keyboardLock)
				mainRunning = FALSE;
			break;

		default:
			break;
	}
	return;
}

void setupTerminal()
{
	if(verbose)
	{
		tcgetattr(0,&storedTermio);
		memcpy(&newTermio,&storedTermio,sizeof(struct termios));

		// Disable canonical mode, and set buffer size to 0 byte(s)
		newTermio.c_lflag &= (~ICANON);
		newTermio.c_lflag &= (~ECHO);
		newTermio.c_cc[VTIME] = 0;
		newTermio.c_cc[VMIN] = 0;
		tcsetattr(0,TCSANOW,&newTermio);
	}
	else
	{
		// Start up Curses window
		initscr();
		cbreak();
		noecho();
		nodelay(stdscr, 1);	// Don't wait at the getch() function if the user hasn't hit a key
		keypad(stdscr, 1); // Allow Function key input and arrow key input
	}
}

void cleanupConsole()
{
	if(verbose)
	{
		tcsetattr(0,TCSANOW,&storedTermio);
	}
	else
	{
		// Stop Curses
		clear();
		endwin();
	}
}

char getUserInput()
{
	char retVal = FALSE;
	int choice = -1;

	if(verbose)
	{
		choice = getc(stdin);
		if(choice > -1)
		{
			parseUserInput(choice);
			retVal = TRUE;
		}
	}
	else
	{
		choice = getch(); // Get the key that the user has selected
		updateScreen(keyboardLock, choice);
		if(choice > -1)
		{
			parseUserInput(choice);
			retVal = TRUE;
		}
	}

	return retVal;
}

void init_modules()
{
//	usleep(1000000); // Espera um segundo para o NodeManager entrar

	pd = pdCreate();
	if (!pd)
		exit(1);

	vss = vssCreate();
	if (!vss)
		exit(1);

	mpd = mpdCreate();
	if (!mpd)
		exit(1);

	sd = sdCreate();
	if (!sd)
		exit(1);
}

void terminate_modules()
{
	pdDestroy(pd);
	vssDestroy(vss);
	mpdDestroy(mpd);
	sdDestroy(sd);
}

static void signal_handler(int signo)
{
	mainRunning = FALSE;
}

void clear_wheel_speed_moving_average()
{
	int wheel_speed_index = 0;
	while (wheel_speed_index < WHEEL_SPEED_MOVING_AVERAGE_SIZE)
	{
		front_left_speed[wheel_speed_index] = 0.0;
		front_right_speed[wheel_speed_index] = 0.0;
		back_left_speed[wheel_speed_index] = 0.0;
		back_right_speed[wheel_speed_index] = 0.0;
		wheel_speed_index++;
	}
}

void update_wheel_speed(double *wheel_speed, int wheel_speed_index,
		int *wheel_odometer, int odometer_index, int delta_odometer, double *time_stamps, int odometer_previous_index, double t)
{
	if (delta_odometer < 0)
		delta_odometer += 256;

	int new_wheel_odometer = wheel_odometer[odometer_previous_index] + delta_odometer;
	if (new_wheel_odometer > MAX_ODOMETER)
		new_wheel_odometer -= MAX_ODOMETER;

	delta_odometer = new_wheel_odometer - wheel_odometer[odometer_index];
	if (delta_odometer < 0)
		delta_odometer += MAX_ODOMETER;
	wheel_odometer[odometer_index] = new_wheel_odometer;

	if (time_stamps[odometer_index] != 0.0)
		wheel_speed[wheel_speed_index] = speed_signal * ((double) delta_odometer * METERS_BY_ODOMETRY_COUNT) / (t - time_stamps[odometer_index]);
	else
		wheel_speed[wheel_speed_index] = 0.0;
}

void update_wheels_speed(struct can_frame frame)
{
	static int wheel_speed_index, odometer_index;

	static int front_left_odometer;
	static int front_right_odometer;
	static int back_left_odometer;
	static int back_right_odometer;

	static int front_left_odometer_fifo[ODOMETER_FIFO_SIZE];
	static int front_right_odometer_fifo[ODOMETER_FIFO_SIZE];
	static int back_left_odometer_fifo[ODOMETER_FIFO_SIZE];
	static int back_right_odometer_fifo[ODOMETER_FIFO_SIZE];
	static double time_stamps[ODOMETER_FIFO_SIZE];

	static int first_time = 1;

	double t = ojGetTimeSec();

	if (first_time)
	{
		memset((void *) front_left_odometer_fifo, 0, ODOMETER_FIFO_SIZE * sizeof(int));
		memset((void *) front_right_odometer_fifo, 0, ODOMETER_FIFO_SIZE * sizeof(int));
		memset((void *) back_left_odometer_fifo, 0, ODOMETER_FIFO_SIZE * sizeof(int));
		memset((void *) back_right_odometer_fifo, 0, ODOMETER_FIFO_SIZE * sizeof(int));

		memset((void *) time_stamps, 0, ODOMETER_FIFO_SIZE * sizeof(double));

		wheel_speed_index = odometer_index = 0;
		clear_wheel_speed_moving_average();

		front_left_odometer  = front_left_odometer_fifo[odometer_index] = (int) frame.data[FRONT_LEFT];
		front_right_odometer = front_right_odometer_fifo[odometer_index] = (int) frame.data[FRONT_RIGHT];
		back_left_odometer   = back_left_odometer_fifo[odometer_index] = (int) frame.data[BACK_LEFT];
		back_right_odometer  = back_right_odometer_fifo[odometer_index] = (int) frame.data[BACK_RIGHT];

		time_stamps[odometer_index] = t;

		odometer_index++;
		first_time = 0;

		return;
	}

	int odometer_previous_index = odometer_index - 1;
	if (odometer_previous_index < 0)
		odometer_previous_index = ODOMETER_FIFO_SIZE - 1;

	update_wheel_speed(front_left_speed, wheel_speed_index, front_left_odometer_fifo, odometer_index, frame.data[FRONT_LEFT] - front_left_odometer,
			time_stamps, odometer_previous_index, t);
	update_wheel_speed(front_right_speed, wheel_speed_index, front_right_odometer_fifo, odometer_index, frame.data[FRONT_RIGHT] - front_right_odometer,
			time_stamps, odometer_previous_index, t);
	update_wheel_speed(back_left_speed, wheel_speed_index, back_left_odometer_fifo, odometer_index, frame.data[BACK_LEFT] - back_left_odometer,
			time_stamps, odometer_previous_index, t);
	update_wheel_speed(back_right_speed, wheel_speed_index, back_right_odometer_fifo, odometer_index, frame.data[BACK_RIGHT] - back_right_odometer,
			time_stamps, odometer_previous_index, t);

	front_left_odometer = (int) frame.data[FRONT_LEFT];
	front_right_odometer = (int) frame.data[FRONT_RIGHT];
	back_left_odometer = (int) frame.data[BACK_LEFT];
	back_right_odometer = (int) frame.data[BACK_RIGHT];

	time_stamps[odometer_index] = t;

	odometer_index++;
	if (odometer_index == ODOMETER_FIFO_SIZE)
		odometer_index = 0;

	wheel_speed_index++;
	if (wheel_speed_index == WHEEL_SPEED_MOVING_AVERAGE_SIZE)
		wheel_speed_index = 0;
}

void update_car_speed(struct can_frame frame)
{
	car_speed = ((double) frame.data[0] * 256.0 + (double) frame.data[1]) * DESOUZA_GUIDOLINI_CONSTANT;
	speed_signal = (((double) frame.data[6] * 256.0 + (double) frame.data[7]) > 0x286C)? 1.0: -1.0;
	car_speed *= speed_signal;
}


double wheel_speed_moving_average(double *wheel_speed)
{
	int i;
	double moving_average_wheel_speed = 0.0;

	for (i = 0; i < WHEEL_SPEED_MOVING_AVERAGE_SIZE; i++)
		moving_average_wheel_speed += wheel_speed[i];

	return (moving_average_wheel_speed / (double) WHEEL_SPEED_MOVING_AVERAGE_SIZE);
}

void update_steering_angle(struct can_frame frame)
{
//	steering_angle = -(((double) frame.data[2] * 256.0 + (double) frame.data[3]) - 20015.0) / 28200.0;
	steering_angle_sensor = frame.data[2] * 256.0 + frame.data[3];
	steering_angle_auxiliary_sensor = frame.data[0] * 256.0 + frame.data[1];
	double val = (double) (steering_angle_sensor - steering_angle_sensor_zero);
	double phi;
	if (val > 0.0)
		phi = (0.000000106681 * val * val + 0.00532682 * val) * M_PI / 180.0; // Ver arquivo "medicoes na boca rodas.xlsx"
	else
	{
		val = -val;
		phi = -(0.000000106681 * val * val + 0.00532682 * val) * M_PI / 180.0; // Ver arquivo "medicoes na boca rodas.xlsx"
	}

	double v = (wheel_speed_moving_average(back_left_speed) + wheel_speed_moving_average(back_right_speed)) / 2.0;
	double curvature = tan(phi / (1.0 + v * v * robot_understeer_coeficient)) / robot_distance_between_front_and_rear_axles; // Ver pg. 42 do ByWire XGV User Manual, Version 1.5
	steering_angle = -atan(curvature); // Ver pg. 73 do ByWire XGV User Manual, Version 1.5
}

void update_manual_override_and_safe_stop(struct can_frame frame)
{
	manual_override_and_safe_stop = frame.data[0];
}

void update_signals(struct can_frame frame)
{
	turn_signal = frame.data[7];
	door_signal = (frame.data[2] << 8) | frame.data[3];
}

void perform_steering_wheel_calibration(struct can_frame frame)
{
	if (frame.data[0] == 0x13) // Calibracao de angulo zero
		calibrate_steering_wheel_zero_angle = 1;
	else if (frame.data[0] == 0x14) // Calibracao torque zero
		calibrate_steering_wheel_zero_torque = 1;
}

void update_Car_state(struct can_frame frame)
{
	if (frame.can_id == 0x216) // Odometro das rodas. Frequencia de 60Hz no Ford Escape Hybrid
		update_wheels_speed(frame);

	if (frame.can_id == 0x425) // Velocidade da IARA. Frequencia de 60Hz no Ford Escape Hybrid
		update_car_speed(frame);

	if (frame.can_id == 0x80) // Angulo do volante
		update_steering_angle(frame);

	if (frame.can_id == 0x431) // Setas acionadas manualmente e estado das portas (abertas/fechadas)
		update_signals(frame);

//	if (frame.can_id == 0x216) // Safe Stop?
//		update_wheels_speed(frame);
}

void update_Torc_state(struct can_frame frame)
{
	if (frame.can_id == 0x600) // Botao amarelo
		update_manual_override_and_safe_stop(frame);

	if (frame.can_id == 0x113) // Calibrar volante
		perform_steering_wheel_calibration(frame);
}

void *can_in_read_thread_func(void *unused)
{
	struct can_frame frame;

	while (mainRunning)
	{
		if (in_can_sockfd != -1)
		{
			recv_frame(in_can_sockfd, &frame);
			update_Car_state(frame);
		}
	}

	return (NULL);
}

void *can_out_read_thread_func(void *unused)
{
	struct can_frame frame;

	while (mainRunning)
	{
		if (out_can_sockfd != -1)
		{
			recv_frame(out_can_sockfd, &frame);
			update_Torc_state(frame);
		}
	}

	return (NULL);
}

void calibrate_steering_wheel_zero_angle_state_machine()
{
#define IDLE 					0
#define WAIT_CLOCKWISE_LIMIT 	1
#define WAIT_SENSOR_RESET 		2
#define TIME_OUT_CONSTANT		5.0

	static int state = IDLE;
	static double wait_clockwise_limit_time = 0.0;
	static double last_steering_angle = 0.0;
	static double last_annotated_time = 0.0;

	if (state == IDLE)
	{
		wait_clockwise_limit_time = ojGetTimeSec();
		last_steering_angle = 0.0;

		state = WAIT_CLOCKWISE_LIMIT;
	}
	if (state == WAIT_CLOCKWISE_LIMIT)
	{
		if (last_steering_angle != steering_angle)
		{
			last_steering_angle = steering_angle;
			wait_clockwise_limit_time = ojGetTimeSec();
		}
		else if (ojGetTimeSec() - wait_clockwise_limit_time > 2.0)
			state = WAIT_SENSOR_RESET;

		send_efforts(0.0, 0.0, 100.0);
		send_gear(0x02); // Neutral

		last_annotated_time = ojGetTimeSec();
	}
	if (state == WAIT_SENSOR_RESET)
	{
		if (steering_angle_auxiliary_sensor != 0xFFFC)
		{
			steering_angle_sensor_zero = steering_angle_sensor - 2550;
			FILE *steering_angle_sensor_zero_file = fopen("/home/pi/steering_angle_sensor_zero_file.txt", "w");
			if (steering_angle_sensor_zero_file)
			{
				fprintf(steering_angle_sensor_zero_file, "%d\n", steering_angle_sensor_zero);
				fclose(steering_angle_sensor_zero_file);
			}
			state = IDLE;
			calibrate_steering_wheel_zero_angle = 0;
		}
		else if (ojGetTimeSec() - last_annotated_time > TIME_OUT_CONSTANT)
		{
			state = IDLE;
			calibrate_steering_wheel_zero_angle = 0;
		}
		else
		{
			send_efforts(0.0, 0.0, -80.0);
			send_gear(0x02); // Neutral
		}
	}
}

void calibrate_steering_wheel_zero_torque_state_machine()
{
#define MOVE_COUNTER_CLOCKWISE0			6
#define MOVE_CLOSE_TO_ZERO_ANGLE 		1
#define MOVE_CLOCKWISE 					2
#define MOVE_CLOSE_TO_ZERO_ANGLE2 		3
#define MOVE_COUNTER_CLOCKWISE 			4
#define CHANGE_ZERO_TORQUE 				5
#define SMALL_ANGLE						0.0
#define LARGE_ANGLE						0.13
#define PROPORTIONAL_CONSTANT_KP		2000.0
#define SMALL_ACCELERATION				0.01
#define ZERO_TORQUE_CORRECTION_FACTOR	100.0

	static int state = IDLE;
	static double last_annotated_time = 0.0;
	static double delta_t_clockwise = 0.0;
	static double delta_t_counter_clockwise = 0.0;

	if (state == IDLE)
	{
		last_annotated_time = ojGetTimeSec();
		delta_t_clockwise = 0.0;
		delta_t_counter_clockwise = 0.0;

		state = MOVE_COUNTER_CLOCKWISE0;
	}
	if (state == MOVE_COUNTER_CLOCKWISE0)
	{
		send_efforts(0.0, 0.0, -100.0);
		send_gear(0x02); // Neutral
		if ((fabs(steering_angle) > LARGE_ANGLE) || (ojGetTimeSec() - last_annotated_time > TIME_OUT_CONSTANT))
		{
			delta_t_counter_clockwise = ojGetTimeSec() - last_annotated_time;
			last_annotated_time = ojGetTimeSec();
			state = MOVE_CLOSE_TO_ZERO_ANGLE;
		}
	}
	if (state == MOVE_CLOSE_TO_ZERO_ANGLE)
	{
		if ((fabs(steering_angle) < SMALL_ANGLE) || (ojGetTimeSec() - last_annotated_time > TIME_OUT_CONSTANT))
		{
			send_efforts(0.0, 0.0, 0.0);
			send_gear(0x02); // Neutral

			if (ojGetTimeSec() - last_annotated_time > TIME_OUT_CONSTANT)
			{
				last_annotated_time = ojGetTimeSec();
				state = MOVE_CLOCKWISE;
			}
		}
		else
		{
			send_efforts(0.0, 0.0, -steering_angle * PROPORTIONAL_CONSTANT_KP);
			send_gear(0x02); // Neutral
		}
	}
	if (state == MOVE_CLOCKWISE)
	{
		send_efforts(0.0, 0.0, 100.0);
		send_gear(0x02); // Neutral
		if ((fabs(steering_angle) > LARGE_ANGLE) || (ojGetTimeSec() - last_annotated_time > TIME_OUT_CONSTANT))
		{
			delta_t_clockwise = ojGetTimeSec() - last_annotated_time;
			last_annotated_time = ojGetTimeSec();
			state = MOVE_CLOSE_TO_ZERO_ANGLE2;
		}
	}
	if (state == MOVE_CLOSE_TO_ZERO_ANGLE2)
	{
		if ((fabs(steering_angle) < SMALL_ANGLE) || (ojGetTimeSec() - last_annotated_time > TIME_OUT_CONSTANT))
		{
			send_efforts(0.0, 0.0, 0.0);
			send_gear(0x02); // Neutral

			if (ojGetTimeSec() - last_annotated_time > TIME_OUT_CONSTANT)
			{
				last_annotated_time = ojGetTimeSec();
				state = MOVE_COUNTER_CLOCKWISE;
			}
		}
		else
		{
			send_efforts(0.0, 0.0, -steering_angle * PROPORTIONAL_CONSTANT_KP);
			send_gear(0x02); // Neutral
		}
	}
	if (state == MOVE_COUNTER_CLOCKWISE)
	{
		send_efforts(0.0, 0.0, -100.0);
		send_gear(0x02); // Neutral
		if ((fabs(steering_angle) > LARGE_ANGLE) || (ojGetTimeSec() - last_annotated_time > TIME_OUT_CONSTANT))
		{
			delta_t_counter_clockwise = ojGetTimeSec() - last_annotated_time;
			last_annotated_time = ojGetTimeSec();
			state = CHANGE_ZERO_TORQUE;
		}
	}
	if (state == CHANGE_ZERO_TORQUE)
	{
		double zero_torque_correction = round((delta_t_clockwise - delta_t_counter_clockwise) * ZERO_TORQUE_CORRECTION_FACTOR);
		if (fabs(zero_torque_correction) < 1.0)
		{
			FILE *steering_wheel_zero_torque_file = fopen("/home/pi/steering_wheel_zero_torque_file.txt", "w");
			if (steering_wheel_zero_torque_file)
			{
				fprintf(steering_wheel_zero_torque_file, "%d\n", steering_wheel_zero_torque);
				fclose(steering_wheel_zero_torque_file);
			}
			state = IDLE;
			calibrate_steering_wheel_zero_torque = 0;
		}
		else
		{
			steering_wheel_zero_torque += zero_torque_correction;
			state = MOVE_CLOSE_TO_ZERO_ANGLE;
		}
	}
}

static int hall_ticks = 0;

void
hall_transition_interrupt(int gpio, int level, uint32_t tick)
{
	if (gpioRead(HALL2_EXTENDING))
		hall_ticks++;
	else
		hall_ticks--;
}


void
init_breaks()
{
	if (gpioInitialise() < 0)
		exit(printf("Error: Could initialize GPIO\n"));

	gpioSetMode(PUSHBREAKS, PI_OUTPUT);
	gpioSetMode(PULLBREAKS, PI_OUTPUT);
	gpioSetMode(HALL1_TRANSITION, PI_INPUT);
	gpioSetMode(HALL2_EXTENDING, PI_INPUT);

	gpioSetISRFunc(HALL1_TRANSITION, RISING_EDGE, 0, hall_transition_interrupt);

	printf("hall = %d\n\r", gpioRead(HALL2_EXTENDING));
	printf("hall_ticks %d\n\r", hall_ticks);

	gpioWrite(PUSHBREAKS, PI_HIGH);
	gpioWrite(PULLBREAKS, PI_LOW);
	ojSleepMsec(1000);
	printf("hall_ticks %d\n\r", hall_ticks);

	gpioWrite(PUSHBREAKS, PI_LOW);
	gpioWrite(PULLBREAKS, PI_LOW);
	ojSleepMsec(5);

	int previous_hall_ticks;
	do
	{
		previous_hall_ticks = hall_ticks;
		gpioWrite(PUSHBREAKS, PI_LOW);
		gpioWrite(PULLBREAKS, PI_HIGH);
		ojSleepMsec(500);
		gpioWrite(PUSHBREAKS, PI_LOW);
		gpioWrite(PULLBREAKS, PI_LOW);
	} while (previous_hall_ticks != hall_ticks);
	hall_ticks = 0;

	printf("hall_ticks %d\n\r", hall_ticks);

	gpioWrite(PUSHBREAKS, PI_LOW);
	gpioWrite(PULLBREAKS, PI_LOW);

	ojSleepMsec(3000);
}


int
main(int argCount, char **argString)
{
	pthread_t can_in_read_thread, can_out_read_thread;

	signal(SIGTERM, signal_handler);
	signal(SIGHUP, signal_handler);
	signal(SIGINT, signal_handler);

	if (argCount == 1) // Sem parametro liga a interface
		interface_active = TRUE;

	if (interface_active)
		system(CLEAR);
	else
	{
		in_can_sockfd = init_can(argString[1]);
		out_can_sockfd = init_can(argString[2]);
	}

	init_modules();

	if (interface_active)
		setupTerminal();

	FILE *steering_angle_sensor_zero_file = fopen("/home/pi/steering_angle_sensor_zero_file.txt", "r");
	if (steering_angle_sensor_zero_file)
	{
		fscanf(steering_angle_sensor_zero_file, "%d\n", &steering_angle_sensor_zero);
		fclose(steering_angle_sensor_zero_file);
	}
	FILE *steering_wheel_zero_torque_file = fopen("/home/pi/steering_wheel_zero_torque_file.txt", "r");
	if (steering_wheel_zero_torque_file)
	{
		fscanf(steering_wheel_zero_torque_file, "%d\n", &steering_wheel_zero_torque);
		fclose(steering_wheel_zero_torque_file);
	}

	init_breaks();

	mainRunning = TRUE;
	while(mainRunning)
	{
		if (interface_active)
		{
			getUserInput();
			ojSleepMsec(100);
		}
		else
		{
			static int first_time = true;
			if (first_time)
			{
				pthread_create(&can_in_read_thread, NULL, can_in_read_thread_func, NULL);
				pthread_create(&can_out_read_thread, NULL, can_out_read_thread_func, NULL);
				first_time = false;
			}

			if (calibrate_steering_wheel_zero_angle)
				calibrate_steering_wheel_zero_angle_state_machine();
			else if (calibrate_steering_wheel_zero_torque)
				calibrate_steering_wheel_zero_torque_state_machine();

			ojSleepMsec(5);
		}
	}

	if (interface_active)
		cleanupConsole();

	terminate_modules();

	if (!interface_active)
	{
		close(in_can_sockfd);
		close(out_can_sockfd);
	}

	gpioTerminate();

	return (0);
}
