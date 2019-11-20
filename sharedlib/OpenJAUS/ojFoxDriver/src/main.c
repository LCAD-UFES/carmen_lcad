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
#define METERS_BY_ODOMETRY_COUNT	0.01559375 // IARA 0.0255, Ford Fusion 19.96/(5*256)
#define MAX_ODOMETER 10000
#define ODOMETER_FIFO_SIZE 12

#define DEFAULT_STRING_LENGTH 128
#define KEYBOARD_LOCK_TIMEOUT_SEC	60.0

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
int steering_angle_sensor_zero = 16000.0;

char steering_angle_sensor_zero_file_name[1024];

int steering_wheel_zero_torque = -30;

char steering_wheel_zero_torque_file_name[1024];

struct can_dump
{
	double timestamp;
	char can_port[10];
	struct can_frame frame;
};

struct can_dump can_dump_record[10000];

#define NUM_MONITORED_CAN_IDS 	4
int monitored_can_ids[NUM_MONITORED_CAN_IDS] = {0x0C2, 0x0D0, 0x3D0, 0x729};

#define MONITORED_CAN_MESSAGE_QUEUE_SIZE 	15
int monitored_can_message_queue[MONITORED_CAN_MESSAGE_QUEUE_SIZE];
int monitored_can_message_queue_in_idx = 0;
int monitored_can_message_queue_out_idx = 0;

#define COMPRESS_DELTA_T		0.001
#define FIRST_CAN_DUMP_RECORD	(103 - 1)
#define LAST_CAN_DUMP_RECORD	(16000 - 1)
#define LOOP_CAN_DUMP_RECORD	(131 - 1)

//int can_dump_record_idx = 1;
int can_dump_record_idx = FIRST_CAN_DUMP_RECORD;

//static pthread_mutex_t monitored_can_message_queue_lock = PTHREAD_MUTEX_INITIALIZER;


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
	usleep(1000000); // Espera um segundo para o NodeManager entrar

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
//	speed_signal = (((double) frame.data[6] * 256.0 + (double) frame.data[7]) > 0x0800)? 1.0: -1.0; // Com can da IARA
	car_speed *= speed_signal;
}


void update_speed_signal(struct can_frame frame)
{
	speed_signal = (frame.data[1] == 0x2C)? -1.0: 1.0; // Marcha reh do Ford Fusion
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
//	printf("0x%x 0x%x 0x%x 0x%x\n", frame.data[0], frame.data[1], frame.data[2], frame.data[3]);
//	steering_angle = -(((double) frame.data[2] * 256.0 + (double) frame.data[3]) - 20015.0) / 28200.0;

	// IARA
//	steering_angle_sensor = frame.data[2] * 256.0 + frame.data[3];
//	steering_angle_auxiliary_sensor = frame.data[0] * 256.0 + frame.data[1];

	// Ford Fusion
	steering_angle_sensor = frame.data[0] * 256.0 + frame.data[1];
	steering_angle_auxiliary_sensor = frame.data[2] * 256.0 + frame.data[3]; // Nao sei se tem a mesma funcao que na IARA... Aparentemente nao

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

int monitored_can_id(int can_id)
{
	for (int i = 0; i < NUM_MONITORED_CAN_IDS; i++)
		if (can_id == monitored_can_ids[i])
			return (1);

	return (0);
}

void send_can_messages_after_monitored_can_message()
{
	static int sending_messages = 0;
	static double last_message_time = 0.0;

//	pthread_mutex_lock(&monitored_can_message_queue_lock);

	if (sending_messages)
	{
		printf(".");
		if ((ojGetTimeSec() - last_message_time) >
			(can_dump_record[can_dump_record_idx].timestamp - can_dump_record[can_dump_record_idx - 1].timestamp - COMPRESS_DELTA_T))
		{
			printf("send %d - (%lf) %s %03x#", can_dump_record_idx, ojGetTimeSec(),
					can_dump_record[can_dump_record_idx].can_port, can_dump_record[can_dump_record_idx].frame.can_id);
			for (int j = 0; j < can_dump_record[can_dump_record_idx].frame.can_dlc; j++)
				printf("%02x", can_dump_record[can_dump_record_idx].frame.data[j]);
			printf("\n");
			fflush(stdout);

			send_frame(out_can_sockfd, &(can_dump_record[can_dump_record_idx].frame));
			printf("+");

			can_dump_record_idx++;
			if (can_dump_record_idx > LAST_CAN_DUMP_RECORD)
				can_dump_record_idx = LOOP_CAN_DUMP_RECORD;

			if (monitored_can_id(can_dump_record[can_dump_record_idx].frame.can_id))
			{
				monitored_can_message_queue_out_idx++;
				if (monitored_can_message_queue_out_idx >= MONITORED_CAN_MESSAGE_QUEUE_SIZE)
					monitored_can_message_queue_out_idx = 0;
				printf("dequeue: in %d, out %d\n", monitored_can_message_queue_in_idx, monitored_can_message_queue_out_idx);
				fflush(stdout);

				sending_messages = 0;
			}

			last_message_time = ojGetTimeSec();
		}
	}
	else
	{
		printf("#");
		while (monitored_can_id(can_dump_record[can_dump_record_idx].frame.can_id))
		{
			printf("* %d - (%lf) %s %03x#", can_dump_record_idx, ojGetTimeSec(),
					can_dump_record[can_dump_record_idx].can_port, can_dump_record[can_dump_record_idx].frame.can_id);
			for (int j = 0; j < can_dump_record[can_dump_record_idx].frame.can_dlc; j++)
				printf("%02x", can_dump_record[can_dump_record_idx].frame.data[j]);
			printf("\n");
			fflush(stdout);

			can_dump_record_idx++;
		}

		sending_messages = 1;
		last_message_time = ojGetTimeSec();
	}

//	pthread_mutex_unlock(&monitored_can_message_queue_lock);
}

void send_initial_can_messages()
{
	static double last_message_time = 0.0;

//	pthread_mutex_lock(&monitored_can_message_queue_lock);

	if ((ojGetTimeSec() - last_message_time) >
		(can_dump_record[can_dump_record_idx].timestamp - can_dump_record[can_dump_record_idx - 1].timestamp - COMPRESS_DELTA_T))
	{
		send_frame(out_can_sockfd, &(can_dump_record[can_dump_record_idx].frame));

		printf("init send %d - (%lf) %s %03x#", can_dump_record_idx, ojGetTimeSec(),
				can_dump_record[can_dump_record_idx].can_port, can_dump_record[can_dump_record_idx].frame.can_id);
		for (int j = 0; j < can_dump_record[can_dump_record_idx].frame.can_dlc; j++)
			printf("%02x", can_dump_record[can_dump_record_idx].frame.data[j]);
		printf("\n");
		fflush(stdout);

		can_dump_record_idx++;
		if (can_dump_record_idx >= FIRST_CAN_DUMP_RECORD)
			can_dump_record_idx = 1;

		last_message_time = ojGetTimeSec();
	}

//	pthread_mutex_unlock(&monitored_can_message_queue_lock);
}

void enqueue_monitored_can_message(struct can_frame frame)
{
//	pthread_mutex_lock(&monitored_can_message_queue_lock);

	monitored_can_message_queue[monitored_can_message_queue_in_idx++] = frame.can_id;

	if (monitored_can_message_queue_in_idx >= MONITORED_CAN_MESSAGE_QUEUE_SIZE)
		monitored_can_message_queue_in_idx = 0;

	printf("enqueue: in %d, out %d\n", monitored_can_message_queue_in_idx, monitored_can_message_queue_out_idx);
	printf("recv %d - (%lf) %s %03x#", can_dump_record_idx, ojGetTimeSec(),
			can_dump_record[can_dump_record_idx].can_port, frame.can_id);
	for (int j = 0; j < frame.can_dlc; j++)
		printf("%02x", frame.data[j]);
	printf("\n");
	fflush(stdout);

	if (monitored_can_message_queue_in_idx == monitored_can_message_queue_out_idx)
		exit(printf("Error: monitored_can_message_queue full\n"));

//	pthread_mutex_unlock(&monitored_can_message_queue_lock);
}

void update_Car_state(struct can_frame frame)
{
	if (frame.can_id == 0x216) // Odometro das rodas. Frequencia de 60Hz no Ford Escape Hybrid
		update_wheels_speed(frame);

	if (frame.can_id == 0x415) // Velocidade da IARA. Frequencia de 60Hz no Ford Escape Hybrid
		update_car_speed(frame);

	if (frame.can_id == 0x76) // Angulo do volante
		update_steering_angle(frame);

	if (frame.can_id == 0x431) // Setas acionadas manualmente e estado das portas (abertas/fechadas)
		update_signals(frame);

	if (frame.can_id == 0x171) // Cambio do Ford Fusion
		update_speed_signal(frame);

	if (monitored_can_id(frame.can_id))
		enqueue_monitored_can_message(frame);

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
//			usleep(10);
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
//			usleep(10);
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
			FILE *steering_angle_sensor_zero_file = fopen(steering_angle_sensor_zero_file_name, "w");
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
			FILE *steering_wheel_zero_torque_file = fopen(steering_wheel_zero_torque_file_name, "w");
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

void load_can_dump(FILE *can_dump_file)
{
	char line[1024];
	char payload[64];
	int i = 0;
	while (fgets(line, 1023, can_dump_file))
	{
		sscanf(line, "(%lf) %s %x#%s\n",
					&(can_dump_record[i].timestamp), can_dump_record[i].can_port, &(can_dump_record[i].frame.can_id), payload);
		int payload_size = strlen(payload) / 2;
		can_dump_record[i].frame.can_dlc = payload_size;
		for (int j = payload_size - 1; j >= 0; j--)
		{
			can_dump_record[i].frame.data[j] = strtol(&(payload[j * 2]), NULL, 16);
			payload[j * 2] = '\0';
		}

//		printf("%d x - %s", i, line);
//		printf("%d y - (%lf) %s %03x#", i, can_dump_record[i].timestamp, can_dump_record[i].can_port, can_dump_record[i].frame.can_id);
//		for (int j = 0; j < payload_size; j++)
//			printf("%02x", can_dump_record[i].frame.data[j]);
//		printf("\n");

		i++;
	}
}

int main(int argCount, char **argString)
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

//	init_modules();

	if (interface_active)
		setupTerminal();

	strcpy(steering_angle_sensor_zero_file_name, getenv("CARMEN_HOME"));
	strcat(steering_angle_sensor_zero_file_name, "/sharedlib/OpenJAUS/ojVehicleDriver/steering_angle_sensor_zero_file.txt");
	FILE *steering_angle_sensor_zero_file = fopen(steering_angle_sensor_zero_file_name, "r");
	if (steering_angle_sensor_zero_file)
	{
		fscanf(steering_angle_sensor_zero_file, "%d\n", &steering_angle_sensor_zero);
		fclose(steering_angle_sensor_zero_file);
	}
	strcpy(steering_wheel_zero_torque_file_name, getenv("CARMEN_HOME"));
	strcat(steering_wheel_zero_torque_file_name, "/sharedlib/OpenJAUS/ojVehicleDriver/steering_wheel_zero_torque_file.txt");
	FILE *steering_wheel_zero_torque_file = fopen(steering_wheel_zero_torque_file_name, "r");
	if (steering_wheel_zero_torque_file)
	{
		fscanf(steering_wheel_zero_torque_file, "%d\n", &steering_wheel_zero_torque);
		fclose(steering_wheel_zero_torque_file);
	}

	char can_dump_file_name[1024];
	strcpy(can_dump_file_name, getenv("CARMEN_HOME"));
//	strcat(can_dump_file_name, "/sharedlib/OpenJAUS/ojFoxDriver/candump-2019-06-13_163031.log");
	strcat(can_dump_file_name, "/sharedlib/OpenJAUS/ojFoxDriver/log_can_fox_20190704-filtrado.log");
	FILE *can_dump_file = fopen(can_dump_file_name, "r");
	load_can_dump(can_dump_file);

//	int send_init_sequence = 1;
	int send_init_sequence = 0;
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

//			if (calibrate_steering_wheel_zero_angle)
//				calibrate_steering_wheel_zero_angle_state_machine();
//			else if (calibrate_steering_wheel_zero_torque)
//				calibrate_steering_wheel_zero_torque_state_machine();

			if (monitored_can_message_queue_in_idx != monitored_can_message_queue_out_idx)
			{
				if (send_init_sequence)
				{
					send_init_sequence = 0;
					can_dump_record_idx = FIRST_CAN_DUMP_RECORD;
				}
				send_can_messages_after_monitored_can_message();
				fflush(stdout);
			}
			else
			{
				if (send_init_sequence)
					send_initial_can_messages();
				printf(";");
				fflush(stdout);
			}

			usleep(10);
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

	return (0);
}
