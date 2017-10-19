#include <ctype.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

#include <jaus.h>
#include <openJaus.h>
#include <torc.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <ncurses.h>
#include <termios.h>
#include <unistd.h>
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

#define DESOUZA_GUIDOLINI_CONSTANT 0.0020882803

#define FRONT_RIGHT	0
#define FRONT_LEFT	1
#define BACK_RIGHT	2
#define BACK_LEFT	3
// Com 0.05 (abaixo), velocidade maxima = 161.1 km/h (pois senao o odeometro termina uma volta)
#define WHEEL_VELOCITY_SAMPLE_TIME 	0.05
// Metros por revolucao do pneu ~= 2.2375 m
#define METERS_BY_ODOMETRY_COUNT	0.02632

#define DEFAULT_STRING_LENGTH 128
#define KEYBOARD_LOCK_TIMEOUT_SEC	60.0

static int mainRunning = FALSE;
static int verbose = FALSE; // Se verdadeiro, printf() funciona; caso contrario, nao.
static int keyboardLock = FALSE;
static int interface_active = FALSE;

// Operating specific console handles
static struct termios newTermio;
static struct termios storedTermio;

OjCmpt pd;
OjCmpt vss;
OjCmpt mpd;
OjCmpt sd;

int in_can_sockfd;
int out_can_sockfd;

int front_left_odometer = -1;
int front_right_odometer = -1;
int back_left_odometer = -1;
int back_right_odometer = -1;
double time_last_odometry = -1.0;

double front_left_speed[WHEEL_SPEED_MOVING_AVERAGE_SIZE];
double front_right_speed[WHEEL_SPEED_MOVING_AVERAGE_SIZE];
double back_left_speed[WHEEL_SPEED_MOVING_AVERAGE_SIZE];
double back_right_speed[WHEEL_SPEED_MOVING_AVERAGE_SIZE];
int wheel_speed_index;

double car_speed = 0.0;
double speed_signal = 1.0;

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

void int_modules()
{
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
	if (interface_active)
		cleanupConsole();

	terminate_modules();

	if (!interface_active)
	{
		close(in_can_sockfd);
		close(out_can_sockfd);
	}

	exit(0);
}

void clear_wheel_speed_moving_average()
{
	wheel_speed_index = 0;
	while (wheel_speed_index < WHEEL_SPEED_MOVING_AVERAGE_SIZE)
	{
		front_left_speed[wheel_speed_index] = 0.0;
		front_right_speed[wheel_speed_index] = 0.0;
		back_left_speed[wheel_speed_index] = 0.0;
		back_right_speed[wheel_speed_index] = 0.0;
		wheel_speed_index++;
	}
}

void update_wheels_speed(struct can_frame frame)
{
	double t = ojGetTimeSec();

	if (time_last_odometry == -1.0) // Inicializacao
	{
		front_left_odometer = (int) frame.data[FRONT_LEFT];
		front_right_odometer = (int) frame.data[FRONT_RIGHT];
		back_left_odometer = (int) frame.data[BACK_LEFT];
		back_right_odometer = (int) frame.data[BACK_RIGHT];

		time_last_odometry = t;
		clear_wheel_speed_moving_average();
		wheel_speed_index = 0;
	}
	else if ((t - time_last_odometry) > WHEEL_VELOCITY_SAMPLE_TIME)
	{
		double delta_t = t - time_last_odometry;

		int delta_odometer = frame.data[FRONT_LEFT] - front_left_odometer;
		if (delta_odometer < 0)
			delta_odometer += 256;
		front_left_speed[wheel_speed_index] = speed_signal * ((double) delta_odometer * METERS_BY_ODOMETRY_COUNT) / delta_t;

		delta_odometer = frame.data[FRONT_RIGHT] - front_right_odometer;
		if (delta_odometer < 0)
			delta_odometer += 256;
		front_right_speed[wheel_speed_index] = speed_signal * ((double) delta_odometer * METERS_BY_ODOMETRY_COUNT) / delta_t;

		delta_odometer = frame.data[BACK_LEFT] - back_left_odometer;
		if (delta_odometer < 0)
			delta_odometer += 256;
		back_left_speed[wheel_speed_index] = speed_signal * ((double) delta_odometer * METERS_BY_ODOMETRY_COUNT) / delta_t;

		delta_odometer = frame.data[BACK_RIGHT] - back_right_odometer;
		if (delta_odometer < 0)
			delta_odometer += 256;
		back_right_speed[wheel_speed_index] = speed_signal * ((double) delta_odometer * METERS_BY_ODOMETRY_COUNT) / delta_t;

		front_left_odometer = (int) frame.data[FRONT_LEFT];
		front_right_odometer = (int) frame.data[FRONT_RIGHT];
		back_left_odometer = (int) frame.data[BACK_LEFT];
		back_right_odometer = (int) frame.data[BACK_RIGHT];

		time_last_odometry = t;
		wheel_speed_index++;
		if (wheel_speed_index >= WHEEL_SPEED_MOVING_AVERAGE_SIZE)
			wheel_speed_index = 0;
	}
}

void update_car_speed(struct can_frame frame)
{
	car_speed = ((double) frame.data[0] * 256.0 + (double) frame.data[1]) * DESOUZA_GUIDOLINI_CONSTANT;
	speed_signal = (((double) frame.data[6] * 256.0 + (double) frame.data[7]) > 0x286C)? 1.0: -1.0;
	car_speed *= speed_signal;
}

void update_IARA_state(struct can_frame frame)
{
	if (frame.can_id == 0x216) // Odometro das rodas
		update_wheels_speed(frame);

	if (frame.can_id == 0x425) // Velocidade da IARA (segundo o can)
		update_car_speed(frame);
//
//	if (frame.can_id == 0xXX) // Reh
//		update_wheels_speed(frame);
//
//	if (frame.can_id == 0xXX) // Botao amarelo
//		update_wheels_speed(frame);
//
//	if (frame.can_id == 0xXX) // Setas acionadas manualmente e estado das portas (abertas/fechadas)
//		update_wheels_speed(frame);
//
//	if (frame.can_id == 0x216) // Safe Stop?
//		update_wheels_speed(frame);
}

int main(int argCount, char **argString)
{
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

	int_modules();

	if (interface_active)
		setupTerminal();

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
			struct can_frame frame;
			recv_frame(in_can_sockfd, &frame);
			update_IARA_state(frame);
			print_frame(&frame);
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
