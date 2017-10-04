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

#include <ncurses.h>
#include <termios.h>
#include <unistd.h>
#define CLEAR "clear"

#include "pd.h"
#include "vss.h"
#include "mpd.h"
#include "sd.h"

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#define DEFAULT_STRING_LENGTH 128
#define KEYBOARD_LOCK_TIMEOUT_SEC	60.0

static int mainRunning = FALSE;
static int verbose = FALSE; // Se verdadeiro, printf() funciona; caso contrario, nao.
static int keyboardLock = FALSE;
static char timeString[DEFAULT_STRING_LENGTH] = "";

// Operating specific console handles
static struct termios newTermio;
static struct termios storedTermio;

OjCmpt pd;
OjCmpt vss;
OjCmpt mpd;
OjCmpt sd;

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

int main(int argCount, char **argString)
{
	char keyPressed = FALSE;
	double keyboardLockTime = ojGetTimeSec() + KEYBOARD_LOCK_TIMEOUT_SEC;
	time_t timeStamp;

	//Get and Format Time String
	time(&timeStamp);
	strftime(timeString, DEFAULT_STRING_LENGTH-1, "%m-%d-%Y %X", localtime(&timeStamp));

	system(CLEAR);

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

	setupTerminal();

	mainRunning = TRUE;

	while(mainRunning)
	{
		keyPressed = getUserInput();

		if(keyPressed)
		{
			keyboardLockTime = ojGetTimeSec() + KEYBOARD_LOCK_TIMEOUT_SEC;
		}
		else if(ojGetTimeSec() > keyboardLockTime)
		{
//			keyboardLock = TRUE;
		}

		ojSleepMsec(100);
	}

	cleanupConsole();

	pdDestroy(pd);
	vssDestroy(vss);
	mpdDestroy(mpd);
	sdDestroy(sd);

	return 0;
}
