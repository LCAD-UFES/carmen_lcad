#include <stdio.h>		
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>
#include <curses.h>

#include "cmtdef.h"
#include "xsens_time.h"
#include "xsens_list.h"
#include "cmtscan.h"
#include "cmt3.h"
#include "example_linux.h"

// this macro tests for an error and exits the program with a message if there was one
#define EXIT_ON_ERROR(res,comment) if (res != XRV_OK) { printw("Error %d occurred in " comment ": %s\n",res,xsensResultText(res)); exit(1); }

using namespace xsens;

inline int isnum(int c)
{
        return (c >= '0' && c <= '9');
}                                     

int main(int argc, char* argv[])
{
	(void) argc; (void) argv;	// Make the compiler stop complaining about unused parameters
	xsens::Cmt3 cmt3;
	int userQuit = 0;	
	unsigned long mtCount = 0;
	int temperatureOffset = 0;
	int screenSensorOffset = 0;
	CmtDeviceId deviceIds[256];
	
	CmtOutputMode mode;
	CmtOutputSettings settings;

	XsensResultValue res = XRV_OK;
	short screenSkipFactor = 10;
	short screenSkipFactorCnt = screenSkipFactor;

	initscr();
	noecho();
	nodelay(stdscr, 1);
	keypad(stdscr, 1);
	raw();

	// Perform hardware scan
	mtCount = doHardwareScan(cmt3, deviceIds);
	
	if (mtCount == 0) {
		printw("press q to quit\n");
		while(getch() != 'q');
		echo();
		endwin();
		cmt3.closePort();
		return 0;
	}
	
	getUserInputs(mode, settings);
	// Set device to user input settings
	doMtSettings(cmt3, mode, settings, deviceIds);
	refresh();
	screenSensorOffset = calcScreenOffset(mode, settings, screenSensorOffset);
	writeHeaders(mtCount, mode, settings, temperatureOffset, screenSensorOffset);
	// vars for sample counter & temp.
	unsigned short sdata;
	double tdata;
	
	//structs to hold data.
	CmtCalData caldata;
	CmtQuat qat_data;
	CmtEuler euler_data;
	CmtMatrix matrix_data;

	// Initialize packet for data
	Packet* packet = new Packet((unsigned short) mtCount, cmt3.isXm());
	int y, x;
	getyx(stdscr, y, x);
	x = 0;
	move(y, x);
	while (!userQuit && res == XRV_OK) 
	{
		XsensResultValue result = cmt3.waitForDataMessage(packet);
		if(result != XRV_OK)
		{
			if( (result == XRV_TIMEOUTNODATA) || (result == XRV_TIMEOUT) )
				continue;  //Ignore the error and restart the while loop			

			echo();
			endwin();
			delete packet;
			cmt3.closePort();
			printf("\nError %d occured in waitForDataMessage, can not recover.\n", result);
			exit(1);
		}

		//get sample count, goto position & display.
		sdata = packet->getSampleCounter();
		mvprintw(y, x, "Sample Counter %05hu\n", sdata);

		if (screenSkipFactorCnt++ != screenSkipFactor) {
			//continue;
		}
		screenSkipFactorCnt = 0;
		
		for (unsigned int i = 0; i < mtCount; i++) {	
			// Output Temperature
			if ((mode & CMT_OUTPUTMODE_TEMP) != 0) {					
				tdata = packet->getTemp(i);
				mvprintw(y + 4 + i * screenSensorOffset, x, "%6.2f", tdata);
			}
			move(y + 5 + temperatureOffset + i * screenSensorOffset, x);
			if ((mode & CMT_OUTPUTMODE_CALIB) != 0) {					
				caldata = packet->getCalData(i);
				mvprintw(y + 5 + temperatureOffset + i * screenSensorOffset, x, 
						"%6.2f\t%6.2f\t%6.2f", 	caldata.m_acc.m_data[0], 
						caldata.m_acc.m_data[1], caldata.m_acc.m_data[2]);
				mvprintw(y + 7 + temperatureOffset + i * screenSensorOffset, x, 
						"%6.2f\t%6.2f\t%6.2f", caldata.m_gyr.m_data[0], 
						caldata.m_gyr.m_data[1], caldata.m_gyr.m_data[2]);
				mvprintw(y + 9 + temperatureOffset + i * screenSensorOffset, x,
						"%6.2f\t%6.2f\t%6.2f",caldata.m_mag.m_data[0], 
						caldata.m_mag.m_data[1], caldata.m_mag.m_data[2]);
				move(y + 13 + temperatureOffset + i * screenSensorOffset, x);
			}

			if ((mode & CMT_OUTPUTMODE_ORIENT) == 0) {
				continue;
			}
			int yt, xt;
			getyx(stdscr, yt, xt);
			switch (settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) {
			case CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
				// Output: quaternion
				qat_data = packet->getOriQuat(i);
				mvprintw(yt++, xt, "%6.3f\t%6.3f\t%6.3f\t%6.3f", qat_data.m_data[0], 
						qat_data.m_data[1], qat_data.m_data[2], qat_data.m_data[3]);
				break;

			case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:
				// Output: Euler
				euler_data = packet->getOriEuler(i);
				mvprintw(yt++, xt, "%6.1f\t%6.1f\t%6.1f", euler_data.m_roll, 
						euler_data.m_pitch, euler_data.m_yaw);
				break;

			case CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX:
				// Output: Cosine Matrix
				matrix_data = packet->getOriMatrix(i);
				mvprintw(yt++, xt, "%6.3f\t%6.3f\t%6.3f", matrix_data.m_data[0][0], 
						matrix_data.m_data[0][1], matrix_data.m_data[0][2]);
				mvprintw(yt++, xt, "%6.3f\t%6.3f\t%6.3f\n", matrix_data.m_data[1][0], 
						matrix_data.m_data[1][1], matrix_data.m_data[1][2]);
				mvprintw(yt++, xt, "%6.3f\t%6.3f\t%6.3f\n", matrix_data.m_data[2][0], 
						matrix_data.m_data[2][1], matrix_data.m_data[2][2]);
				break;
			default:
				break;
			}

			if ((mode & CMT_OUTPUTMODE_POSITION) != 0) {
				yt += 2;
				xt = 0;
				move(yt, xt);
				if (packet->containsPositionLLA()) {
					/* output position */
					CmtVector positionLLA = packet->getPositionLLA();
					if (res != XRV_OK) {
						printw("error %ud", res);
					}
	
					for (int i = 0; i < 2; i++) {
						double deg = positionLLA.m_data[i];
						double min = (deg - (int)deg)*60;
						double sec = (min - (int)min)*60;
						printw("%3d\xb0%2d\'%2.2lf\"\t", (int)deg, (int)min, sec);
					}
					printw(" %3.2lf\n", positionLLA.m_data[2]);
				} else {
					printw("No position data available\n");
				}
			}
			move(yt, xt);
			refresh();
		}
		
		int chr = getch();
		if (chr == 3 || chr == 'q') {
			userQuit = 1;
			// break;
		}
	
	}
	echo();
	endwin();
	delete packet;
	cmt3.closePort();
	return 0;
}

//////////////////////////////////////////////////////////////////////////
// doHardwareScan
//
// Checks available COM ports and scans for MotionTrackers
int doHardwareScan(xsens::Cmt3 &cmt3, CmtDeviceId deviceIds[])
{
	XsensResultValue res;
	List<CmtPortInfo> portInfo;
	unsigned long portCount = 0;
	int mtCount;
	
	printw("Scanning for connected Xsens devices...");
	xsens::cmtScanPorts(portInfo);
	portCount = portInfo.length();
	printw("done\n");

	if (portCount == 0) {
		printw("No MotionTrackers found\n\n");
		return 0;
	}

	for(int i = 0; i < (int)portCount; i++) {	
		printw("Using COM port %s at ", portInfo[i].m_portName);
		
		switch (portInfo[i].m_baudrate) {
		case B9600  : printw("9k6");   break;
		case B19200 : printw("19k2");  break;
		case B38400 : printw("38k4");  break;
		case B57600 : printw("57k6");  break;
		case B115200: printw("115k2"); break;
		case B230400: printw("230k4"); break;
		case B460800: printw("460k8"); break;
		case B921600: printw("921k6"); break;
		default: printw("0x%lx", portInfo[i].m_baudrate);
		}
		printw(" baud\n\n");
	}

	printw("Opening ports...");
	//open the port which the device is connected to and connect at the device's baudrate.
	for(int p = 0; p < (int)portCount; p++){
		res = cmt3.openPort(portInfo[p].m_portName, portInfo[p].m_baudrate);
		EXIT_ON_ERROR(res,"cmtOpenPort");  
	}
	printw("done\n\n");

	 //set the measurement timeout to 100ms (default is 16ms)
	int timeOut = 100;
	res = cmt3.setTimeoutMeasurement(timeOut);
	EXIT_ON_ERROR(res, "set measurment timeout");
	printf("Measurement timeout set to %d ms\n", timeOut);

	//get the Mt sensor count.
	printw("Retrieving MotionTracker count (excluding attached Xbus Master(s))\n");
	mtCount = cmt3.getMtCount();
	printw("MotionTracker count: %d\n\n", mtCount);

	// retrieve the device IDs 
	printw("Retrieving MotionTrackers device ID(s)\n");
	for(int j = 0; j < mtCount; j++){
		res = cmt3.getDeviceId((unsigned char)(j+1), deviceIds[j]);
		EXIT_ON_ERROR(res,"getDeviceId");
		printw("Device ID at busId %i: %08lx\n\n",j+1,(long) deviceIds[j]);
	}
	
	return mtCount;
}

//////////////////////////////////////////////////////////////////////////
// getUserInputs
//
// Request user for output data
void getUserInputs(CmtOutputMode &mode, CmtOutputSettings &settings)
{
	mode = 0;
	int y, x;
	getyx(stdscr, y, x);
	while (mode < 1 || mode > 6) {
		printw("Select desired output:\n");
		printw("1 - Calibrated data\n");
		printw("2 - Orientation data and GPS Position (MTi-G only)\n");
		printw("3 - Both Calibrated and Orientation data\n");
		printw("4 - Temperature and Calibrated data\n");
		printw("5 - Temperature and Orientation data\n");
		printw("6 - Temperature, Calibrated and Orientation data\n");
		printw("Enter your choice: ");
		refresh();
		fflush(stdin);
		while(!isnum(mode = getch()));
		printw("%c", mode);
		mode -= 48;
		refresh();
		fflush(stdin);
		
		move(y,x);
		clrtobot();
		move(y,x);
				
		if (mode < 1 || mode > 6) {
			printw("\n\nPlease enter a valid output mode\n");
		}
	}

	switch(mode)
	{
	case 1:
		mode = CMT_OUTPUTMODE_CALIB;
		break;
	case 2:
		mode = CMT_OUTPUTMODE_ORIENT | CMT_OUTPUTMODE_POSITION;
		break;
	case 3:
		mode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
		break;
	case 4:
		mode = CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_CALIB;
		break;
	case 5:
		mode = CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_ORIENT;
		break;
	case 6:
		mode = CMT_OUTPUTMODE_TEMP | CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
		break;
	}

	if ((mode & CMT_OUTPUTMODE_ORIENT) != 0) {
		settings = 0;
		while (settings < 1 || settings > 3) {
			printw("\nSelect desired output format\n");
			printw("1 - Quaternions\n");
			printw("2 - Euler angles\n");
			printw("3 - Matrix\n");
			printw("Enter your choice: ");
			fflush(stdin);
			refresh();
			while(!isnum(settings = getch()));
			printw("%c", settings);
			settings -= 48;
			refresh();
			move(y,x);
			clrtobot();
			move(y,x);
			fflush(stdin);
			
			if (settings < 1  || settings > 3) {
				printw("\n\nPlease enter a valid output format\n");
			}
		}

		// Update outputSettings to match data specs of SetOutputSettings
		switch(settings) {
		case 1:
			settings = CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION;
			break;
		case 2:
			settings = CMT_OUTPUTSETTINGS_ORIENTMODE_EULER;
			break;
		case 3:
			settings = CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX;
			break;
		}
	} else {
		settings = 0;
	}
	settings |= CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
}

//////////////////////////////////////////////////////////////////////////
// doMTSettings
//
// Set user settings in MTi/MTx
// Assumes initialized global MTComm class
void doMtSettings(xsens::Cmt3 &cmt3, CmtOutputMode &mode, 
		CmtOutputSettings &settings, CmtDeviceId deviceIds[]) 
{
	XsensResultValue res;
	unsigned long mtCount = cmt3.getMtCount();

	// set sensor to config sate
	res = cmt3.gotoConfig();
	EXIT_ON_ERROR(res,"gotoConfig");

	unsigned short sampleFreq;
	sampleFreq = cmt3.getSampleFrequency();

	// set the device output mode for the device(s)
	printw("Configuring your mode selection\n");

	for (unsigned int i = 0; i < mtCount; i++) {
		CmtDeviceMode deviceMode(mode, settings, sampleFreq);
		if ((deviceIds[i] & 0xFFF00000) != 0x00500000) {
			// not an MTi-G, remove all GPS related stuff
			deviceMode.m_outputMode &= 0xFF0F;
		}
		res = cmt3.setDeviceMode(deviceMode, true, deviceIds[i]);
		EXIT_ON_ERROR(res,"setDeviceMode");
	}

	// start receiving data
	res = cmt3.gotoMeasurement();
	EXIT_ON_ERROR(res,"gotoMeasurement");
}

//////////////////////////////////////////////////////////////////////////
// writeHeaders
//
// Write appropriate headers to screen
void writeHeaders(unsigned long mtCount, CmtOutputMode &mode, 
		CmtOutputSettings &settings, int &temperatureOffset, 
		int &screenSensorOffset)
{
	int y, x;
	getyx(stdscr, y, x);
	for (unsigned int i = 0; i < mtCount; i++) {
		mvprintw(y + 2 + i * screenSensorOffset, x, "MotionTracker %d\n", i + 1);
		if ((mode & CMT_OUTPUTMODE_TEMP) != 0) {
			temperatureOffset = 3;
			mvprintw(y + 3 + i * screenSensorOffset, x, "Temperature");
			mvprintw(y + 4 + i * screenSensorOffset, 7, "degrees celcius\n");
			move(y + 6 + i * screenSensorOffset, x);
		}

		if ((mode & CMT_OUTPUTMODE_CALIB) != 0) {
			mvprintw(y + 3 + temperatureOffset + i * screenSensorOffset, x, 
					"Calibrated sensor data");
			mvprintw(y + 4 + temperatureOffset + i * screenSensorOffset, x, 
					" Acc X\t Acc Y\t Acc Z");
			mvprintw(y + 5 + temperatureOffset + i * screenSensorOffset, x + 23, 
					"(m/s^2)");
			mvprintw(y + 6 + temperatureOffset + i * screenSensorOffset, x, 
					" Gyr X\t Gyr Y\t Gyr Z");
			mvprintw(y + 7 + temperatureOffset + i * screenSensorOffset, x + 23, 
					"(rad/s)");
			mvprintw(y + 8 + temperatureOffset + i * screenSensorOffset, x, 
					" Mag X\t Mag Y\t Mag Z");
			mvprintw(y + 9 + temperatureOffset + i * screenSensorOffset, x + 23, 
					"(a.u.)");
			move(y + 11 + temperatureOffset + i * screenSensorOffset, x);
		}

		if ((mode & CMT_OUTPUTMODE_ORIENT) != 0) {
			printw("Orientation data\n");
			switch(settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) {
			case CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
				printw("    q0\t    q1\t    q2\t    q3\n");
				break;
			case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:
				printw("  Roll\t Pitch\t   Yaw\n");
				printw("                       degrees\n");
				break;
			case CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX:
				printw(" Matrix\n");
				break;
			default:
				;
			}			
		}

		if ((mode & CMT_OUTPUTMODE_POSITION) != 0) {
			printw("\nLongitude\tLatitude\t Altitude\n");
		}
	}
	move(y, x);
	refresh();
}

//////////////////////////////////////////////////////////////////////////
// calcScreenOffset
//
// Calculates offset for screen data with multiple MTx on Xbus Master
int calcScreenOffset(CmtOutputMode &mode, CmtOutputSettings &settings, 
		int screenSensorOffset)
{
    // 1 line for "Sensor ..."
    screenSensorOffset += 1;
    if ((mode & CMT_OUTPUTMODE_TEMP) != 0)
        screenSensorOffset += 3;
    if ((mode & CMT_OUTPUTMODE_CALIB) != 0)
        screenSensorOffset += 8;
    if ((mode & CMT_OUTPUTMODE_ORIENT) != 0) {
        switch(settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK) {
        case CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
            screenSensorOffset += 4;
            break;
        case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:
            screenSensorOffset += 4;
            break;
        case CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX:
            screenSensorOffset += 6;
            break;
        default:
            ;
        }
    }

	if ((mode & CMT_OUTPUTMODE_POSITION) != 0) {
		screenSensorOffset += 4;
	}
	return screenSensorOffset;
}

