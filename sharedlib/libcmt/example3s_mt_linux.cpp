/*! \file example3s_XM.cpp
    \brief Example file for CMT level 3 serial branch.
    
    This file contains an example of the CMT level 3 serial interface.
*/

#include "cmt3.h"
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>

int quit = 0;

void exitFunc(void)
{
    (void) signal(SIGINT, SIG_DFL);
}

void ctrlchandler(int sig)
{
    // got ctrl-c
    quit = 1;
}

using namespace xsens;

#define EXIT_ERROR(loc) {printf("Error %d occurred during " loc ": %s\n", serial.getLastResult(), xsensResultText(serial.getLastResult())); exit(-1); }

int main(void)
{
    Cmt3 serial;
    Packet reply(1,0); /* 1 item, not xbus */
    (void) signal(SIGINT, ctrlchandler);
    atexit(exitFunc);
    
    printf( "This example will connect to the MT, configure all sensors for euler output at 100Hz and\n"
            "read data messages until Control-C is pressed\n");
	printf(	"Please be aware that this example has a hard-coded baud rate of 115200!\n");
    
    char portname[32];
    printf("Enter the port name you wish to connect to (default: /dev/ttyUSB0): ");
	fgets(portname, 32, stdin);
	char *newlinePtr = strchr(portname, '\n');
	if(newlinePtr != 0)
		*newlinePtr = '\0';

    if (strlen(portname) == 0)
        sprintf(portname, "/dev/ttyUSB0");

    if (serial.openPort(portname, B115200) != XRV_OK)
        EXIT_ERROR("open");
    printf("MT now in config mode\n");
    
	 //set the measurement timeout to 100ms (default is 16ms)
	int timeOut = 100;
	if (serial.setTimeoutMeasurement(timeOut) != XRV_OK)
		EXIT_ERROR("set timeout");
	printf("Measurement timeout set to %d ms\n", timeOut);
	
    CmtDeviceMode mode(CMT_OUTPUTMODE_ORIENT, CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT | CMT_OUTPUTSETTINGS_ORIENTMODE_EULER, 100);
    if (serial.setDeviceMode(mode, false, CMT_DID_BROADCAST))
        EXIT_ERROR("set device mode");
    printf("Device modes set\n");

    if (serial.gotoMeasurement())
        EXIT_ERROR("goto measurement");
    printf("Now in measurement mode\n");

    long msgCount = 0;
    while (!quit)
    {
        if (serial.waitForDataMessage(&reply) != XRV_OK)
        {
            printf("\n");
            EXIT_ERROR("read data message");
        }

        msgCount++;
        printf("%3ld SC: %hu TOA: %llu TS: %u Roll %+6.1f Pitch %+6.1f Yaw %+6.1f       \r",
            msgCount,
            reply.getSampleCounter(),              // sample counter
            (long long unsigned)reply.m_rtc,       // Time Of Arrival / Real-Time Clock
            getTimeOfDay(),                         // timestamp
            (double) reply.getOriEuler().m_roll,     // roll
            (double) reply.getOriEuler().m_pitch,    // pitch
            (double) reply.getOriEuler().m_yaw       // yaw
            );

    }

    printf("\n\nDone reading\n");
    return 0;
}
