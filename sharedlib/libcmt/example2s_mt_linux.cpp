/*! \file example2s_XM_linux.cpp
    \brief Example file for CMT level 2 serial branch.
    
    This file contains an example of the CMT level 2 serial interface.
*/

#include "cmt2.h"
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>

int quit = 0;

void ctrlchandler(int sig)
{
    
    // got ctrl-c
    quit = 1;
}

using namespace xsens;

#define EXIT_ERROR(loc) {printf("Error %d occurred during " loc ": %s\n", serial.getLastResult(), xsensResultText(serial.getLastResult())); exit(-1); }

int main(void)
{
    Cmt2s serial;
    Message msg, reply;
    (void) signal(SIGINT, ctrlchandler);
    printf( "This example will connect to the MT, configure it for euler output at 100Hz and\n"
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

    if (serial.open(portname, B115200) != XRV_OK)
        EXIT_ERROR("open");
	
    msg.setMessageId(CMT_MID_GOTOCONFIG);
    printf("Putting MT in config mode\n");
    if (serial.writeMessage(&msg))
        EXIT_ERROR("goto config");
    if (serial.waitForMessage(&reply, CMT_MID_GOTOCONFIGACK, 0,  1) != XRV_OK)
        EXIT_ERROR("goto config ack");
    printf("MT now in config mode\n");

    msg.setMessageId(CMT_MID_SETPERIOD);
    msg.setDataShort(1152);
    if (serial.writeMessage(&msg))
        EXIT_ERROR("set period");
    if (serial.waitForMessage(&reply, CMT_MID_SETPERIODACK, 0,  1) != XRV_OK)
        EXIT_ERROR("set period ack");
    printf("Period is now set to 100Hz\n");

    msg.setMessageId(CMT_MID_SETOUTPUTMODE);
    msg.setDataShort(CMT_OUTPUTMODE_ORIENT);
    if (serial.writeMessage(&msg))
        EXIT_ERROR("set output mode");
    if (serial.waitForMessage(&reply, CMT_MID_SETOUTPUTMODEACK, 0,  1) != XRV_OK)
        EXIT_ERROR("set output mode ack");
    printf("Output mode is now set to orientation\n");

    msg.setMessageId(CMT_MID_SETOUTPUTSETTINGS);
    msg.setDataLong(CMT_OUTPUTSETTINGS_ORIENTMODE_EULER | CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT);
    if (serial.writeMessage(&msg))
        EXIT_ERROR("set output settings");
    if (serial.waitForMessage(&reply, CMT_MID_SETOUTPUTSETTINGSACK, 0,  1) != XRV_OK)
        EXIT_ERROR("set output settings ack");
    printf("Output settings now set to euler + timestamp\n");

    msg.setMessageId(CMT_MID_GOTOMEASUREMENT);
    msg.resizeData(0);
    if (serial.writeMessage(&msg))
        EXIT_ERROR("goto measurement");
    if (serial.waitForMessage(&reply, CMT_MID_GOTOMEASUREMENTACK, 0,  1) != XRV_OK)
        EXIT_ERROR("goto measurement ack");
    printf("Now in measurement mode, Time of day: %u\n", getTimeOfDay());

    long msgCount = 0;
    while (!quit)
    {
        if (serial.waitForMessage(&reply, 0, 0, 1) != XRV_OK)
            EXIT_ERROR("read data message");

         msgCount++;
         printf("%3ld  SC: %hu  TS: %u  Roll %+6.1f  Pitch %+6.1f  Yaw %+6.1f         \r",
             msgCount,
             reply.getDataShort(3*4),           // sample counter
             getTimeOfDay(),                     // timestamp
             (double) reply.getDataFloat(0*4),  // roll
             (double) reply.getDataFloat(1*4),  // pitch
             (double) reply.getDataFloat(2*4)   // yaw
             );
    }

    printf("\n\nDone reading\n");
    (void) signal(SIGINT, SIG_DFL);
    return 0;
}
