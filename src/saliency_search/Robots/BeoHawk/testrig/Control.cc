/*
 * control.cc
 *
 *  Created on: Feb 26, 2010
 *      Author: uscr
 */


#include "Control.h"

#include <cstring>
#include <cstdlib>
#include <cmath>
#include <termio.h>
#include <sys/fcntl.h>
#include <sys/file.h>
#include <unistd.h>

int propSerialCon;

bool initializeController () {

	//=========================================================================
	//Initialize the serial conn to the Propeller
	//=========================================================================
	if ((propSerialCon = open (PROP_SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
		return false;

	struct termios attr;

	if (tcgetattr(propSerialCon, &attr) < 0)
		return false;

	/* 8 bits + baud rate + local control */
	attr.c_cflag = PROP_SERIAL_BAUD|CLOCAL|CREAD;
	attr.c_cflag &= ~CRTSCTS;

    attr.c_iflag = IXOFF;
    attr.c_iflag |= ~( IGNBRK | BRKINT | ISTRIP | IGNCR | ICRNL | IXON | INLCR | PARMRK);
	attr.c_oflag = 0;
	attr.c_lflag = 0;
	/* set output and input baud rates */
	cfsetospeed(&attr, PROP_SERIAL_BAUD);
	cfsetispeed(&attr, PROP_SERIAL_BAUD);
	if (tcsetattr(propSerialCon, TCSANOW, &attr) < 0)
		return false;

	return true;
}

void setMotors (unsigned char front, unsigned char rear, unsigned char left, unsigned char right) {
	char buf[6] = {'A', front, rear, left, right, 'Z'};
	write(propSerialCon, buf, 6);
}

void shutdownController() {
	close(propSerialCon);

}
