/*
 * Atomic6DOF.cc
 *
 *  Created on: Feb 18, 2010
 *      Author: uscr
 */

#include "Atomic6DOF.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <termio.h>
#include <sys/fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#define MAX_SERIAL_BUF_LENGTH		1024
int serialCon;
struct currentIMUData curIMUData;
unsigned char serialBuf[MAX_SERIAL_BUF_LENGTH];
int serialBufLength = 0;

bool initializeAtomic6DOF() {

	memset(serialBuf, 0, MAX_SERIAL_BUF_LENGTH);

	if ((serialCon = open (SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
		return false;

	struct termios attr;

	if (tcgetattr(serialCon, &attr) < 0)
		return false;

	/* 8 bits + baud rate + local control */
	attr.c_cflag = SERIAL_BAUD|CLOCAL|CREAD;
	attr.c_cflag &= ~CRTSCTS;

    attr.c_iflag = IXOFF;
    attr.c_iflag |= ~( IGNBRK | BRKINT | ISTRIP | IGNCR | ICRNL | IXON | INLCR | PARMRK);
	attr.c_oflag = 0;
	attr.c_lflag = 0;
	/* set output and input baud rates */
	cfsetospeed(&attr, SERIAL_BAUD);
	cfsetispeed(&attr, SERIAL_BAUD);
	if (tcsetattr(serialCon, TCSANOW, &attr) < 0)
		return false;

	if (write(serialCon, "*#", 2) != 2)
		return false;

	return true;

}

void shutdownAtomic6DOF() {
	close(serialCon);

}

bool isIMUDataReady() {

	unsigned char buffer[801];
	int numRead = read(serialCon, buffer, 800);

	//if we didn't read anything then don't continue
	if (numRead < 1)
		return false;

	//copy what we read into our long-term buffer
	memcpy(serialBuf + serialBufLength, buffer, numRead);
	serialBufLength += numRead;

	//if we have more than two packets in our buffer, then
	// we should disregard our first ones and look at the last
	if (serialBufLength > 32) {
		int lastAPos = serialBufLength - 1;

		//find the position of the last "Z"
		while (lastAPos >= 0 && serialBuf[lastAPos] != 'Z') {lastAPos--;}
		//find the start of the last complete packet
		while (lastAPos >= 0 && serialBuf[lastAPos] != 'A') {lastAPos--;}

		unsigned char tempBuf[32];
		serialBufLength -= lastAPos;
		if (serialBufLength <= 32) {
			memcpy (tempBuf, serialBuf + lastAPos, serialBufLength);
			memcpy (serialBuf, tempBuf, serialBufLength);
			memset (serialBuf + serialBufLength, 0, MAX_SERIAL_BUF_LENGTH - serialBufLength);
		}
	}

	//if we still don't have enough to form a complete packet
	// or if our packet is bad, then don't continue
	if (serialBufLength < 16 || serialBuf[0] != 'A' || serialBuf[15] != 'Z')
		return false;

//	printf("%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X\n",
//			serialBuf[0], serialBuf[1], serialBuf[2], serialBuf[3], serialBuf[4], serialBuf[5],
//			serialBuf[6], serialBuf[7], serialBuf[8], serialBuf[9], serialBuf[10], serialBuf[11],
//			serialBuf[12], serialBuf[13], serialBuf[14], serialBuf[15]);
	//throw the data into the currentIMUData structure
	curIMUData.count = (double) (( ((serialBuf[1]&0x7F) << 8) | (serialBuf[2]&0xFF) ) & 0x7FFF);
	curIMUData.accel_x = (double) (( ((serialBuf[3]&0x3) << 8) | (serialBuf[4]&0xFF) ) & 0x3FF);
	curIMUData.accel_y = (double) (( ((serialBuf[5]&0x3) << 8) | (serialBuf[6]&0xFF) ) & 0x3FF);
	curIMUData.accel_z = (double) (( ((serialBuf[7]&0x3) << 8) | (serialBuf[8]&0xFF) ) & 0x3FF);
	curIMUData.gyro_pitch = (double) (( ((serialBuf[9]&0x3) << 8) | (serialBuf[10]&0xFF) ) & 0x3FF);
	curIMUData.gyro_roll = (double) (( ((serialBuf[11]&0x3) << 8) | (serialBuf[12]&0xFF) ) & 0x3FF);
	curIMUData.gyro_yaw = (double) (( ((serialBuf[13]&0x3) << 8) | (serialBuf[14]&0xFF) ) & 0x3FF);

	//adjust the data (i.e., accel raw -> angle  and  gyro raw -> angular velocity)
	curIMUData.adj_accel_x = curIMUData.accel_x*X_ACCEL_COEFF - X_ACCEL_OFFSET;
	curIMUData.adj_accel_y = curIMUData.accel_y*Y_ACCEL_COEFF - Y_ACCEL_OFFSET;
	curIMUData.adj_accel_z = curIMUData.accel_z*Z_ACCEL_COEFF - Z_ACCEL_OFFSET;
	curIMUData.adj_gyro_pitch = curIMUData.gyro_pitch*GYRO_COEFF - GYRO_OFFSET;
	curIMUData.adj_gyro_roll = curIMUData.gyro_roll*GYRO_COEFF - GYRO_OFFSET;
	curIMUData.adj_gyro_yaw = curIMUData.gyro_yaw*GYRO_COEFF - GYRO_OFFSET;
	curIMUData.adj_accel_pitch = asin(curIMUData.adj_accel_y)*RAD_TO_DEG;
	if (isnan(curIMUData.adj_accel_pitch))
		curIMUData.adj_accel_pitch = (curIMUData.adj_accel_y < 0 ? -1 : 1) * 90.0;
	curIMUData.adj_accel_roll = asin(curIMUData.adj_accel_x)*RAD_TO_DEG;
	if (isnan(curIMUData.adj_accel_roll))
		curIMUData.adj_accel_roll = (curIMUData.adj_accel_x < 0 ? -1 : 1) * 90.0;


	//shift the data over in the long-term buffer so we get new data next time
	serialBufLength -= 16;
	if (serialBufLength > 0) {
		for (int i = 0; i < serialBufLength; i++)
			serialBuf[i] = serialBuf[i + 16];
	}

	//we need this because of the occasional bad packet :*(
	memset(serialBuf + serialBufLength, 0, MAX_SERIAL_BUF_LENGTH - serialBufLength);

	return true;
}
