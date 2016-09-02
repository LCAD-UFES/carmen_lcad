/****************************************************************************
 *
 * $Id: vpSickLDMRS.cpp 4056 2013-01-05 13:04:42Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Sick LD-MRS laser driver on UNIX platform.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/


#include <sickldmrs.h>
#include <vpMath.h>
#include <vpTime.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <errno.h>
#include <netdb.h>
#include <string.h>
#include <strings.h>
#include <math.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>


/*!

  \file vpSickLDMRS.cpp

  \brief Driver for the Sick LD-MRS laser scanner. 
 */

/*! 

  Default constructor that initialize the Ethernet address to
  "131.254.12.119", set the port to 12002 and allocates memory for the
  body messages.
 */
vpSickLDMRS::vpSickLDMRS()
{
	ip = "192.168.0.104";
	port = 12002;
	body = new unsigned char [104000];
	isFirstMeasure = true;
	time_offset = 0;

	vAngle.resize(4); // Vertical angle of the 4 layers
	vAngle[0] = vpMath::rad(-1.2);
	vAngle[1] = vpMath::rad(-0.4);
	vAngle[2] = vpMath::rad( 0.4);
	vAngle[3] = vpMath::rad( 1.2);

}

/*!
  Destructor that deallocate the memory for the body messages.
 */
vpSickLDMRS::~vpSickLDMRS()
{
	if (body)
		delete [] body;
}

/*! 
  Initialize the connexion with the Sick LD-MRS laser scanner.

  \param ip : Ethernet address of the laser.
  \param port : Ethernet port of the laser.

  \return true if the device was initialized, false otherwise.

 */
bool vpSickLDMRS::setup(std::string ip, int port)
{
	setIpAddress( ip );
	setPort( port );
	return ( this->setup() );
}

/*! 
  Initialize the connexion with the Sick LD-MRS laser scanner.

  \return true if the device was initialized, false otherwise.
 */
bool vpSickLDMRS::setup()
{
	struct sockaddr_in serv_addr;
	int res;
	struct timeval tv;
	fd_set myset;

	// Create the TCP socket
	socket_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (socket_fd < 0) {
		fprintf(stderr, "Failed to create socket\n");
		return false;
	}
	bzero(&serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;                     // Internet/IP
	serv_addr.sin_addr.s_addr = inet_addr(ip.c_str());  // IP address
	serv_addr.sin_port = htons(port);                   // server port

	// Establish connection
	res = connect(socket_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) ;
	if (errno == EINPROGRESS) {
		tv.tv_sec = 3;
		tv.tv_usec = 0;
		FD_ZERO(&myset);
		FD_SET(static_cast<unsigned int>(socket_fd), &myset);
		res = select(socket_fd+1, NULL, &myset, NULL, &tv);
		if (res < 0 && errno != EINTR) {
			fprintf(stderr, "Error connecting to server %d - %s\n", errno, strerror(errno));
			return false;
		}
		else if (res > 0) {
			fprintf(stderr,"ok");
		}
		else {
			fprintf(stderr, "Timeout in select() - Cancelling!\n");
			return false;
		}
	}

	return true;
}


unsigned int readUValueLE(unsigned char* buffer, unsigned char bytes)
{
	unsigned int value;

	switch (bytes)
	{
	case 1:
		value = buffer[0];
		break;
	case 2:
		value = buffer[0];
		value += ((unsigned int)buffer[1]) << 8;
		break;
	case 4:
		value = buffer[0];
		value += ((unsigned int)buffer[1]) << 8;
		value += ((unsigned int)buffer[2]) << 16;
		value += ((unsigned int)buffer[3]) << 24;
		break;
	default:
		value = 0xFFFFFFFF;
	}

	return value;
}


// decode scan data
void vpSickLDMRS::decodeScanData(vpLaserScan laserscan[4])
{
	unsigned int *uintptr;
	unsigned short *ushortptr;

	double time_second = 0;

	if (isFirstMeasure) {
		time_second = vpTime::measureTimeSecond();
	}

	// get the measurement number
	unsigned short measurementId;
	ushortptr = (unsigned short *) body;
	measurementId = ushortptr[0];

	// get the start timestamp
	uintptr=(unsigned int *) (body+6);
	unsigned int seconds = uintptr[1];
	unsigned int fractional=uintptr[0];
	double startTimestamp = seconds + fractional / 4294967296.; // 4294967296. = 2^32

	// get the end timestamp
	uintptr=(unsigned int *) (body+14);
	seconds = uintptr[1];
	fractional=uintptr[0];
	double endTimestamp = seconds + fractional / 4294967296.; // 4294967296. = 2^32

	// compute the time offset to bring the measures in the Unix time reference
	if (isFirstMeasure) {
		time_offset = time_second - startTimestamp;
		isFirstMeasure = false;
	}

	startTimestamp += time_offset;
	endTimestamp += time_offset;

	// get the number of steps per scanner rotation
	unsigned short numSteps = ushortptr[11];

	// get the start/stop angle
	short startAngle = (short)ushortptr[12];
	short stopAngle = (short)ushortptr[13];
	//   std::cout << "angle in [" << startAngle << "; " << stopAngle
	// 	    << "]" << std::endl;

	// get the number of points of this measurement
	unsigned short numPoints = ushortptr[14];

	int nlayers = 4;
	for (int i=0; i < nlayers; i++) {
		laserscan[i].clear();
		laserscan[i].setMeasurementId(measurementId);
		laserscan[i].setStartTimestamp(startTimestamp);
		laserscan[i].setEndTimestamp(endTimestamp);
		laserscan[i].setNumSteps(numSteps);
		laserscan[i].setStartAngle(startAngle);
		laserscan[i].setStopAngle(stopAngle);
		laserscan[i].setNumPoints(numPoints);
	}

	// decode the measured points
	double hAngle; // horizontal angle in rad
	double rDist; // radial distance in meters
	vpScanPoint scanPoint;

	for (int i=0; i < numPoints; i++) {
		ushortptr = (unsigned short *) (body+44+i*10);
		unsigned char layer = ((unsigned char)  body[44+i*10])&0x0F;
		unsigned char echo  = ((unsigned char)  body[44+i*10])>>4;
		unsigned char flags = (unsigned char)  body[44+i*10+1];
		scanPoint.setFlags(flags);
		if (echo==0) {
			hAngle = (2.f * M_PI / numSteps)*(short) ushortptr[1];
			rDist = 0.01 * ushortptr[2]; // cm to meters conversion

			//vpTRACE("layer: %d d: %f hangle: %f", layer, rDist, hAngle);
			scanPoint.setPolar(rDist, hAngle, vAngle[layer]);
			laserscan[layer].addPoint(scanPoint);
		}
	}
}

// decode the objects data
void vpSickLDMRS::decodeObjectsData(vpLaserObjectData *objectData)
{
	// get the start timestamp

	unsigned int *uintptr;

	uintptr=(unsigned int *) body;
	unsigned int seconds = uintptr[1];
	unsigned int fractional=uintptr[0];
	double startTimestamp = seconds + fractional / 4294967296.; // 4294967296. = 2^32

	unsigned int offset = 8;
	// get number of objects
	// ushortptr = (unsigned short *) (body + 8);
	unsigned short numObjects = (unsigned short) readUValueLE(&(body[offset]), 2);
	objectData->setNumObjects(numObjects);
	objectData->setStartTimestamp(startTimestamp);

	offset += 2;

	// decode objects
	for (int i=0; i < numObjects; i++) {

		vpObject objectContent;

		unsigned short objectID = (unsigned short) readUValueLE(&(body[offset]), 2);
		offset += 2;

		unsigned short objectAge = (unsigned short) readUValueLE(&(body[offset]), 2);
		offset += 2;

		unsigned short objectPredictionAge = (unsigned short) readUValueLE(&(body[offset]), 2);
		offset += 2;

		unsigned short relativeTimestamp = (unsigned short) readUValueLE(&(body[offset]), 2);
		offset += 2;

		point_2d referencePoint;
		referencePoint.x_pos = (short) readUValueLE(&(body[offset]), 2);
		offset += 2;

		referencePoint.y_pos = (short) readUValueLE(&(body[offset]), 2);
		offset += 2;

		point_2d referencePointSigma;
		referencePointSigma.x_pos = (short) readUValueLE(&(body[offset]), 2);
		offset += 2;

		referencePointSigma.y_pos = (short) readUValueLE(&(body[offset]), 2);
		offset += 2;

		point_2d closestPoint;
		closestPoint.x_pos = (short) readUValueLE(&(body[offset]), 2);
		offset += 2;
		closestPoint.y_pos = (short) readUValueLE(&(body[offset]), 2);
		offset += 2;

		point_2d boundingBoxCenter;
		boundingBoxCenter.x_pos = (short) readUValueLE(&(body[offset]), 2);
		offset += 2;
		boundingBoxCenter.y_pos = (short) readUValueLE(&(body[offset]), 2);
		offset += 2;

		size_2d boundingBoxSize;
		boundingBoxSize.x_size = (unsigned short) readUValueLE(&(body[offset]), 2);
		offset += 2;
		boundingBoxSize.y_size = (unsigned short) readUValueLE(&(body[offset]), 2);
		offset += 2;

		point_2d objectBoxCenter;
		objectBoxCenter.x_pos = (short) readUValueLE(&(body[offset]), 2);
		offset += 2;
		objectBoxCenter.y_pos = (short) readUValueLE(&(body[offset]), 2);
		offset += 2;

		size_2d objectBoxSize;
		objectBoxSize.x_size = (unsigned short) readUValueLE(&(body[offset]), 2);
		offset += 2;
		objectBoxSize.y_size = (unsigned short) readUValueLE(&(body[offset]), 2);
		offset += 2;

		short objectBoxOrientation = (short) readUValueLE(&(body[offset]), 2);
		offset += 2;

		point_2d absoluteVelocity;
		absoluteVelocity.x_pos = (short) readUValueLE(&(body[offset]), 2);
		offset += 2;
		absoluteVelocity.y_pos = (short) readUValueLE(&(body[offset]), 2);
		offset += 2;

		size_2d absoluteVelocitySigma;
		absoluteVelocitySigma.x_size = (unsigned short) readUValueLE(&(body[offset]), 2);
		offset += 2;
		absoluteVelocitySigma.y_size = (unsigned short) readUValueLE(&(body[offset]), 2);
		offset += 2;

		point_2d relativeVelocity;
		relativeVelocity.x_pos = (short) readUValueLE(&(body[offset]), 2);
		offset += 2;
		relativeVelocity.y_pos = (short) readUValueLE(&(body[offset]), 2);
		offset += 2;

		//reserved Class
		unsigned short classification = (unsigned short) readUValueLE(&(body[offset]), 2);
		offset += 2;

		// reserved
		offset += 2;

		// reserved
		offset += 2;

		unsigned short numContourPoints = (unsigned short) readUValueLE(&(body[offset]), 2);
		offset += 2;

		objectContent.setObjectId(objectID);
		objectContent.setObjectAge(objectAge);
		objectContent.setObjectPredictionAge(objectPredictionAge);
		objectContent.setRelativeTimestamp(relativeTimestamp);
		objectContent.setReferencePoint(referencePoint);
		objectContent.setReferencePointSigma(referencePointSigma);
		objectContent.setClosestPoint(closestPoint);
		objectContent.setBoundingBoxCenter(boundingBoxCenter);
		objectContent.setBoundingBoxSize(boundingBoxSize);
		objectContent.setObjectBoxCenter(objectBoxCenter);
		objectContent.setObjectBoxSize(objectBoxSize);
		objectContent.setObjectBoxOrientation(objectBoxOrientation);

		if (absoluteVelocity.x_pos < -320.0)
			objectContent.setAbsoluteVelocity(relativeVelocity);
		else
			objectContent.setAbsoluteVelocity(absoluteVelocity);

		objectContent.setAbsoluteVelocitySigma(absoluteVelocitySigma);
		objectContent.setRelativeVelocity(relativeVelocity);
		objectContent.setClassification(classification);
		objectContent.setNumContourPoints(numContourPoints);

		for(int j = 0; j < numContourPoints; j ++) {
			point_2d contourPoint;
			contourPoint.x_pos = (short) readUValueLE(&(body[offset]), 2);
			offset += 2;
			contourPoint.y_pos = (short) readUValueLE(&(body[offset]), 2);
			offset += 2;

			objectContent.addContourPoint(contourPoint);
		}
		objectData->addObject(objectContent);
	}

}

// this function decode the error messages from device
void vpSickLDMRS::decodeErrorData()
{

	// decode message
	unsigned short errorRegister1 = (unsigned short) readUValueLE(&(body[0]), 2);
	unsigned short errorRegister2 = (unsigned short) readUValueLE(&(body[2]), 2);
	unsigned short warningRegister1 = (unsigned short) readUValueLE(&(body[4]), 2);
	unsigned short warningRegister2 = (unsigned short) readUValueLE(&(body[6]), 2);

	// print FPGA error messages
	if((errorRegister1 & 0x0003) != 0)
	{
		printf("FPGA Error: Contact Support.\n");
	}
	if((errorRegister1 & 0x000C) != 0)
	{
		printf("FPGA Error: decrease scan resolution/frequency/range.\n");
	}
	if((errorRegister1 & 0x0010) != 0)
	{
		printf("FPGA Error: Contact Support.\n");
	}
	if((errorRegister1 & 0x0100) != 0)
	{
		printf("FPGA Error: APD Under Temperature, provide heating.\n");
	}
	if((errorRegister1 & 0x0200) != 0)
	{
		printf("FPGA Error: APD Over Temperature, provide cooling.\n");
	}
	if((errorRegister1 & 0x3C00) != 0)
	{
		printf("FPGA Error: Contact support.\n");
	}

	// processor errors
	if((errorRegister2 & 0x0001) != 0)
	{
		printf("DSP Error: Internal communication error. The DSP did not receive any scan data from the FPGA.\n");
	}
	if((errorRegister2 & 0x0002) != 0)
	{
		printf("DSP Error: Internal communication error. The DSP could not communicate correctly with the FPGA via the control interface.\n");
	}
	if((errorRegister2 & 0x0004) != 0)
	{
		printf("DSP Error: Internal communication error. The DSP did not receive valid scan data from the FPGA for more than 500 ms.\n");
	}
	if((errorRegister2 & 0x0008) != 0)
	{
		printf("DSP Error: Contact support.\n");
	}
	if((errorRegister2 & 0x0010) != 0)
	{
		printf("DSP Error: Incorrect configuration data, load correct configuration values.\n");
	}
	if((errorRegister2 & 0x0020) != 0)
	{
		printf("DSP Error: Configuration contains incorrect parameters, load correct configuration values.\n");
	}
	if((errorRegister2 & 0x0040) != 0)
	{
		printf("DSP Error: Data processing timeout, decrease scan resolution or scan frequency.\n");
	}
	if((errorRegister2 & 0x0200) != 0)
	{
		printf("DSP Error: Severe deviation (> 10\%%) from expected scan frequency. This may indicate motor trouble.\n");
	}
	if((errorRegister2 & 0x0400) != 0)
	{
		printf("DSP Error: Motor blocked. No rotation of the internal mirror was detected, and automatic restart has failed.\n");
	}

	// fpga warningg
	if((warningRegister1 & 0x0008) != 0)
	{
		printf("FPGA Warning: Low temperature.\n");
	}
	if((warningRegister1 & 0x0010) != 0)
	{
		printf("FPGA Warning: High temperature.\n");
	}

	// dsp warningg
	if((warningRegister2 & 0x0040) != 0)
	{
		printf("DSP Warning: Memory access failure, restart LD-MRS, contact support.\n");
	}
	if((warningRegister2 & 0x0080) != 0)
	{
		printf("DSP Warning: Segment overflow.\n");
	}
	if((warningRegister2 & 0x0100) != 0)
	{
		printf("DSP Warning: Invalid Ego Motion data.\n");
	}
	if((warningRegister2 & 0x8000) != 0)
	{
		printf("DSP Warning: High temperature.\n");
	}

}

// read data sent from sensor and decode it
unsigned short vpSickLDMRS::readData(vpLaserScan laserscan[4], vpLaserObjectData *objectData)
{
	unsigned int *uintptr;
	unsigned short *ushortptr;
	unsigned short dataType;
	static unsigned char header[24];
	ushortptr=(unsigned short *)header;
	uintptr=(unsigned int *)header;

	assert (sizeof(header) == 24);
	//std::cout << "size " << sizeof(header) << std::endl;

	// read the 24 bytes header
	if (recv(socket_fd, header, sizeof(header), MSG_WAITALL) == -1) {
		printf("recv\n");
		perror("recv");
		return false;
	}

	if (ntohl(uintptr[0]) != vpSickLDMRS::MagicWordC2) {
		printf("Error, wrong magic number !!!\n");
		return false;
	}

	// get the message body
	uint16_t msgtype = ntohs(ushortptr[7]);
	uint32_t msgLength = ntohl(uintptr[2]);

	ssize_t len = recv(socket_fd, body, msgLength, MSG_WAITALL);
	if (len != (ssize_t)msgLength){
		printf("Error, wrong msg length: %d of %d bytes.\n", (int)len, msgLength);
		return false;
	}

	switch (msgtype)
	{
		case vpSickLDMRS::MeasuredData:
			dataType = vpSickLDMRS::MeasuredData;
			decodeScanData(laserscan);
			break;
		case vpSickLDMRS::ObjectData:
			dataType = vpSickLDMRS::ObjectData;
			decodeObjectsData(objectData);
			break;
		case vpSickLDMRS::ErrorData:
			dataType = vpSickLDMRS::ErrorData;
			decodeErrorData();
			break;
		default:
			dataType = 0;
			break;
	}

	return dataType;
}

bool vpSickLDMRS::sendEgoMotionData(short velocity, short steeringWheelAngle, short yawRate)
{
	//message that will be sent to the laser
	unsigned char motion_data[34];

	/*
	 * inserting the header part of the message
	 */

	// magic word
	motion_data[0] = 0xAF;
	motion_data[1] = 0xFE;
	motion_data[2] = 0xC0;
	motion_data[3] = 0xC2;

	// size of previous message
	motion_data[4] = 0x00;
	motion_data[5] = 0x00;
	motion_data[6] = 0x00;
	motion_data[7] = 0x00;

	// size of this message
	motion_data[8] = 0x00;
	motion_data[9] = 0x00;
	motion_data[10] = 0x00;
	motion_data[11] = 0x0A;

	// reserved
	motion_data[12] = 0x00;

	// device ID
	motion_data[13] = 0x00;

	// data type
	motion_data[14] = 0x28;
	motion_data[15] = 0x50;

	// NTP timestamp
	motion_data[16] = 0x00;
	motion_data[17] = 0x00;
	motion_data[18] = 0x00;
	motion_data[19] = 0x00;
	motion_data[20] = 0x00;
	motion_data[21] = 0x00;
	motion_data[22] = 0x00;
	motion_data[23] = 0x00;

	/*
	 * inserting the body of the message
	 */
	// version
	motion_data[24] = 0x01;
	motion_data[25] = 0x00;

	// velocity
	motion_data[26] = (unsigned char) (velocity & 0x00FF);
	motion_data[27] = (unsigned char) ((velocity & 0xFF00) >> 8);

	// reserved
	motion_data[28] = 0x00;
	motion_data[29] = 0x00;

	// steering wheel angle
	motion_data[30] = (unsigned char) (steeringWheelAngle & 0x00FF);
	motion_data[31] = (unsigned char) ((steeringWheelAngle & 0xFF00) >> 8);

	// yaw rate
	motion_data[32] = (unsigned char) (yawRate & 0x00FF);
	motion_data[33] = (unsigned char) ((yawRate & 0xFF00) >> 8);

	//sending the message to the laser
	if(send(socket_fd, motion_data, 34, 0) != 34)
	{
		printf("Failed to send data to socket.\n");
		perror("Failed to send data to socket.");
		return false;
	}

	return true;
}
