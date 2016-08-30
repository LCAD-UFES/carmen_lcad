/****************************************************************************
 *
 * $Id: vpSickLDMRS.h 4056 2013-01-05 13:04:42Z fspindle $
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
 * Sick LD-MRS laser driver.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#ifndef vpSickLDMRS_h
#define vpSickLDMRS_h



#include <arpa/inet.h>
#include <iostream>
#include <vector>


#include <vpScanPoint.h>
#include <vpLaserScan.h>
#include <vpLaserScanner.h>
#include <vpLaserObjectData.h>

/**
 * Human readable measure point
 */
struct sickldmrs_point {
	uint8_t layer;
	uint8_t echo;
	uint8_t flags;
	float horizontal_angle;
	float radial_distance;
	float pulse_width;
};

/**
 * Scan data frames in human readable form
 */
struct sickldmrs_scan {
	uint16_t scan_number;
	uint16_t scanner_status;
	uint16_t sync_phase_offset;
	struct timespec scan_start_time;
	struct timespec scan_end_time;
	uint16_t angle_ticks_per_rotation;
	float start_angle;
	float end_angle;
	uint16_t scan_points;
	float mount_yaw;
	float mount_pitch;
	float mount_roll;
	float mount_x;
	float mount_y;
	float mount_z;
	uint16_t flags;
	struct sickldmrs_point points[];
};



class /*VISP_EXPORT*/ vpSickLDMRS : public vpLaserScanner
{

public:

	enum MagicWord {
		MagicWordC2 = 0xAFFEC0C2   ///< The magic word that allows to identify the messages that are sent by the Sick LD-MRS.
	};
	enum DataType {
		MeasuredData = 0x2202,      ///< Flag to indicate that the body of a message contains measured data.
		ObjectData = 0x2221,
		EgoMotionData = 0x2850,
		ErrorData = 0x2030
	};

	vpSickLDMRS();

	/*! Copy constructor. */
	vpSickLDMRS(const vpSickLDMRS &sick) : vpLaserScanner(sick) {
		socket_fd = sick.socket_fd;
		body = new unsigned char [104000];
	};

	virtual ~vpSickLDMRS();
	bool setup(std::string ip, int port);
	bool setup();
	bool measure(vpLaserScan laserscan[4]);
	bool tracking(vpLaserObjectData *objectData);
	bool sendEgoMotionData(short velocity, short steeringWheelAngle, short yawRate);
	bool getErrors();

protected:

	int socket_fd;

private:
	unsigned char *body;
	std::vector<float> vAngle; // constant vertical angle for each layer
	double time_offset;
	bool isFirstMeasure;
};


#endif
