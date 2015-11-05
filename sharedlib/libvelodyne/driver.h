/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id: driver.h 2220 2012-03-25 22:35:57Z jack.oquin $
 */

/** \file
 *
 *  ROS driver interface for the Velodyne 3D LIDARs
 */

#ifndef _VELODYNE_DRIVER_H_
#define _VELODYNE_DRIVER_H_

#include <string>
#include <carmen/carmen.h>

#include "input.h"

namespace velodyne_driver
{

typedef struct
{
	unsigned short distance[32];
	unsigned char intensity[32];
	double angle;
}carmen_velodyne_32_laser_shot;

typedef struct
{
	carmen_velodyne_32_laser_shot *laser_shots;
	int number_of_32_laser_shots;
	double timestamp;
}velodyne_partial_scan_t;

typedef struct
{
	velodyne_gps_packet_t packet;
	double timestamp;
}velodyne_gps_t;

typedef struct
{
	double angular_distance_between_two_32_laser_shots;
	int size_of_32_laser_shot_grouping;
}velodyne_config_t;

class VelodyneDriver
{
public:


	VelodyneDriver(int laser_udp_port, int gps_upd_port, int number_of_32_laser_shots_per_revolution, int size_of_32_laser_shot_grouping);
	~VelodyneDriver();

	bool pollScan(void);
	bool pollGps(void);

	velodyne_partial_scan_t getVelodyneScan() { return scan_; }
	velodyne_gps_t getVelodyneGps() { return gps_; }
	velodyne_config_t getVelodyneConfig() { return config_; }

	void printVelodyneGps();
	void printVelodyneScan();

private:

	InputSocket* laser_input_;
	InputSocket* gps_input_;

	velodyne_partial_scan_t scan_;
	velodyne_gps_t gps_;
	velodyne_config_t config_;
};

} // namespace velodyne_driver

#endif // _VELODYNE_DRIVER_H_
