/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id: driver.cc 2221 2012-03-25 23:12:32Z jack.oquin $
 */

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include <string>
#include <stdlib.h>
#include <cmath>

#include "driver.h"

namespace velodyne_driver
{

VelodyneDriver::VelodyneDriver(int laser_udp_port, int gps_upd_port,
			       int number_of_32_laser_shots_per_revolution, int size_of_32_laser_shot_grouping)
{
	config_.angular_distance_between_two_32_laser_shots = 360.0 / (double) number_of_32_laser_shots_per_revolution;
	config_.size_of_32_laser_shot_grouping = size_of_32_laser_shot_grouping;

	scan_.number_of_32_laser_shots = config_.size_of_32_laser_shot_grouping;
	scan_.laser_shots = (carmen_velodyne_32_laser_shot* ) malloc (config_.size_of_32_laser_shot_grouping * sizeof(carmen_velodyne_32_laser_shot));

	laser_input_ = new velodyne_driver::InputSocket(laser_udp_port);
	gps_input_ = new velodyne_driver::InputSocket(gps_upd_port);
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool VelodyneDriver::pollScan(void)
{
	int i = 0;
	double initial_angular_value = 0.0;
	// Since the velodyne delivers data at a very high rate, keep
	// reading and publishing scans as fast as possible.
	while(true)
	{
		// keep reading until full packet received
		velodyne_packet_t packet;
		int rc = laser_input_->getScanPacket(&packet);

		if (rc < 0) return false; // end of file reached?

		for (int j = 0; j < velodyne_driver::VELODYNE_NUM_SHOTS; j++)
		{
			double current_angular_value = packet.shots[j].rotational_pos / 100.0;

			if(fabs(current_angular_value - initial_angular_value) >= config_.angular_distance_between_two_32_laser_shots)
			{
				for(int k=0; k < velodyne_driver::VELODYNE_NUM_LASERS; k++)
				{
					scan_.laser_shots[i].distance[k]  =  packet.shots[j].lasers[k].distance;
					scan_.laser_shots[i].intensity[k] =  packet.shots[j].lasers[k].intensity;
				}

				scan_.laser_shots[i].angle = current_angular_value;
				initial_angular_value = current_angular_value;

				if(i >= (config_.size_of_32_laser_shot_grouping - 1))
				{
					scan_.timestamp = carmen_get_time();
					return true;
				}

				i++;
			}
		}
	}

	printf("time: %f\n", scan_.timestamp);

	return true;
}

bool VelodyneDriver::pollGps(void)
{
	while (true)
	{
		// keep reading until full packet received
		int rc = gps_input_->getGpsPacket(&gps_.packet);
		if (rc == 0) break;       // got a full packet?
		if (rc < 0) return false; // end of file reached?
	}

	gps_.timestamp = carmen_get_time();  //get the last shot timestamp

	return true;
}

void VelodyneDriver::printVelodyneGps()
{
	gps_input_->printGpsPacket(gps_.packet);
}

void VelodyneDriver::printVelodyneScan()
{
	for(int i=0; i < scan_.number_of_32_laser_shots; i++)
	{
		printf("angle (%d): %6.2f\n", i, scan_.laser_shots[i].angle);

		//printf("data :");
		//for(int j=0; j < 32; j++)
		//	printf("%d (%d) ", scan_.laser_shots[i].distance[j], scan_.laser_shots[i].intensity[j]);
		//printf("\n");
	}
}

VelodyneDriver::~VelodyneDriver()
{
	free(scan_.laser_shots);

	if(laser_input_ != NULL)
		laser_input_->~InputSocket();
}

} // namespace velodyne_driver
