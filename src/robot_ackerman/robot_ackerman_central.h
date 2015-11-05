/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

/** @addtogroup robot librobot **/
// @{

/** 
 * \file robot_central.h 
 * \brief Library for robot.
 *
 * ...
 **/


#ifndef CARMEN_ACKERMAN_ROBOT_H
#define CARMEN_ACKERMAN_ROBOT_H

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C" {
#endif

  // MAX_READINGS controls the number of odometry measurements we 
  // remember for interpolating position stamps.

  // Clock skew estimate is guaranteed only if sufficient odometry
  // is available, so ESTIMATES_CONVERGE is the amount of odometry
  // we need to estimate the clock skew

#define      CARMEN_ROBOT_ACKERMAN_MAX_READINGS         5
#define      CARMEN_ROBOT_ACKERMAN_ESTIMATES_CONVERGE   (CARMEN_ROBOT_ACKERMAN_MAX_READINGS+25)

#define      CARMEN_ROBOT_ACKERMAN_ALL_STOP             1
#define      CARMEN_ROBOT_ACKERMAN_ALLOW_ROTATE         2

#define      CARMEN_ROBOT_ACKERMAN_MIN_ALLOWED_VELOCITY 0.03 // cm/s

extern carmen_base_ackerman_odometry_message carmen_robot_ackerman_latest_odometry;
extern carmen_base_ackerman_odometry_message carmen_robot_ackerman_odometry[CARMEN_ROBOT_ACKERMAN_MAX_READINGS];
extern int carmen_robot_position_received;
extern int carmen_robot_converge;
extern carmen_robot_ackerman_config_t carmen_robot_ackerman_config;
extern char *carmen_robot_host;

extern double carmen_robot_ackerman_collision_avoidance_frequency;
extern double carmen_robot_ackerman_sensor_time_of_last_update;
  

double carmen_robot_ackerman_get_fraction(double timestamp, double skew,
				 int *low, int *high);
int carmen_robot_ackerman_get_skew(int msg_count, double *skew,
			  carmen_running_average_t *average, char *hostname);
void carmen_robot_ackerman_update_skew(carmen_running_average_t *average, int *count, 
			      double time, char *hostname);

#ifdef __cplusplus
}
#endif

#endif
// @}
