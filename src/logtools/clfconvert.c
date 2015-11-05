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

#include <carmen/global.h>
#include <carmen/ipc_wrapper.h>
#include <carmen/laser_interface.h>
#include <carmen/robot_interface.h>
#include <carmen/simulator_interface.h>
#include "clfconvert.h"
#include <carmen/movement.h>

void carmen_clfconvert_transform_laser_message_laser_pose(carmen_point_t refpose_currentframe,
							  carmen_point_t refpose_newframe,
							  carmen_robot_laser_message* msg) {
  msg->laser_pose = 
    carmen_movement_transformation_between_frames(refpose_currentframe, 
						  refpose_newframe,
						  msg->laser_pose);
}


void carmen_clfconvert_transform_laser_message_odom_pose(carmen_point_t refpose_currentframe,
							 carmen_point_t refpose_newframe,
							 carmen_robot_laser_message* msg) {
  msg->robot_pose = 
    carmen_movement_transformation_between_frames(refpose_currentframe, 
						  refpose_newframe, 
						  msg->laser_pose);
}

void carmen_clfconvert_transform_laser_message(carmen_point_t refpose_currentframe,
					       carmen_point_t refpose_newframe,
					       carmen_robot_laser_message* msg) {


  carmen_clfconvert_transform_laser_message_laser_pose(refpose_currentframe,
						       refpose_newframe,
						       msg);


  carmen_clfconvert_transform_laser_message_odom_pose(refpose_currentframe,
						      refpose_newframe,
						      msg);
}

void carmen_clfconvert_transform_odometry_message(carmen_point_t refpose_currentframe,
						  carmen_point_t refpose_newframe,
						  carmen_base_odometry_message* msg) {

  carmen_point_t t;
  
  t.x = msg->x;
  t.y = msg->y;
  t.theta = msg->theta;
  t = carmen_movement_transformation_between_frames(refpose_currentframe, 
						    refpose_newframe, t);
  msg->x = t.x;
  msg->y = t.y;
  msg->theta = t.theta;
}

void carmen_clfconvert_transform_truepos_message(carmen_point_t refpose_currentframe,
						 carmen_point_t refpose_newframe,
						 carmen_simulator_truepos_message* msg) {

  msg->odometrypose =  carmen_movement_transformation_between_frames(refpose_currentframe,
								     refpose_newframe, 
								     msg->odometrypose);
}

