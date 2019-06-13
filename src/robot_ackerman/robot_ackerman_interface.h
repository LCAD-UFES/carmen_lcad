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

/** @addtogroup robot librobot_interface **/
// @{

/** \file robot_interface.h
 * \brief Definition of the interface of the module robot.
 *
 * This file specifies the interface to subscribe the messages of
 * that module and to receive its data via ipc.
 **/

#ifndef CARMEN_ROBOT_ACKERMAN_INTERFACE_H
#define CARMEN_ROBOT_ACKERMAN_INTERFACE_H

#include "robot_ackerman_messages.h"
#include <carmen/ipc_wrapper.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef COMPILE_WITHOUT_LASER_SUPPORT
void carmen_robot_ackerman_subscribe_frontlaser_message
(carmen_robot_ackerman_laser_message *laser, carmen_handler_t handler,
 carmen_subscribe_t subscribe_how);

void carmen_robot_ackerman_subscribe_rearlaser_message
(carmen_robot_ackerman_laser_message *laser, carmen_handler_t handler,
 carmen_subscribe_t subscribe_how);
#endif

void carmen_robot_ackerman_subscribe_sonar_message
(carmen_robot_ackerman_sonar_message *sonar, carmen_handler_t handler,
 carmen_subscribe_t subscribe_how);

void carmen_robot_ackerman_subscribe_vector_status_message
(carmen_robot_ackerman_vector_status_message *vector, carmen_handler_t handler,
 carmen_subscribe_t subscribe_how);

void carmen_robot_ackerman_subscribe_follow_trajectory_message
(carmen_robot_ackerman_follow_trajectory_message *msg, carmen_handler_t handler,
 carmen_subscribe_t subscribe_how);
 
void carmen_robot_ackerman_subscribe_vector_move_message
(carmen_robot_ackerman_vector_move_message *msg, carmen_handler_t handler,
 carmen_subscribe_t subscribe_how);

void carmen_robot_ackerman_subscribe_velocity_message
(carmen_robot_ackerman_velocity_message *msg, carmen_handler_t handler,
 carmen_subscribe_t subscribe_how);

// Message redefined to insert a module between the obstacle_avoider and the ford_escape_hybrid
void carmen_robot_ackerman_subscribe_velocity_message_2(carmen_robot_ackerman_velocity_message *msg,
 	carmen_handler_t handler,
 	carmen_subscribe_t subscribe_how);
 
void carmen_robot_ackerman_subscribe_motion_command(carmen_robot_ackerman_motion_command_message *motion_command,carmen_handler_t handler, carmen_subscribe_t subscribe_how);
void carmen_robot_ackerman_subscribe_teacher_motion_command(carmen_robot_ackerman_motion_command_message *motion_command, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void carmen_robot_ackerman_move_along_vector(double distance, double theta);
void carmen_robot_ackerman_follow_trajectory(carmen_ackerman_traj_point_p trajectory, int trajectory_length,
			carmen_ackerman_traj_point_t *robot);

void carmen_robot_ackerman_publish_motion_command(carmen_ackerman_motion_command_p motion_command, int num_motion_commands, double timestamp);
void carmen_robot_ackerman_publish_teacher_motion_command(carmen_ackerman_motion_command_p motion_command, int num_motion_commands, double timestamp);

#ifdef __cplusplus
}
#endif

#endif
// @}
