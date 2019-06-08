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

#include <carmen/carmen.h>
#include "robot_ackerman_messages.h"
#include "robot_ackerman_laser.h"
#include "robot_ackerman_interface.h"
#include <carmen/ipc_wrapper.h>

#include <carmen/carmen.h>
#include <carmen/navigator_ackerman_messages.h>


#ifndef COMPILE_WITHOUT_LASER_SUPPORT
void 
carmen_robot_ackerman_subscribe_frontlaser_message(carmen_robot_ackerman_laser_message *laser,
	carmen_handler_t handler,
	carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME, 
		CARMEN_ROBOT_ACKERMAN_FRONTLASER_FMT,
		laser, sizeof(carmen_robot_ackerman_laser_message),
		handler, subscribe_how);
}


void 
carmen_robot_ackerman_subscribe_rearlaser_message(carmen_robot_ackerman_laser_message *laser,
	carmen_handler_t handler,
	carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_ROBOT_ACKERMAN_REARLASER_NAME, 
		CARMEN_ROBOT_ACKERMAN_REARLASER_FMT,
		laser, sizeof(carmen_robot_ackerman_laser_message),
		handler, subscribe_how);
}
#endif


void
carmen_robot_ackerman_subscribe_sonar_message(carmen_robot_ackerman_sonar_message *sonar,
	carmen_handler_t handler,
	carmen_subscribe_t subscribe_how) 
{
	carmen_subscribe_message(CARMEN_ROBOT_ACKERMAN_SONAR_NAME, 
		CARMEN_ROBOT_ACKERMAN_SONAR_FMT,
		sonar, sizeof(carmen_robot_ackerman_sonar_message),
		handler, subscribe_how);
}


void
carmen_robot_ackerman_subscribe_vector_status_message(carmen_robot_ackerman_vector_status_message *status, 
	carmen_handler_t handler, 
	carmen_subscribe_t subscribe_how) 
{
	carmen_subscribe_message(CARMEN_ROBOT_ACKERMAN_VECTOR_STATUS_NAME, 
		CARMEN_ROBOT_ACKERMAN_VECTOR_STATUS_FMT,
		status, sizeof(carmen_robot_ackerman_vector_status_message),
		handler, subscribe_how);
}


void 
carmen_robot_ackerman_subscribe_follow_trajectory_message(carmen_robot_ackerman_follow_trajectory_message *msg, 
	carmen_handler_t handler,
	carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_ROBOT_ACKERMAN_FOLLOW_TRAJECTORY_NAME,
		CARMEN_ROBOT_ACKERMAN_FOLLOW_TRAJECTORY_FMT,
		msg, sizeof(carmen_robot_ackerman_follow_trajectory_message),
		handler, subscribe_how);	
}
 

void 
carmen_robot_ackerman_subscribe_vector_move_message(carmen_robot_ackerman_vector_move_message *msg, 
	carmen_handler_t handler,
	carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_ROBOT_ACKERMAN_VECTOR_MOVE_NAME,
		CARMEN_ROBOT_ACKERMAN_VECTOR_MOVE_FMT,
		msg, sizeof(carmen_robot_ackerman_vector_move_message),
		handler, subscribe_how);
}


void 
carmen_robot_ackerman_subscribe_velocity_message(carmen_robot_ackerman_velocity_message *msg, 
	carmen_handler_t handler,
	carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME,
		CARMEN_ROBOT_ACKERMAN_VELOCITY_FMT,
		msg, sizeof(carmen_robot_ackerman_velocity_message),
		handler, subscribe_how);
}


// Message redefined to insert a module between the obstacle_avoider and the ford_escape_hybrid
void
carmen_robot_ackerman_subscribe_velocity_message_2(carmen_robot_ackerman_velocity_message *msg,
	carmen_handler_t handler,
	carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_ROBOT_ACKERMAN_VELOCITY_2_NAME,
		CARMEN_ROBOT_ACKERMAN_VELOCITY_2_FMT,
		msg, sizeof(carmen_robot_ackerman_velocity_message),
		handler, subscribe_how);
}


void
carmen_robot_ackerman_subscribe_motion_command(carmen_robot_ackerman_motion_command_message *motion_command,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_NAME,
		CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_FMT,
		motion_command, sizeof(carmen_robot_ackerman_motion_command_message),
		handler, subscribe_how);
}


void
carmen_robot_ackerman_subscribe_teacher_motion_command(carmen_robot_ackerman_motion_command_message *motion_command,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_ROBOT_ACKERMAN_TEACHER_MOTION_COMMAND_NAME,
		CARMEN_ROBOT_ACKERMAN_TEACHER_MOTION_COMMAND_FMT,
		motion_command, sizeof(carmen_robot_ackerman_motion_command_message),
		handler, subscribe_how);
}


void
carmen_robot_ackerman_move_along_vector(double distance, double theta)
{
	IPC_RETURN_TYPE err;
	static carmen_robot_ackerman_vector_move_message msg;
	static int first_time = 1;

	if (first_time)
	{
		err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_VECTOR_MOVE_NAME, IPC_VARIABLE_LENGTH, 
			      CARMEN_ROBOT_ACKERMAN_VECTOR_MOVE_FMT);
		carmen_test_ipc_exit(err, "Could not define message", 
			      CARMEN_ROBOT_ACKERMAN_VECTOR_MOVE_NAME);
	}
	msg.distance = distance;
	msg.theta = theta;
	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_ROBOT_ACKERMAN_VECTOR_MOVE_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_ROBOT_ACKERMAN_VECTOR_MOVE_NAME);
}


void
carmen_robot_ackerman_follow_trajectory(carmen_ackerman_traj_point_p trajectory, int trajectory_length, carmen_ackerman_traj_point_t *robot)
{
	IPC_RETURN_TYPE err;
	static carmen_robot_ackerman_follow_trajectory_message msg;
	static int first_time = 1;

	if (first_time)
	{
		err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_FOLLOW_TRAJECTORY_NAME, 
			      IPC_VARIABLE_LENGTH, 
			      CARMEN_ROBOT_ACKERMAN_FOLLOW_TRAJECTORY_FMT);
		carmen_test_ipc_exit(err, "Could not define message", 
			      CARMEN_ROBOT_ACKERMAN_FOLLOW_TRAJECTORY_NAME);
	}
	msg.trajectory = trajectory;
	msg.trajectory_length = trajectory_length;
	msg.robot_position = *robot;

	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_ROBOT_ACKERMAN_FOLLOW_TRAJECTORY_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", 
		  CARMEN_ROBOT_ACKERMAN_FOLLOW_TRAJECTORY_NAME);
}


void
define_motion_command_message(const char *name, const char *fmt)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(name, IPC_VARIABLE_LENGTH, fmt);
	carmen_test_ipc_exit(err, "Could not define message", name);
}


void
publish_motion_command_with_name(carmen_ackerman_motion_command_p motion_command, int num_motion_commands, double timestamp, const char *name)
{
	IPC_RETURN_TYPE err;

	static carmen_robot_ackerman_motion_command_message msg;

	msg.motion_command = motion_command;
	msg.num_motion_commands = num_motion_commands;

	msg.timestamp = timestamp;
	msg.host = carmen_get_host();

	err = IPC_publishData(name, &msg);
	carmen_test_ipc(err, "Could not publish", name);
}


void 
carmen_robot_ackerman_publish_motion_command(carmen_ackerman_motion_command_p motion_command, int num_motion_commands, double timestamp)
{
	static int first_time = 1;

	if (first_time)
	{
		define_motion_command_message(CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_NAME, CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_FMT);
		first_time = 0;
	}

	publish_motion_command_with_name(motion_command, num_motion_commands,
		timestamp, CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_NAME);
}


void
carmen_robot_ackerman_publish_teacher_motion_command(carmen_ackerman_motion_command_p motion_command, int num_motion_commands, double timestamp)
{
	static int first_time = 1;

	if (first_time)
	{
		define_motion_command_message(CARMEN_ROBOT_ACKERMAN_TEACHER_MOTION_COMMAND_NAME, CARMEN_ROBOT_ACKERMAN_TEACHER_MOTION_COMMAND_FMT);
		first_time = 0;
	}

	publish_motion_command_with_name(motion_command, num_motion_commands,
		timestamp, CARMEN_ROBOT_ACKERMAN_TEACHER_MOTION_COMMAND_NAME);
}

