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
#include "base_ackerman_messages.h"

void
carmen_base_ackerman_subscribe_odometry_message(carmen_base_ackerman_odometry_message *odometry,
				       carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, CARMEN_BASE_ACKERMAN_ODOMETRY_FMT,
                           odometry, sizeof(carmen_base_ackerman_odometry_message), handler, subscribe_how);
}


// Message redefined to insert a module between the obstacle_avoider and the ford_escape_hybrid
void
carmen_base_ackerman_subscribe_odometry_message_2(carmen_base_ackerman_odometry_message *odometry,
				       carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_BASE_ACKERMAN_ODOMETRY_2_NAME, CARMEN_BASE_ACKERMAN_ODOMETRY_2_FMT,
                           odometry, sizeof(carmen_base_ackerman_odometry_message), handler, subscribe_how);
}


void
carmen_base_ackerman_unsubscribe_odometry_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, handler);
}


void
carmen_base_ackerman_subscribe_motion_command(carmen_base_ackerman_motion_command_message *motion_command,
				     carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, CARMEN_BASE_ACKERMAN_MOTION_COMMAND_FMT,
                	   motion_command, sizeof(carmen_base_ackerman_motion_command_message), handler, subscribe_how);
}


void
carmen_base_ackerman_publish_motion_command(carmen_ackerman_motion_command_p motion_command, int num_motion_commands, double timestamp)
{
	IPC_RETURN_TYPE err;
	static carmen_base_ackerman_motion_command_message msg;
	static int first_time = 1;

	if (first_time)
	{
		err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, IPC_VARIABLE_LENGTH, CARMEN_BASE_ACKERMAN_MOTION_COMMAND_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME);
		first_time = 0;
	}

	msg.motion_command = motion_command;
	msg.num_motion_commands = num_motion_commands;

	msg.timestamp = timestamp;
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME);
}


// Message redefined to insert a module between the obstacle_avoider and the ford_escape_hybrid
void
carmen_base_ackerman_subscribe_motion_command_2(carmen_base_ackerman_motion_command_message *motion_command,
				     carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_2_NAME, CARMEN_BASE_ACKERMAN_MOTION_COMMAND_2_FMT,
                	   motion_command, sizeof(carmen_base_ackerman_motion_command_message), handler, subscribe_how);
}


void
carmen_base_ackerman_publish_motion_command_2(carmen_ackerman_motion_command_p motion_command, int num_motion_commands, double timestamp)
{
	IPC_RETURN_TYPE err;
	static carmen_base_ackerman_motion_command_message msg;
	static int first_time = 1;

	if (first_time)
	{
		err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_2_NAME, IPC_VARIABLE_LENGTH, CARMEN_BASE_ACKERMAN_MOTION_COMMAND_2_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_BASE_ACKERMAN_MOTION_COMMAND_2_NAME);
		first_time = 0;
	}

	msg.motion_command = motion_command;
	msg.num_motion_commands = num_motion_commands;

	msg.timestamp = timestamp;
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_2_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_BASE_ACKERMAN_MOTION_COMMAND_2_NAME);
}

