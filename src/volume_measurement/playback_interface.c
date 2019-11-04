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


void
carmen_subscribe_playback_info_message (carmen_playback_info_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message (CARMEN_PLAYBACK_INFO_MESSAGE_NAME, CARMEN_PLAYBACK_INFO_MESSAGE_FMT, message, sizeof (carmen_playback_info_message), handler, subscribe_how);
}


void
carmen_unsubscribe_playback_info_message (carmen_handler_t handler)
{
	carmen_unsubscribe_message (CARMEN_PLAYBACK_INFO_MESSAGE_NAME, handler);
}


void
carmen_playback_define_messages (void)
{
	IPC_RETURN_TYPE err = IPC_defineMsg (CARMEN_PLAYBACK_INFO_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_PLAYBACK_INFO_MESSAGE_FMT);
	carmen_test_ipc_exit (err, "Could not define", CARMEN_PLAYBACK_INFO_MESSAGE_NAME);
}


void
carmen_playback_command (int command, int argument, float speed)
{
	IPC_RETURN_TYPE err;
	carmen_playback_command_message playback_msg;

	static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg (CARMEN_PLAYBACK_COMMAND_NAME, IPC_VARIABLE_LENGTH, CARMEN_PLAYBACK_COMMAND_FMT);
		carmen_test_ipc_exit (err, "Could not define message", CARMEN_PLAYBACK_COMMAND_NAME);

		initialized = 1;
	}

	playback_msg.cmd = command;
	playback_msg.arg = argument;
	playback_msg.speed = speed;

	err = IPC_publishData (CARMEN_PLAYBACK_COMMAND_NAME, &playback_msg);
	carmen_test_ipc (err, "Could not publish", CARMEN_PLAYBACK_COMMAND_NAME);
}
