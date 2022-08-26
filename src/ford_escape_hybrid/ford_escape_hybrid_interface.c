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
#include "ford_escape_hybrid_messages.h"


void
carmen_ford_escape_subscribe_status_message(carmen_ford_escape_status_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_FORD_ESCAPE_STATUS_NAME,
		  	  	  	  	   CARMEN_FORD_ESCAPE_STATUS_FMT,
                           message, sizeof(carmen_ford_escape_status_message),
                           handler, subscribe_how);
}


void
carmen_ford_escape_unsubscribe_status_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_FORD_ESCAPE_STATUS_NAME, handler);
}

void
carmen_ford_escape_subscribe_tune_pid_gain_velocity_parameters_message(tune_pid_gain_velocity_parameters_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(TUNE_PID_GAIN_VELOCITY_PARAMENTERS_NAME, TUNE_PID_GAIN_VELOCITY_PARAMENTERS_FMT, message, sizeof(tune_pid_gain_velocity_parameters_message), handler, subscribe_how);
}

void
carmen_ford_escape_subscribe_tune_pid_gain_steering_parameters_message(tune_pid_gain_steering_parameters_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(TUNE_PID_GAIN_STEERING_PARAMENTERS_NAME, TUNE_PID_GAIN_STEERING_PARAMENTERS_FMT, message, sizeof(tune_pid_gain_steering_parameters_message), handler, subscribe_how);
}

void
carmen_ford_escape_subscribe_velocity_pid_data_message(velocity_pid_data_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(VELOCITY_PID_DATA_PARAMENTERS_NAME, VELOCITY_PID_DATA_PARAMENTERS_FMT, message, sizeof(velocity_pid_data_message), handler, subscribe_how);
}

void
carmen_ford_escape_subscribe_steering_pid_data_message(steering_pid_data_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(STEERING_PID_DATA_PARAMENTERS_NAME, STEERING_PID_DATA_PARAMENTERS_FMT, message, sizeof(steering_pid_data_message), handler, subscribe_how);
}


void
carmen_ford_escape_unsubscribe_tune_pid_gain_velocity_parameters_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(TUNE_PID_GAIN_VELOCITY_PARAMENTERS_NAME, handler);
}

void
carmen_ford_escape_unsubscribe_tune_pid_gain_steering_parameters_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(TUNE_PID_GAIN_STEERING_PARAMENTERS_NAME, handler);
}
void
carmen_ford_escape_unsubscribe_velocity_pid_data_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(VELOCITY_PID_DATA_PARAMENTERS_NAME,  handler);
}

void
carmen_ford_escape_unsubscribe_steering_pid_data_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(STEERING_PID_DATA_PARAMENTERS_NAME,  handler);
}

void
carmen_ford_escape_publish_tune_pid_gain_velocity_parameters_message(tune_pid_gain_velocity_parameters_message *msg, double timestamp)
{
	IPC_RETURN_TYPE err;
	static int first_time = 1;

	if (first_time)
	{
		err = IPC_defineMsg(TUNE_PID_GAIN_VELOCITY_PARAMENTERS_NAME, IPC_VARIABLE_LENGTH, TUNE_PID_GAIN_VELOCITY_PARAMENTERS_FMT);
		carmen_test_ipc_exit(err, "Could not define message", TUNE_PID_GAIN_VELOCITY_PARAMENTERS_NAME);
		first_time = 0;
	}

	msg->timestamp = timestamp;
	msg->host = carmen_get_host();

	err = IPC_publishData(TUNE_PID_GAIN_VELOCITY_PARAMENTERS_NAME, msg);
	carmen_test_ipc(err, "Could not publish", TUNE_PID_GAIN_VELOCITY_PARAMENTERS_NAME);
}

void
carmen_ford_escape_publish_tune_pid_gain_steering_parameters_message(tune_pid_gain_steering_parameters_message *msg, double timestamp)
{
	IPC_RETURN_TYPE err;
	static int first_time = 1;

	if (first_time)
	{
		err = IPC_defineMsg(TUNE_PID_GAIN_STEERING_PARAMENTERS_NAME, IPC_VARIABLE_LENGTH, TUNE_PID_GAIN_STEERING_PARAMENTERS_FMT);
		carmen_test_ipc_exit(err, "Could not define message", TUNE_PID_GAIN_STEERING_PARAMENTERS_NAME);
		first_time = 0;
	}

	msg->timestamp = timestamp;
	msg->host = carmen_get_host();

	err = IPC_publishData(TUNE_PID_GAIN_STEERING_PARAMENTERS_NAME, msg);
	carmen_test_ipc(err, "Could not publish", TUNE_PID_GAIN_STEERING_PARAMENTERS_NAME);
}

void
carmen_ford_escape_publish_velocity_pid_data_message(velocity_pid_data_message *msg, double timestamp)
{
	IPC_RETURN_TYPE err;
	static int first_time = 1;

	if (first_time)
	{
		err = IPC_defineMsg(VELOCITY_PID_DATA_PARAMENTERS_NAME, IPC_VARIABLE_LENGTH, VELOCITY_PID_DATA_PARAMENTERS_FMT);
		carmen_test_ipc_exit(err, "Could not define message", VELOCITY_PID_DATA_PARAMENTERS_NAME);
		first_time = 0;
	}

	msg->timestamp = timestamp;
	msg->host = carmen_get_host();

	err = IPC_publishData(VELOCITY_PID_DATA_PARAMENTERS_NAME, msg);
	carmen_test_ipc(err, "Could not publish", VELOCITY_PID_DATA_PARAMENTERS_NAME);
}

void
carmen_ford_escape_publish_steering_pid_data_message(steering_pid_data_message *msg, double timestamp)
{
	IPC_RETURN_TYPE err;
	static int first_time = 1;

	if (first_time)
	{
		err = IPC_defineMsg(STEERING_PID_DATA_PARAMENTERS_NAME, IPC_VARIABLE_LENGTH, STEERING_PID_DATA_PARAMENTERS_FMT);
		carmen_test_ipc_exit(err, "Could not define message", STEERING_PID_DATA_PARAMENTERS_NAME);
		first_time = 0;
	}

	msg->timestamp = timestamp;
	msg->host = carmen_get_host();

	err = IPC_publishData(STEERING_PID_DATA_PARAMENTERS_NAME, msg);
	carmen_test_ipc(err, "Could not publish", STEERING_PID_DATA_PARAMENTERS_NAME);
}


void
carmen_ford_escape_publish_status_message(carmen_ford_escape_status_message *msg, double timestamp)
{
	IPC_RETURN_TYPE err;
	static int first_time = 1;

	if (first_time)
	{
		err = IPC_defineMsg(CARMEN_FORD_ESCAPE_STATUS_NAME, IPC_VARIABLE_LENGTH, CARMEN_FORD_ESCAPE_STATUS_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_FORD_ESCAPE_STATUS_NAME);
		first_time = 0;
	}

	msg->timestamp = timestamp;
	msg->host = carmen_get_host();

	err = IPC_publishData(CARMEN_FORD_ESCAPE_STATUS_NAME, msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_FORD_ESCAPE_STATUS_NAME);
}


void
carmen_ford_escape_publish_signals_message(carmen_ford_escape_signals_message *msg, double timestamp)
{
	IPC_RETURN_TYPE err;
	static int first_time = 1;

	if (first_time)
	{
		err = IPC_defineMsg(CARMEN_FORD_ESCAPE_SIGNAL_NAME, IPC_VARIABLE_LENGTH, CARMEN_FORD_ESCAPE_SIGNAL_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_FORD_ESCAPE_SIGNAL_NAME);
		first_time = 0;
	}

	msg->timestamp = timestamp;
	msg->host = carmen_get_host();

	err = IPC_publishData(CARMEN_FORD_ESCAPE_SIGNAL_NAME, msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_FORD_ESCAPE_SIGNAL_NAME);
}

