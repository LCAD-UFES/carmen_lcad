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
#include <carmen/base_messages.h>

void 
carmen_base_subscribe_velocity_message(carmen_base_velocity_message *velocity,
				       carmen_handler_t handler,
				       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_BASE_VELOCITY_NAME, 
                           CARMEN_BASE_VELOCITY_FMT,
                           velocity, sizeof(carmen_base_velocity_message), 
			   handler, subscribe_how);
}

void
carmen_base_subscribe_odometry_message(carmen_base_odometry_message *odometry,
				       carmen_handler_t handler,
				       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_BASE_ODOMETRY_NAME, 
                           CARMEN_BASE_ODOMETRY_FMT,
                           odometry, sizeof(carmen_base_odometry_message), 
			   handler, subscribe_how);
}

void
carmen_base_unsubscribe_odometry_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_BASE_ODOMETRY_NAME, handler);
}

void
carmen_base_subscribe_sonar_message(carmen_base_sonar_message *sonar,
				    carmen_handler_t handler,
				    carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_BASE_SONAR_NAME, 
                           CARMEN_BASE_SONAR_FMT,
                           sonar, sizeof(carmen_base_sonar_message), 
			   handler, subscribe_how);
}

void
carmen_base_unsubscribe_sonar_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_BASE_SONAR_NAME, handler);
}

void
carmen_base_subscribe_bumper_message(carmen_base_bumper_message *bumper,
				     carmen_handler_t handler,
				     carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_BASE_BUMPER_NAME, 
                           CARMEN_BASE_BUMPER_FMT,
                           bumper, sizeof(carmen_base_bumper_message), 
			   handler, subscribe_how);
}

void
carmen_base_unsubscribe_bumper_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_BASE_BUMPER_NAME, handler);
}

void
carmen_base_subscribe_reset_occurred_message(carmen_base_reset_occurred_message *reset,
					     carmen_handler_t handler,
					     carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_BASE_RESET_OCCURRED_NAME, 
                           CARMEN_DEFAULT_MESSAGE_FMT,
                           reset, sizeof(carmen_default_message), 
			   handler, subscribe_how);
}

void
carmen_base_unsubscribe_reset_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_BASE_RESET_OCCURRED_NAME, handler);
}

void 
carmen_base_reset(void)
{
  IPC_RETURN_TYPE err;
  carmen_base_reset_command_message *msg;
  static int first = 1;

  if(first) {
    err = IPC_defineMsg(CARMEN_BASE_RESET_COMMAND_NAME, 
			IPC_VARIABLE_LENGTH, 
			CARMEN_DEFAULT_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_BASE_RESET_COMMAND_NAME);
    
    first = 0;
  }

  msg = carmen_default_message_create();

  err = IPC_publishData(CARMEN_BASE_RESET_COMMAND_NAME, msg);
  carmen_test_ipc(err, "Could not publish", CARMEN_BASE_RESET_COMMAND_NAME);
}
