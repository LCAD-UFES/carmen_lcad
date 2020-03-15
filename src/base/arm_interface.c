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

#include <carmen/ipc.h>
#include <carmen/global.h>
#include <carmen/ipc_wrapper.h>
#include "arm_interface.h"
#include "arm_messages.h"

void
carmen_arm_subscribe_state_message(carmen_arm_state_message *arm_state,
				   carmen_handler_t handler,
				   carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_ARM_STATE_NAME,
                           CARMEN_ARM_STATE_FMT,
                           arm_state, sizeof(carmen_arm_state_message), 
			   handler, subscribe_how);
}

void carmen_arm_command(int num_joints, double *joint_angles)
{
  IPC_RETURN_TYPE err;
  carmen_arm_command_message msg;
  static int first = 1;

  if(first) {
    err = IPC_defineMsg(CARMEN_ARM_COMMAND_NAME, IPC_VARIABLE_LENGTH, 
			CARMEN_ARM_COMMAND_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_ARM_COMMAND_NAME);
    
    first = 0;
  }

  msg.num_joints = num_joints;
  msg.joint_angles = joint_angles;

  msg.timestamp = carmen_get_time();
  msg.host = carmen_get_host();

  err = IPC_publishData(CARMEN_ARM_COMMAND_NAME, &msg);
  carmen_test_ipc(err, "Could not publish", CARMEN_ARM_COMMAND_NAME);
}

static unsigned int timeout = 500;

int carmen_arm_query_state(carmen_arm_state_message **arm_state) 
{
  IPC_RETURN_TYPE err;
  carmen_arm_query_message *msg;
  static int initialized = 0;

  msg = carmen_default_message_create();

  if (!initialized) {
    err = IPC_defineMsg(CARMEN_ARM_QUERY_NAME, 
			IPC_VARIABLE_LENGTH, 
			CARMEN_DEFAULT_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_ARM_QUERY_NAME);
    initialized = 1;
  }

  err = IPC_queryResponseData(CARMEN_ARM_QUERY_NAME, msg, 
			      (void **)arm_state, timeout);
  carmen_test_ipc_return_int(err, "Could not query arm state", 
			     CARMEN_ARM_QUERY_NAME);

  return 0;
}
