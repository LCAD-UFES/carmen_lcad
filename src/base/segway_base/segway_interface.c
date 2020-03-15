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
#include <carmen/segway_messages.h>

void
carmen_segway_subscribe_pose_message(carmen_segway_pose_message *pose,
				     carmen_handler_t handler,
				     carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_SEGWAY_POSE_NAME, 
                           CARMEN_SEGWAY_POSE_FMT,
                           pose, sizeof(carmen_segway_pose_message), 
			   handler, subscribe_how);
}

void
carmen_segway_unsubscribe_pose_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_SEGWAY_POSE_NAME, handler);
}

void
carmen_segway_subscribe_battery_message(carmen_segway_battery_message *battery,
					carmen_handler_t handler,
					carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_SEGWAY_BATTERY_NAME, 
                           CARMEN_SEGWAY_BATTERY_FMT,
                           battery, sizeof(carmen_segway_battery_message), 
			   handler, subscribe_how);
}

void
carmen_segway_unsubscribe_battery_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_SEGWAY_BATTERY_NAME, handler);
}

void carmen_segway_kill_command(void)
{
  IPC_RETURN_TYPE err;
  static int first = 1;
  
  if(first) {
    err = IPC_defineMsg(CARMEN_SEGWAY_KILL_NAME, IPC_VARIABLE_LENGTH, 
                        CARMEN_SEGWAY_KILL_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
                         CARMEN_SEGWAY_KILL_NAME);
    first = 0;
  }
  err = IPC_publishData(CARMEN_SEGWAY_KILL_NAME, NULL);
  carmen_test_ipc(err, "Could not publish", CARMEN_SEGWAY_KILL_NAME);
}
