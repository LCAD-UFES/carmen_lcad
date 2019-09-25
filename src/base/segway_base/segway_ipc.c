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
#include "segwaycore.h"
#include "segway_messages.h"

extern segway_t segway;
extern double command_tv, command_rv;
extern double last_command;
extern int quit_signal;

static void segway_velocity_handler(MSG_INSTANCE msgRef, 
				    BYTE_ARRAY callData,
				    void *clientData __attribute__ ((unused)))
{
  IPC_RETURN_TYPE err;
  carmen_base_velocity_message vel;
 
  err = IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData, &vel,
                           sizeof(carmen_base_velocity_message));
  IPC_freeByteArray(callData);
  carmen_test_ipc_return(err, "Could not unmarshall", 
			 IPC_msgInstanceName(msgRef));

  command_tv = vel.tv;
  command_rv = vel.rv;
  last_command = carmen_get_time();

  fprintf(stderr, "\rTV = %.1f     RV = %.1f   Battery = %d%%     ",
          command_tv, command_rv, (int)segway.voltage);
}

static void segway_kill_handler(MSG_INSTANCE msgRef __attribute__ ((unused)),
				BYTE_ARRAY callData __attribute__ ((unused)),
				void *clientData __attribute__ ((unused)))
{
  quit_signal = 1;
}

void carmen_segway_register_messages(void)
{
  IPC_RETURN_TYPE err;

  /* define messages created by this module */
  err = IPC_defineMsg(CARMEN_BASE_ODOMETRY_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_BASE_ODOMETRY_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_BASE_ODOMETRY_NAME);
  
  err = IPC_defineMsg(CARMEN_BASE_SONAR_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_BASE_SONAR_FMT);
  carmen_test_ipc_exit(err, "Could not define IPC message", 
		       CARMEN_BASE_SONAR_NAME);

  err = IPC_defineMsg(CARMEN_BASE_VELOCITY_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_BASE_VELOCITY_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_BASE_VELOCITY_NAME);

  err = IPC_defineMsg(CARMEN_SEGWAY_POSE_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_SEGWAY_POSE_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_SEGWAY_POSE_NAME);

  err = IPC_defineMsg(CARMEN_SEGWAY_BATTERY_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_SEGWAY_BATTERY_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_SEGWAY_POSE_NAME);

  err = IPC_defineMsg(CARMEN_SEGWAY_KILL_NAME, IPC_VARIABLE_LENGTH,
		      CARMEN_SEGWAY_KILL_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_SEGWAY_KILL_NAME);

  /* setup incoming message handlers */
  err = IPC_subscribe(CARMEN_BASE_VELOCITY_NAME, 
                      segway_velocity_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subscribe", CARMEN_BASE_VELOCITY_NAME);
  IPC_setMsgQueueLength(CARMEN_BASE_VELOCITY_NAME, 1);

  err = IPC_subscribe(CARMEN_SEGWAY_KILL_NAME, 
                      segway_kill_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subscribe", CARMEN_SEGWAY_KILL_NAME);
}

void carmen_segway_publish_odometry(segway_p segway, double timestamp)
{
  carmen_base_odometry_message odometry;
  IPC_RETURN_TYPE err;

  odometry.x = segway->x;
  odometry.y = segway->y;
  odometry.theta = segway->theta;
  odometry.tv = (segway->lw_velocity + segway->rw_velocity) / 2.0;
  odometry.rv = segway->yaw_rate;
  odometry.acceleration = 0;
  odometry.timestamp = timestamp;
  odometry.host = carmen_get_host();
  err = IPC_publishData(CARMEN_BASE_ODOMETRY_NAME, &odometry);
  carmen_test_ipc_exit(err, "Could not publish", CARMEN_BASE_ODOMETRY_NAME);
}

void carmen_segway_publish_pose(segway_p segway, double timestamp)
{
  static carmen_segway_pose_message pose;
  static int first = 1;
  IPC_RETURN_TYPE err;

  if(first) {
    pose.host = carmen_get_host();
    first = 0;
  }
  pose.pitch = segway->pitch;
  pose.pitch_rate = segway->pitch_rate;
  pose.roll = segway->roll;
  pose.roll_rate = segway->roll_rate;
  pose.lw_velocity = segway->lw_velocity;
  pose.rw_velocity = segway->rw_velocity;
  pose.tv = (segway->lw_velocity + segway->rw_velocity) / 2.0;
  pose.rv = segway->yaw_rate;
  pose.x = segway->x;
  pose.y = segway->y;
  pose.theta = segway->theta;
  pose.timestamp = timestamp;

  err = IPC_publishData(CARMEN_SEGWAY_POSE_NAME, &pose);
  carmen_test_ipc_exit(err, "Could not publish", CARMEN_SEGWAY_POSE_NAME);
}

void carmen_segway_publish_battery(segway_p segway)
{
  static carmen_segway_battery_message battery;
  static int first = 1;
  IPC_RETURN_TYPE err;

  if(first) {
    battery.host = carmen_get_host();
    first = 0;
  }
  battery.timestamp = carmen_get_time();
  battery.percent = segway->voltage;
  err = IPC_publishData(CARMEN_SEGWAY_BATTERY_NAME, &battery);
  carmen_test_ipc_exit(err, "Could not publish", CARMEN_SEGWAY_BATTERY_NAME);
}
