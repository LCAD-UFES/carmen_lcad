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
#include "xr4000_control.h"

static void carmen_xr4000_velocity_handler(MSG_INSTANCE msgRef, 
					 BYTE_ARRAY callData,
					 void *clientData 
					 __attribute__ ((unused)))
{
  IPC_RETURN_TYPE err;
  FORMATTER_PTR formatter;
  carmen_base_velocity_message vel;
 
  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &vel,
			   sizeof(carmen_base_velocity_message));
  IPC_freeByteArray(callData);
  carmen_test_ipc_return(err, "Could not unmarshall", 
		       IPC_msgInstanceName(msgRef));
  carmen_xr4000_set_velocity(0, vel.tv, vel.rv);
}

static void carmen_xr4000_holonomic_velocity_handler(MSG_INSTANCE msgRef, 
						   BYTE_ARRAY callData,
						   void *clientData 
						   __attribute__ ((unused)))
{
  IPC_RETURN_TYPE err;
  FORMATTER_PTR formatter;
  carmen_base_holonomic_velocity_message vel;
  
  formatter = IPC_msgInstanceFormatter(msgRef);
  err = IPC_unmarshallData(formatter, callData, &vel,
			   sizeof(carmen_base_holonomic_velocity_message));
  IPC_freeByteArray(callData);
  carmen_test_ipc_return(err, "Could not unmarshall", 
		       IPC_msgInstanceName(msgRef));
  carmen_xr4000_set_velocity(vel.xv, vel.yv, vel.rv);
}

void carmen_xr4000_register_ipc_messages(void)
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

  err = IPC_defineMsg(CARMEN_BASE_HOLONOMIC_VELOCITY_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_BASE_HOLONOMIC_VELOCITY_FMT);
  carmen_test_ipc_exit(err, "Could not define", 
		     CARMEN_BASE_HOLONOMIC_VELOCITY_NAME);

  /* setup incoming message handlers */
  err = IPC_subscribe(CARMEN_BASE_VELOCITY_NAME, 
		      carmen_xr4000_velocity_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subscribe", CARMEN_BASE_VELOCITY_NAME);
  IPC_setMsgQueueLength(CARMEN_BASE_VELOCITY_NAME, 1);

  err = IPC_subscribe(CARMEN_BASE_HOLONOMIC_VELOCITY_NAME, 
		      carmen_xr4000_holonomic_velocity_handler, NULL);
  carmen_test_ipc_exit(err, "Could not subscribe", 
		     CARMEN_BASE_HOLONOMIC_VELOCITY_NAME);
  IPC_setMsgQueueLength(CARMEN_BASE_HOLONOMIC_VELOCITY_NAME, 1);
}

