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
#include "laser.h"
#include "laser_messages.h"
#include "sick.h"


int allocsize[5] = {0, 0, 0, 0, 0};
float *range_buffer[5] = {NULL, NULL, NULL, NULL, NULL};

int allocremsize[5] = {0, 0, 0, 0, 0};
float *remission_buffer[5] = {NULL, NULL, NULL, NULL, NULL};


void publish_laser_alive(int front_stalled, int rear_stalled,
			 int laser3_stalled, int laser4_stalled,
			 int laser5_stalled )
{
  IPC_RETURN_TYPE err;
  carmen_laser_alive_message msg;

  msg.frontlaser_stalled = front_stalled;
  msg.rearlaser_stalled  = rear_stalled;
  msg.laser3_stalled     = laser3_stalled;
  msg.laser4_stalled     = laser4_stalled;
  msg.laser5_stalled     = laser5_stalled;

  err = IPC_publishData(CARMEN_LASER_ALIVE_NAME, &msg);
  carmen_test_ipc_exit(err, "Could not publish", CARMEN_LASER_ALIVE_NAME);
}

void publish_laser_message(sick_laser_p laser,
			   const carmen_laser_laser_config_t* config)
{
  static carmen_laser_laser_message msg;

  IPC_RETURN_TYPE err;
  int i;

  msg.host = carmen_get_host();
  msg.num_readings = laser->numvalues; 
  msg.timestamp = laser->timestamp;
  msg.config = *config;
  
  if(msg.num_readings != allocsize[laser->settings.laser_num]) {
    range_buffer[laser->settings.laser_num] = 
      realloc(range_buffer[laser->settings.laser_num],
	      msg.num_readings * sizeof(float));
    carmen_test_alloc(range_buffer[laser->settings.laser_num]);
    allocsize[laser->settings.laser_num] = msg.num_readings;
  }
  msg.range = range_buffer[laser->settings.laser_num];

  if( laser->settings.laser_flipped == 0)  {
    for(i = 0; i < msg.num_readings; i++)
      msg.range[i] = laser->range[i] / 100.0;
  }
  else {      
    for(i = 0; i < msg.num_readings; i++)
      msg.range[i] = laser->range[msg.num_readings-1-i] / 100.0;
  }

  if(laser->settings.use_remission == 1) {
    msg.num_remissions = msg.num_readings;
    
    if(msg.num_remissions != allocremsize[laser->settings.laser_num]) {
      remission_buffer[laser->settings.laser_num] = 
	realloc(remission_buffer[laser->settings.laser_num],
		msg.num_remissions * sizeof(float));
      carmen_test_alloc(remission_buffer[laser->settings.laser_num]);
      allocremsize[laser->settings.laser_num] = msg.num_remissions;
    }
    msg.remission = remission_buffer[laser->settings.laser_num];
    
    if( laser->settings.laser_flipped == 0)  {
      for(i = 0; i < msg.num_remissions; i++) 
	msg.remission[i] = laser->remission[i];
    }
    else {
      for(i = 0; i < msg.num_remissions; i++)
	msg.remission[i] = laser->remission[msg.num_remissions-1-i];
    }      
  }
  else {
    msg.num_remissions = 0;
    msg.remission = NULL ;
  }

  switch(laser->settings.laser_num) {
  case CARMEN_FRONT_LASER_NUM:
    msg.id = 1;
    err = IPC_publishData(CARMEN_LASER_FRONTLASER_NAME, &msg);
    carmen_test_ipc_exit(err, "Could not publish", 
			 CARMEN_LASER_FRONTLASER_NAME);
    break;
  case CARMEN_REAR_LASER_NUM:
    msg.id = 2;
    err = IPC_publishData(CARMEN_LASER_REARLASER_NAME, &msg);
    carmen_test_ipc_exit(err, "Could not publish", 
			 CARMEN_LASER_REARLASER_NAME);
    break;
  case CARMEN_LASER3_NUM:
    msg.id = 3;
    err = IPC_publishData(CARMEN_LASER_LASER3_NAME, &msg);
    carmen_test_ipc_exit(err, "Could not publish", 
			 CARMEN_LASER_LASER3_NAME);
    break;
  case CARMEN_LASER4_NUM:
    msg.id = 4;
    err = IPC_publishData(CARMEN_LASER_LASER4_NAME, &msg);
    carmen_test_ipc_exit(err, "Could not publish", 
			 CARMEN_LASER_LASER4_NAME);
    break;
  case CARMEN_LASER5_NUM:
    msg.id = 5;
    err = IPC_publishData(CARMEN_LASER_LASER5_NAME, &msg);
    carmen_test_ipc_exit(err, "Could not publish", 
			 CARMEN_LASER_LASER5_NAME);
    break;
  }
}

void ipc_initialize_messages(void)
{
  IPC_RETURN_TYPE err;
  
  err = IPC_defineMsg(CARMEN_LASER_FRONTLASER_NAME, IPC_VARIABLE_LENGTH, 
		      CARMEN_LASER_FRONTLASER_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_FRONTLASER_NAME);
  
  err = IPC_defineMsg(CARMEN_LASER_REARLASER_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_LASER_REARLASER_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_REARLASER_NAME);

  err = IPC_defineMsg(CARMEN_LASER_LASER3_NAME, IPC_VARIABLE_LENGTH, 
		      CARMEN_LASER_LASER3_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_LASER3_NAME);
  
  err = IPC_defineMsg(CARMEN_LASER_LASER4_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_LASER_LASER4_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_LASER4_NAME);
  
  err = IPC_defineMsg(CARMEN_LASER_LASER5_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_LASER_LASER5_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_LASER5_NAME);
  
  err = IPC_defineMsg(CARMEN_LASER_ALIVE_NAME, IPC_VARIABLE_LENGTH,
                      CARMEN_LASER_ALIVE_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_LASER_ALIVE_NAME);
}
