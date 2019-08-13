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
#include "simulator_ackerman_messages.h"

void
carmen_simulator_ackerman_subscribe_truepos_message(carmen_simulator_ackerman_truepos_message 
					    *truepos,
					    carmen_handler_t handler,
					    carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME, 
                           CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_FMT,
                           truepos, sizeof(carmen_simulator_ackerman_truepos_message), 
			   handler, subscribe_how);
}

void
carmen_simulator_ackerman_subscribe_external_truepos_message(carmen_simulator_ackerman_truepos_message
					    *truepos,
					    carmen_handler_t handler,
					    carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_SIMULATOR_ACKERMAN_EXTERNAL_TRUEPOSE_NAME,
		  	  	  	  	   CARMEN_SIMULATOR_ACKERMAN_EXTERNAL_TRUEPOSE_FMT,
                           truepos, sizeof(carmen_simulator_ackerman_truepos_message),
			   handler, subscribe_how);
}

void
carmen_simulator_ackerman_unsubscribe_truepos_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME, handler);
}

void
carmen_simulator_ackerman_unsubscribe_external_truepos_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_SIMULATOR_ACKERMAN_EXTERNAL_TRUEPOSE_NAME, handler);
}

void
carmen_simulator_ackerman_subscribe_objects_message(carmen_simulator_ackerman_objects_message 
					   *objects,
					   carmen_handler_t handler,
					   carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_SIMULATOR_ACKERMAN_OBJECTS_NAME, 
                           CARMEN_SIMULATOR_ACKERMAN_OBJECTS_FMT,
                           objects, sizeof(carmen_simulator_ackerman_objects_message), 
			   handler, subscribe_how);
}

void
carmen_simulator_ackerman_unsubscribe_objects_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_SIMULATOR_ACKERMAN_OBJECTS_NAME, handler);
}

static unsigned int timeout = 5000;

int
carmen_simulator_ackerman_query_truepos(carmen_simulator_ackerman_truepos_message 
			       **carmen_simulator_interface_truepos_msg) 
{
  IPC_RETURN_TYPE err;
  carmen_simulator_ackerman_truepos_query_message *msg;
  static int initialized = 0;

  if(!initialized) {
    err = IPC_defineMsg(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_QUERY_NAME, 
			IPC_VARIABLE_LENGTH, 
			CARMEN_DEFAULT_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_QUERY_NAME);
    initialized = 1;
  }
  
  msg = carmen_default_message_create();
  err = IPC_queryResponseData(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_QUERY_NAME, msg, 
			      (void **)carmen_simulator_interface_truepos_msg,
			      timeout);
  carmen_test_ipc_return_int(err, "Could not query simulator truepos", 
			     CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_QUERY_NAME);
  return 0;
}

int 
carmen_simulator_ackerman_query_objects(carmen_simulator_ackerman_objects_message
			       **carmen_simulator_interface_objects_msg) 
{
  IPC_RETURN_TYPE err;
  carmen_simulator_ackerman_objects_query_message *msg;
  static int initialized = 0;

  if(!initialized) {
    err = IPC_defineMsg(CARMEN_SIMULATOR_ACKERMAN_OBJECTS_QUERY_NAME, 
			IPC_VARIABLE_LENGTH, 
			CARMEN_DEFAULT_MESSAGE_FMT);
    carmen_test_ipc_exit(err, "Could not define message", 
			 CARMEN_SIMULATOR_ACKERMAN_OBJECTS_QUERY_NAME);
    initialized = 1;
  }

  msg = carmen_default_message_create();
  err = IPC_queryResponseData(CARMEN_SIMULATOR_ACKERMAN_OBJECTS_QUERY_NAME, msg, 
			      (void **)carmen_simulator_interface_objects_msg,
			      timeout);
  carmen_test_ipc_return_int(err, "Could not query objects", 
			     CARMEN_SIMULATOR_ACKERMAN_OBJECTS_QUERY_NAME);
  return 0;
}

int 
carmen_simulator_ackerman_set_object(carmen_point_t *point, double speed,
			    carmen_simulator_ackerman_object_t type) 
{
  IPC_RETURN_TYPE err = IPC_OK;
  carmen_simulator_ackerman_set_object_message msg;
  static int initialized = 0;

  if (!initialized) 
    {
      err = IPC_defineMsg(CARMEN_SIMULATOR_ACKERMAN_SET_OBJECT_NAME, 
			  IPC_VARIABLE_LENGTH, 
			  CARMEN_SIMULATOR_ACKERMAN_SET_OBJECT_FMT);
      carmen_test_ipc_exit(err, "Could not define message", 
			   CARMEN_SIMULATOR_ACKERMAN_SET_OBJECT_NAME);
      initialized = 1;
    }

  msg.pose = *point;
  msg.speed = speed;
  msg.type = type;
  msg.timestamp = carmen_get_time();
  msg.host = carmen_get_host();

  err = IPC_publishData(CARMEN_SIMULATOR_ACKERMAN_SET_OBJECT_NAME, &msg);
  carmen_test_ipc(err, "Could not publish", CARMEN_SIMULATOR_ACKERMAN_SET_OBJECT_NAME);

  return 0;
}

void
carmen_simulator_ackerman_connect_robots(char *other_central)
{
  IPC_RETURN_TYPE err = IPC_OK;
  carmen_simulator_ackerman_connect_robots_message msg;
  static int initialized = 0;

  if (!initialized) 
    {
      err = IPC_defineMsg(CARMEN_SIMULATOR_ACKERMAN_CONNECT_ROBOTS_NAME, 
			  IPC_VARIABLE_LENGTH, 
			  CARMEN_SIMULATOR_ACKERMAN_CONNECT_ROBOTS_FMT);
      carmen_test_ipc_exit(err, "Could not define message", 
			   CARMEN_SIMULATOR_ACKERMAN_CONNECT_ROBOTS_NAME);
      initialized = 1;
    }


  msg.other_central = other_central;
  msg.timestamp = carmen_get_time();
  msg.host = carmen_get_host();

  err = IPC_publishData(CARMEN_SIMULATOR_ACKERMAN_CONNECT_ROBOTS_NAME, &msg);
  carmen_test_ipc(err, "Could not publish", 
		  CARMEN_SIMULATOR_ACKERMAN_CONNECT_ROBOTS_NAME);
}

void
carmen_simulator_ackerman_clear_objects(void)
{
  carmen_simulator_ackerman_clear_objects_message *msg;
  static int initialized = 0;
  IPC_RETURN_TYPE err = IPC_OK;

  if (!initialized) 
    {
      err = IPC_defineMsg(CARMEN_SIMULATOR_ACKERMAN_CLEAR_OBJECTS_NAME, 
			  IPC_VARIABLE_LENGTH, 
			  CARMEN_DEFAULT_MESSAGE_FMT);
      carmen_test_ipc_exit(err, "Could not define message", 
			   CARMEN_SIMULATOR_ACKERMAN_CLEAR_OBJECTS_NAME);
      initialized = 1;
    }

  msg = carmen_default_message_create();

  err = IPC_publishData(CARMEN_SIMULATOR_ACKERMAN_CLEAR_OBJECTS_NAME, msg);
  carmen_test_ipc(err, "Could not publish", 
		  CARMEN_SIMULATOR_ACKERMAN_CLEAR_OBJECTS_NAME);
}

void
carmen_simulator_ackerman_next_tick(void)
{
  IPC_RETURN_TYPE err = IPC_OK;
  carmen_simulator_ackerman_next_tick_message *msg;
  static int initialized = 0;

  if (!initialized) 
    {
      err = IPC_defineMsg(CARMEN_SIMULATOR_ACKERMAN_NEXT_TICK_NAME, 
			  IPC_VARIABLE_LENGTH, 
			  CARMEN_DEFAULT_MESSAGE_FMT);
      carmen_test_ipc_exit(err, "Could not define message", 
			   CARMEN_SIMULATOR_ACKERMAN_NEXT_TICK_NAME);
      initialized = 1;
    }

  msg = carmen_default_message_create();
  err = IPC_publishData(CARMEN_SIMULATOR_ACKERMAN_NEXT_TICK_NAME, msg);
  carmen_test_ipc(err, "Could not publish", 
		  CARMEN_SIMULATOR_ACKERMAN_NEXT_TICK_NAME);
}

int 
carmen_simulator_ackerman_set_truepose(carmen_point_t *point)
{
  IPC_RETURN_TYPE err = IPC_OK;
  carmen_simulator_ackerman_set_truepose_message msg;
  static int initialized = 0;

  if (!initialized) 
    {
      err = IPC_defineMsg(CARMEN_SIMULATOR_ACKERMAN_SET_TRUEPOSE_NAME, 
			  IPC_VARIABLE_LENGTH, 
			  CARMEN_SIMULATOR_ACKERMAN_SET_TRUEPOSE_FMT);
      carmen_test_ipc_exit(err, "Could not define message", 
			   CARMEN_SIMULATOR_ACKERMAN_SET_TRUEPOSE_NAME);
      initialized = 1;
    }

  msg.pose = *point;
  msg.timestamp = carmen_get_time();
  msg.host = carmen_get_host();

  err = IPC_publishData(CARMEN_SIMULATOR_ACKERMAN_SET_TRUEPOSE_NAME, &msg);
  carmen_test_ipc(err, "Could not publish", 
		  CARMEN_SIMULATOR_ACKERMAN_SET_TRUEPOSE_NAME);

  return 0;
}

int
carmen_simulator_ackerman_publish_external_truepose(carmen_simulator_ackerman_truepos_message *msg)
{
	IPC_RETURN_TYPE err = IPC_OK;
	static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_SIMULATOR_ACKERMAN_EXTERNAL_TRUEPOSE_NAME,
				IPC_VARIABLE_LENGTH, CARMEN_SIMULATOR_ACKERMAN_EXTERNAL_TRUEPOSE_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_SIMULATOR_ACKERMAN_EXTERNAL_TRUEPOSE_NAME);
		initialized = 1;
	}

	err = IPC_publishData(CARMEN_SIMULATOR_ACKERMAN_EXTERNAL_TRUEPOSE_NAME, msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_SIMULATOR_ACKERMAN_EXTERNAL_TRUEPOSE_NAME);

	return (0);
}
