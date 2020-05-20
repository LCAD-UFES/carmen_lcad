#include <carmen/carmen.h>
#include <carmen/moving_objects_messages.h>

/*************
 * grid map *
 ************/
void
carmen_moving_objects_map_subscribe_message(carmen_moving_objects_map_message *message,
	       carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	  carmen_subscribe_message(CARMEN_MOVING_OBJECTS_MAP_MESSAGE_NAME,
			  	  	  	  	  CARMEN_MOVING_OBJECTS_MAP_MESSAGE_FMT,
	                           message, sizeof(carmen_moving_objects_map_message),
														 handler, subscribe_how);
}

void
carmen_moving_objects_map_unsubscribe_message(carmen_handler_t handler)
{
	  carmen_unsubscribe_message(CARMEN_MOVING_OBJECTS_MAP_MESSAGE_NAME, handler);
}

void
carmen_moving_objects_map_publish_message(carmen_moving_objects_map_message *moving_objects_map_message)
{
	 IPC_RETURN_TYPE err;

	  err = IPC_publishData(CARMEN_MOVING_OBJECTS_MAP_MESSAGE_NAME, moving_objects_map_message);
	  carmen_test_ipc_exit(err, "Could not publish", CARMEN_MOVING_OBJECTS_MAP_MESSAGE_NAME);
}

void
carmen_moving_objects_map_define_messages(){
	 IPC_RETURN_TYPE err;

	  err = IPC_defineMsg(CARMEN_MOVING_OBJECTS_MAP_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			  CARMEN_MOVING_OBJECTS_MAP_MESSAGE_FMT);
	  carmen_test_ipc_exit(err, "Could not define", CARMEN_MOVING_OBJECTS_MAP_MESSAGE_NAME);
}


/*************
 * laser *
 ************/
void
carmen_moving_objects_point_clouds_subscribe_message(carmen_moving_objects_point_clouds_message *message,
	       carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	  carmen_subscribe_message(CARMEN_MOVING_OBJECTS_POINT_CLOUDS_MESSAGE_NAME,
			  	  	  	  	  CARMEN_MOVING_OBJECTS_POINT_CLOUDS_MESSAGE_FMT,
	                           message, sizeof(carmen_moving_objects_point_clouds_message),
														 handler, subscribe_how);

}

void
carmen_moving_objects_point_clouds_unsubscribe_message(carmen_handler_t handler)
{
	  carmen_unsubscribe_message(CARMEN_MOVING_OBJECTS_POINT_CLOUDS_MESSAGE_NAME, handler);
}

void
carmen_moving_objects_point_clouds_publish_message(carmen_moving_objects_point_clouds_message *msg)
{
	 IPC_RETURN_TYPE err;

	 err = IPC_publishData(CARMEN_MOVING_OBJECTS_POINT_CLOUDS_MESSAGE_NAME, msg);
	 carmen_test_ipc_exit(err, "Could not publish", CARMEN_MOVING_OBJECTS_POINT_CLOUDS_MESSAGE_NAME);
}

void
carmen_moving_objects_point_clouds_define_messages(){
	 IPC_RETURN_TYPE err;

	  err = IPC_defineMsg(CARMEN_MOVING_OBJECTS_POINT_CLOUDS_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			  CARMEN_MOVING_OBJECTS_POINT_CLOUDS_MESSAGE_FMT);
	  carmen_test_ipc_exit(err, "Could not define", CARMEN_MOVING_OBJECTS_POINT_CLOUDS_MESSAGE_NAME);
}
