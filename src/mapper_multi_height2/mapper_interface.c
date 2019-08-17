#include <carmen/carmen.h>
#include "mapper_messages.h"


void
carmen_mapper_copy_map_from_message(carmen_map_t *current_map, carmen_mapper_map_message *online_map_message)
{
	int i;
	
	if (current_map->complete_map == NULL)
	{
		current_map->complete_map = (double *) malloc(online_map_message->size * sizeof(double));
		carmen_test_alloc(current_map->complete_map);
		current_map->map = (double **) malloc(online_map_message->config.x_size * sizeof(double *));
		carmen_test_alloc(current_map->map);
	}
	else if ((current_map->config.x_size != online_map_message->config.x_size) || (current_map->config.y_size != online_map_message->config.y_size))
	{
		free(current_map->complete_map);
		free(current_map->map);
		current_map->complete_map = (double *) malloc(online_map_message->size * sizeof(double));
		carmen_test_alloc(current_map->complete_map);
		current_map->map = (double **) malloc(online_map_message->config.x_size * sizeof(double *));
		carmen_test_alloc(current_map->map);
	}
	
	if (current_map->config.map_name != NULL)
		free(current_map->config.map_name);
	current_map->config = online_map_message->config;
	if (online_map_message->config.map_name != NULL)
	{
		current_map->config.map_name = (char *) malloc((strlen(online_map_message->config.map_name) + 1) * sizeof(char));
		carmen_test_alloc(current_map->config.map_name);
		strcpy(current_map->config.map_name, online_map_message->config.map_name);
	}

	memcpy(current_map->complete_map, online_map_message->complete_map, sizeof(double) * online_map_message->size);
	for (i = 0; i < current_map->config.x_size; i++)
		current_map->map[i] = current_map->complete_map + i * current_map->config.y_size;
}


void
carmen_mapper_subscribe_virtual_laser_message(carmen_mapper_virtual_laser_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_MAPPER_VIRTUAL_LASER_MESSAGE_NAME,
	CARMEN_MAPPER_VIRTUAL_LASER_MESSAGE_FMT, message, sizeof(carmen_mapper_virtual_laser_message), handler, subscribe_how);
}


void
carmen_mapper_subscribe_virtual_scan_message(carmen_mapper_virtual_scan_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_MAPPER_VIRTUAL_SCAN_MESSAGE_NAME, CARMEN_MAPPER_VIRTUAL_SCAN_MESSAGE_FMT, message,
			sizeof(carmen_mapper_virtual_scan_message), handler, subscribe_how);
}


void
carmen_mapper_subscribe_map_message(carmen_mapper_map_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_MAPPER_MAP_MESSAGE_NAME,
	CARMEN_MAPPER_MAP_MESSAGE_FMT, message, sizeof(carmen_mapper_map_message), handler, subscribe_how);
}

void
carmen_mapper_subscribe_map_level1_message(carmen_mapper_map_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_MAPPER_MAP_LEVEL1_MESSAGE_NAME,
	CARMEN_MAPPER_MAP_MESSAGE_FMT, message, sizeof(carmen_mapper_map_message), handler, subscribe_how);
}

void
carmen_mapper_moving_objects_raw_map_subscribe_message(carmen_mapper_map_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_MAPPER_MAP_MOVING_OBJECTS_RAW_MAP_MESSAGE_NAME,
	CARMEN_MAPPER_MAP_MESSAGE_FMT, message, sizeof(carmen_mapper_map_message), handler, subscribe_how);
}


//void
//carmen_mapper_distance_map_subscribe_message(carmen_mapper_distance_map_message *message,
//			       carmen_handler_t handler,
//			       carmen_subscribe_t subscribe_how)
//{
//  carmen_subscribe_message(CARMEN_MAPPER_DISTANCE_MAP_MESSAGE_NAME,
//		  	  	  	  	  CARMEN_MAPPER_DISTANCE_MAP_MESSAGE_FMT,
//                          message, sizeof(carmen_mapper_distance_map_message),
//                          handler, subscribe_how);
//}


void
carmen_mapper_unsubscribe_virtual_laser_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_MAPPER_VIRTUAL_LASER_MESSAGE_NAME, handler);
}


void
carmen_mapper_unsubscribe_virtual_scan_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_MAPPER_VIRTUAL_SCAN_MESSAGE_NAME, handler);
}


void
carmen_mapper_unsubscribe_map_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_MAPPER_MAP_MESSAGE_NAME, handler);
}

void
carmen_mapper_unsubscribe_map_level1_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_MAPPER_MAP_LEVEL1_MESSAGE_NAME, handler);
}

void
carmen_mapper_moving_objects_raw_map_unsubscribe_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_MAPPER_MAP_MOVING_OBJECTS_RAW_MAP_MESSAGE_NAME, handler);
}


void
carmen_mapper_define_virtual_laser_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_MAPPER_VIRTUAL_LASER_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAPPER_VIRTUAL_LASER_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAPPER_VIRTUAL_LASER_MESSAGE_NAME);
}


void
carmen_mapper_define_virtual_scan_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_MAPPER_VIRTUAL_SCAN_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAPPER_VIRTUAL_SCAN_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAPPER_VIRTUAL_SCAN_MESSAGE_NAME);
}


void
carmen_mapper_define_map_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_MAPPER_MAP_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAPPER_MAP_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAPPER_MAP_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_MAPPER_MAP_LEVEL1_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
				CARMEN_MAPPER_MAP_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAPPER_MAP_LEVEL1_MESSAGE_NAME);

}


void
carmen_mapper_define_messages()
{
	IPC_RETURN_TYPE err;

	carmen_mapper_define_map_message();

	err = IPC_defineMsg(CARMEN_MAPPER_MAP_MOVING_OBJECTS_RAW_MAP_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAPPER_MAP_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAPPER_MAP_MOVING_OBJECTS_RAW_MAP_MESSAGE_NAME);

	carmen_mapper_define_virtual_laser_message();
}


void
carmen_mapper_publish_virtual_laser_message(carmen_mapper_virtual_laser_message *virtual_laser_message, double timestamp)
{
	IPC_RETURN_TYPE err;

	virtual_laser_message->timestamp = timestamp;

	err = IPC_publishData(CARMEN_MAPPER_VIRTUAL_LASER_MESSAGE_NAME, virtual_laser_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_MAPPER_VIRTUAL_LASER_MESSAGE_NAME);
}


void
carmen_mapper_publish_virtual_scan_message(carmen_mapper_virtual_scan_message *virtual_scan_message, double timestamp)
{
	IPC_RETURN_TYPE err;

	virtual_scan_message->timestamp = timestamp;

	err = IPC_publishData(CARMEN_MAPPER_VIRTUAL_SCAN_MESSAGE_NAME, virtual_scan_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_MAPPER_VIRTUAL_SCAN_MESSAGE_NAME);
}


void
carmen_mapper_publish_map_message(carmen_map_t *carmen_map, double timestamp)
{
	IPC_RETURN_TYPE err;
	static carmen_mapper_map_message mapper_message;

	strcpy(mapper_message.config.origin, "from_mapping");
	mapper_message.complete_map = carmen_map->complete_map;
	mapper_message.size = carmen_map->config.x_size * carmen_map->config.y_size;
	mapper_message.config = carmen_map->config;
	mapper_message.host = carmen_get_host();
	mapper_message.timestamp = timestamp;

	err = IPC_publishData(CARMEN_MAPPER_MAP_MESSAGE_NAME, &mapper_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_MAPPER_MAP_MESSAGE_NAME);
}


void
carmen_mapper_publish_map_level_message(carmen_map_t *carmen_map, double timestamp, int height_level)
{
	IPC_RETURN_TYPE err;
	static carmen_mapper_map_message mapper_message;

	strcpy(mapper_message.config.origin, "from_mapping");
	mapper_message.complete_map = carmen_map->complete_map;
	mapper_message.size = carmen_map->config.x_size * carmen_map->config.y_size;
	mapper_message.config = carmen_map->config;
	mapper_message.host = carmen_get_host();
	mapper_message.timestamp = timestamp;

	char* message_name= "\0";

	switch(height_level)
	{
		case 0:
			message_name = CARMEN_MAPPER_MAP_MESSAGE_NAME;
			break;
		case 1:
			printf("Publishing map level1\n");
			message_name = CARMEN_MAPPER_MAP_LEVEL1_MESSAGE_NAME;
			break;
		default:
			carmen_die("INVALID LAYER HEIGHT - Get this parameter from command line");
	}

	err = IPC_publishData(message_name, &mapper_message);
	carmen_test_ipc_exit(err, "Could not publish", message_name);
}


void
carmen_mapper_moving_objects_raw_map_publish_message(carmen_map_t *carmen_map, double timestamp)
{
	IPC_RETURN_TYPE err;
	static carmen_mapper_map_message mapper_message;

	mapper_message.config = carmen_map->config;
	strcpy(mapper_message.config.origin, "from_mapping");
	mapper_message.complete_map = carmen_map->complete_map;
	mapper_message.size = carmen_map->config.x_size * carmen_map->config.y_size;
	mapper_message.host = carmen_get_host();
	mapper_message.timestamp = timestamp;

	err = IPC_publishData(CARMEN_MAPPER_MAP_MOVING_OBJECTS_RAW_MAP_MESSAGE_NAME, &mapper_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_MAPPER_MAP_MOVING_OBJECTS_RAW_MAP_MESSAGE_NAME);
}

//void
//carmen_mapper_publish_distance_map_message(carmen_mapper_distance_map *distance_map, double timestamp)
//{
//  IPC_RETURN_TYPE err;
//  static carmen_mapper_distance_map_message distance_map_message;
//  static int first_time = 1;
//
//  if (first_time)
//  {
//    err = IPC_defineMsg(CARMEN_MAPPER_DISTANCE_MAP_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
//    					CARMEN_MAPPER_DISTANCE_MAP_MESSAGE_FMT);
//	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAPPER_DISTANCE_MAP_MESSAGE_NAME);
//
//	first_time = 0;
//  }
//
//  distance_map_message.config = distance_map->config;
//  strcpy(distance_map_message.config.origin, "from_mapping");
//  distance_map_message.size = distance_map->config.x_size * distance_map->config.y_size;
//  distance_map_message.complete_x_offset = distance_map->complete_x_offset;
//  distance_map_message.complete_y_offset = distance_map->complete_y_offset;
//  distance_map_message.timestamp = timestamp;
//  distance_map_message.host = carmen_get_host();
//
//  err = IPC_publishData(CARMEN_MAPPER_DISTANCE_MAP_MESSAGE_NAME, &distance_map_message);
//  carmen_test_ipc_exit(err, "Could not publish", CARMEN_MAPPER_DISTANCE_MAP_MESSAGE_NAME);
//}
