#include <carmen/carmen.h>
#include "obstacle_distance_mapper_messages.h"

void
carmen_grid_mapping_distance_map_subscribe_message(carmen_grid_mapping_distance_map_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_GRID_MAPPING_DISTANCE_MAP_MESSAGE_NAME,
		  	  	  	  	  CARMEN_GRID_MAPPING_DISTANCE_MAP_MESSAGE_FMT,
                          message, sizeof(carmen_grid_mapping_distance_map_message),
                          handler, subscribe_how);
}

void
carmen_grid_mapping_publish_distance_map_message(carmen_grid_mapping_distance_map *distance_map, double timestamp)
{
  IPC_RETURN_TYPE err;
  static carmen_grid_mapping_distance_map_message distance_map_message;
  static int first_time = 1;

  if (first_time)
  {
    err = IPC_defineMsg(CARMEN_GRID_MAPPING_DISTANCE_MAP_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
    					CARMEN_GRID_MAPPING_DISTANCE_MAP_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_GRID_MAPPING_DISTANCE_MAP_MESSAGE_NAME);

	first_time = 0;
  }

  distance_map_message.config = distance_map->config;
  strcpy(distance_map_message.config.origin, "from_mapping");
  distance_map_message.size = distance_map->config.x_size * distance_map->config.y_size;
  distance_map_message.complete_x_offset = distance_map->complete_x_offset;
  distance_map_message.complete_y_offset = distance_map->complete_y_offset;
  distance_map_message.timestamp = timestamp;
  distance_map_message.host = carmen_get_host();

  err = IPC_publishData(CARMEN_GRID_MAPPING_DISTANCE_MAP_MESSAGE_NAME, &distance_map_message);
  carmen_test_ipc_exit(err, "Could not publish", CARMEN_GRID_MAPPING_DISTANCE_MAP_MESSAGE_NAME);
}
