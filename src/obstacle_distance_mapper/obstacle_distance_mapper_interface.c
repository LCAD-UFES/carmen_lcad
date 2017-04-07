#include <carmen/carmen.h>
#include <prob_map.h>
#include "obstacle_distance_mapper_messages.h"


void
carmen_obstacle_distance_mapper_subscribe_message(carmen_obstacle_distance_mapper_map_message *message, carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_OBSTACLE_DISTANCE_MAPPER_MAP_MESSAGE_NAME,
	CARMEN_OBSTACLE_DISTANCE_MAPPER_MAP_MESSAGE_FMT, message, sizeof(carmen_obstacle_distance_mapper_map_message), handler, subscribe_how);
}


void
carmen_obstacle_distance_mapper_subscribe_compact_map_message(carmen_obstacle_distance_mapper_compact_map_message *message,
		carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_MAP_MESSAGE_NAME,
	CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_MAP_MESSAGE_FMT, message, sizeof(carmen_obstacle_distance_mapper_compact_map_message), handler, subscribe_how);
}


void
carmen_obstacle_distance_mapper_publish_distance_map_message(carmen_prob_models_distance_map *distance_map, double timestamp)
{
	IPC_RETURN_TYPE err;
	carmen_obstacle_distance_mapper_map_message distance_map_message;
	static int first_time = 1;

	if (first_time)
	{
		err = IPC_defineMsg(CARMEN_OBSTACLE_DISTANCE_MAPPER_MAP_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
		CARMEN_OBSTACLE_DISTANCE_MAPPER_MAP_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define", CARMEN_OBSTACLE_DISTANCE_MAPPER_MAP_MESSAGE_NAME);

		first_time = 0;
	}

	distance_map_message.config = distance_map->config;
	strcpy(distance_map_message.config.origin, "from_mapping");
	distance_map_message.size = distance_map->config.x_size * distance_map->config.y_size;
	distance_map_message.complete_x_offset = distance_map->complete_x_offset;
	distance_map_message.complete_y_offset = distance_map->complete_y_offset;
	distance_map_message.timestamp = timestamp;
	distance_map_message.host = carmen_get_host();

	err = IPC_publishData(CARMEN_OBSTACLE_DISTANCE_MAPPER_MAP_MESSAGE_NAME, &distance_map_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_OBSTACLE_DISTANCE_MAPPER_MAP_MESSAGE_NAME);
}


void
carmen_obstacle_distance_mapper_publish_compact_distance_map_message(carmen_obstacle_distance_mapper_compact_map_message *message, double timestamp)
{
	IPC_RETURN_TYPE err;
	static int first_time = 1;

	if (first_time)
	{
		err = IPC_defineMsg(CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_MAP_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
		CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_MAP_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define", CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_MAP_MESSAGE_NAME);

		first_time = 0;
	}

	message->timestamp = timestamp;
	message->host = carmen_get_host();

	err = IPC_publishData(CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_MAP_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_MAP_MESSAGE_NAME);
}


void
carmen_obstacle_distance_mapper_clear_distance_map_using_compact_map(carmen_prob_models_distance_map *map,
		carmen_obstacle_distance_mapper_compact_map_message *cmap, int value)
{
	for (int i = 0; i < cmap->size; i++)
	{
		map->x_offset[cmap->coord_x[i]][cmap->coord_y[i]] = value;
		map->y_offset[cmap->coord_x[i]][cmap->coord_y[i]] = value;
		map->distance[cmap->coord_x[i]][cmap->coord_y[i]] = (double) value;
	}
}


void
carmen_obstacle_distance_mapper_uncompress_compact_distance_map(carmen_prob_models_distance_map *map,
		carmen_obstacle_distance_mapper_compact_map_message *cmap)
{
	for (int i = 0; i < cmap->size; i++)
	{
		map->x_offset[cmap->coord_x[i]][cmap->coord_y[i]] = cmap->x_offset[i];
		map->y_offset[cmap->coord_x[i]][cmap->coord_y[i]] = cmap->y_offset[i];
		map->distance[cmap->coord_x[i]][cmap->coord_y[i]] = sqrt((double) cmap->x_offset[i] * (double) cmap->x_offset[i] +
				(double) cmap->y_offset[i] * (double) cmap->y_offset[i]);
	}
}


void
carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(carmen_obstacle_distance_mapper_map_message *map,
		carmen_obstacle_distance_mapper_compact_map_message *cmap)
{
	for (int i = 0; i < cmap->size; i++)
	{
		int index = cmap->coord_y[i] + map->config.y_size * cmap->coord_x[i];
		map->complete_x_offset[index] = cmap->x_offset[i];
		map->complete_y_offset[index] = cmap->y_offset[i];
	}
}
