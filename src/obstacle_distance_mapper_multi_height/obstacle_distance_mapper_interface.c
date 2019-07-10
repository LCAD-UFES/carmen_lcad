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
carmen_obstacle_distance_mapper_subscribe_multi_height_map_message(carmen_obstacle_distance_mapper_map_message *message, carmen_handler_t handler,
		carmen_subscribe_t subscribe_how, int height_level)
{
	carmen_subscribe_message(CARMEN_OBSTACLE_DISTANCE_MAPPER_MULTI_HEIGHT_MAP_MESSAGE_NAME(height_level),
	CARMEN_OBSTACLE_DISTANCE_MAPPER_MULTI_HEIGHT_MAP_MESSAGE_FMT, message, sizeof(carmen_obstacle_distance_mapper_map_message), handler, subscribe_how);
}


void
carmen_obstacle_distance_mapper_subscribe_compact_map_message(carmen_obstacle_distance_mapper_compact_map_message *message,
		carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_MAP_MESSAGE_NAME,
	CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_MAP_MESSAGE_FMT, message, sizeof(carmen_obstacle_distance_mapper_compact_map_message), handler, subscribe_how);
}


void
carmen_obstacle_distance_mapper_subscribe_multi_height_compact_map_message(carmen_obstacle_distance_mapper_compact_map_message *message,
		carmen_handler_t handler, carmen_subscribe_t subscribe_how, int height_level)
{
	carmen_subscribe_message(CARMEN_OBSTACLE_DISTANCE_MAPPER_MULTI_HEIGHT_COMPACT_MAP_MESSAGE_NAME(height_level),
	CARMEN_OBSTACLE_DISTANCE_MAPPER_MULTI_HEIGHT_COMPACT_MAP_MESSAGE_FMT, message, sizeof(carmen_obstacle_distance_mapper_compact_map_message), handler, subscribe_how);
}


void
carmen_obstacle_distance_mapper_subscribe_compact_lane_contents_message(carmen_obstacle_distance_mapper_compact_map_message *message,
		carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_LANE_CONTENTS_MESSAGE_NAME,
			CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_LANE_CONTENTS_MESSAGE_FMT, message, sizeof(carmen_obstacle_distance_mapper_compact_map_message), handler, subscribe_how);
}


void
carmen_behaviour_selector_subscribe_compact_lane_contents_message(carmen_obstacle_distance_mapper_compact_map_message *message,
		carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_BEHAVIOUR_SELECTOR_COMPACT_LANE_CONTENTS_MESSAGE_NAME,
			CARMEN_BEHAVIOUR_SELECTOR_COMPACT_LANE_CONTENTS_MESSAGE_FMT, message, sizeof(carmen_obstacle_distance_mapper_compact_map_message), handler, subscribe_how);
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
carmen_obstacle_distance_mapper_publish_multi_height_distance_map_message(carmen_prob_models_distance_map *distance_map, double timestamp, int height_level)
{

	IPC_RETURN_TYPE err;
	carmen_obstacle_distance_mapper_map_message distance_map_message;
	static int first_time = 1;

	if (first_time)
	{
		err = IPC_defineMsg(CARMEN_OBSTACLE_DISTANCE_MAPPER_MULTI_HEIGHT_MAP_MESSAGE_NAME(height_level), IPC_VARIABLE_LENGTH,
		CARMEN_OBSTACLE_DISTANCE_MAPPER_MULTI_HEIGHT_MAP_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define", CARMEN_OBSTACLE_DISTANCE_MAPPER_MULTI_HEIGHT_MAP_MESSAGE_NAME(height_level));

		first_time = 0;
	}

	distance_map_message.config = distance_map->config;
	strcpy(distance_map_message.config.origin, "from_mapping");
	distance_map_message.size = distance_map->config.x_size * distance_map->config.y_size;
	distance_map_message.complete_x_offset = distance_map->complete_x_offset;
	distance_map_message.complete_y_offset = distance_map->complete_y_offset;
	distance_map_message.timestamp = timestamp;
	distance_map_message.host = carmen_get_host();

	err = IPC_publishData(CARMEN_OBSTACLE_DISTANCE_MAPPER_MULTI_HEIGHT_MAP_MESSAGE_NAME(height_level), &distance_map_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_OBSTACLE_DISTANCE_MAPPER_MULTI_HEIGHT_MAP_MESSAGE_NAME(height_level));
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
carmen_obstacle_distance_mapper_publish_multi_height_compact_distance_map_message(carmen_obstacle_distance_mapper_compact_map_message *message, double timestamp, int height_level)
{
	IPC_RETURN_TYPE err;
	static int first_time = 1;

	if (first_time)
	{
		err = IPC_defineMsg(CARMEN_OBSTACLE_DISTANCE_MAPPER_MULTI_HEIGHT_COMPACT_MAP_MESSAGE_NAME(height_level), IPC_VARIABLE_LENGTH,
		CARMEN_OBSTACLE_DISTANCE_MAPPER_MULTI_HEIGHT_COMPACT_MAP_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define", CARMEN_OBSTACLE_DISTANCE_MAPPER_MULTI_HEIGHT_COMPACT_MAP_MESSAGE_NAME(height_level));

		first_time = 0;
	}

	message->timestamp = timestamp;
	message->host = carmen_get_host();

	err = IPC_publishData(CARMEN_OBSTACLE_DISTANCE_MAPPER_MULTI_HEIGHT_COMPACT_MAP_MESSAGE_NAME(height_level), message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_OBSTACLE_DISTANCE_MAPPER_MULTI_HEIGHT_COMPACT_MAP_MESSAGE_NAME(height_level));
}


void
carmen_obstacle_distance_mapper_publish_compact_lane_contents_message(carmen_obstacle_distance_mapper_compact_map_message *message, double timestamp)
{
	IPC_RETURN_TYPE err;
	static int first_time = 1;

	if (first_time)
	{
		err = IPC_defineMsg(CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_LANE_CONTENTS_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
				CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_LANE_CONTENTS_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define", CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_LANE_CONTENTS_MESSAGE_NAME);

		first_time = 0;
	}

	message->timestamp = timestamp;
	message->host = carmen_get_host();

	err = IPC_publishData(CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_LANE_CONTENTS_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_OBSTACLE_DISTANCE_MAPPER_COMPACT_LANE_CONTENTS_MESSAGE_NAME);
}


void
carmen_behaviour_selector_publish_compact_lane_contents_message(carmen_obstacle_distance_mapper_compact_map_message *message, double timestamp)
{
	IPC_RETURN_TYPE err;
	static int first_time = 1;

	if (first_time)
	{
		err = IPC_defineMsg(CARMEN_BEHAVIOUR_SELECTOR_COMPACT_LANE_CONTENTS_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
				CARMEN_BEHAVIOUR_SELECTOR_COMPACT_LANE_CONTENTS_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define", CARMEN_BEHAVIOUR_SELECTOR_COMPACT_LANE_CONTENTS_MESSAGE_NAME);

		first_time = 0;
	}

	message->timestamp = timestamp;
	message->host = carmen_get_host();

	err = IPC_publishData(CARMEN_BEHAVIOUR_SELECTOR_COMPACT_LANE_CONTENTS_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOUR_SELECTOR_COMPACT_LANE_CONTENTS_MESSAGE_NAME);
}


void
carmen_obstacle_distance_mapper_create_new_map(carmen_obstacle_distance_mapper_map_message *distance_map,
		carmen_map_config_t config, char *host, double timestamp)
{
	distance_map->config = config;
	distance_map->size = config.x_size * config.y_size;
	distance_map->complete_x_offset = (short int *) malloc(distance_map->size * sizeof(short int));
	distance_map->complete_y_offset = (short int *) malloc(distance_map->size * sizeof(short int));

	for (int i = 0; i < distance_map->size; i++)
	{
		distance_map->complete_x_offset[i] = distance_map->complete_y_offset[i] = DISTANCE_MAP_HUGE_DISTANCE;
	}

	distance_map->timestamp = timestamp;
	distance_map->host = host;
}


cell_coords_t
carmen_obstacle_distance_mapper_get_map_cell_from_configs(carmen_map_config_t distance_map_config, carmen_map_config_t compact_lane_map_config,
		short int x, short int y)
{
	cell_coords_t map_cell;

	if ((distance_map_config.x_origin == compact_lane_map_config.x_origin) &&
		(distance_map_config.y_origin == compact_lane_map_config.y_origin))
	{
		map_cell.x = x;
		map_cell.y = y;
		return (map_cell);
	}
	else
	{
		double world_x = (double) x * compact_lane_map_config.resolution + compact_lane_map_config.x_origin;
		double world_y = (double) y * compact_lane_map_config.resolution + compact_lane_map_config.y_origin;
		map_cell.x = round((world_x - distance_map_config.x_origin) / distance_map_config.resolution);
		map_cell.y = round((world_y - distance_map_config.y_origin) / distance_map_config.resolution);
		return (map_cell);
	}
}


void
carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(carmen_obstacle_distance_mapper_compact_map_message *compact_map,
		carmen_obstacle_distance_mapper_compact_map_message *message)
{
	compact_map->config = message->config;

	compact_map->coord_x = (short int *) (malloc(message->size * sizeof(short int)));
	compact_map->coord_y = (short int *) (malloc(message->size * sizeof(short int)));
	compact_map->x_offset = (char *) (malloc(message->size * sizeof(char)));
	compact_map->y_offset = (char *) (malloc(message->size * sizeof(char)));

	memcpy(compact_map->coord_x, message->coord_x, message->size * sizeof(short int));
	memcpy(compact_map->coord_y, message->coord_y, message->size * sizeof(short int));
	memcpy(compact_map->x_offset, message->x_offset, message->size * sizeof(char));
	memcpy(compact_map->y_offset, message->y_offset, message->size * sizeof(char));

	compact_map->size = message->size;
	compact_map->timestamp = message->timestamp;
	compact_map->host = message->host;
}


void
carmen_obstacle_distance_mapper_clear_distance_map_using_compact_map(carmen_prob_models_distance_map *map,
		carmen_obstacle_distance_mapper_compact_map_message *cmap, int value)
{	// os dois configs tem que ter origens iguais para esta funcao funcionar
	for (int i = 0; i < cmap->size; i++)
	{
		map->x_offset[cmap->coord_x[i]][cmap->coord_y[i]] = value;
		map->y_offset[cmap->coord_x[i]][cmap->coord_y[i]] = value;
		map->distance[cmap->coord_x[i]][cmap->coord_y[i]] = (double) value;
	}
}


void
carmen_obstacle_distance_mapper_clear_distance_map_message_using_compact_map(carmen_obstacle_distance_mapper_map_message *map,
		carmen_obstacle_distance_mapper_compact_map_message *cmap, int value)
{	// os dois configs tem que ter origens iguais para esta funcao funcionar
	for (int i = 0; i < cmap->size; i++)
	{
		int index = cmap->coord_y[i] + map->config.y_size * cmap->coord_x[i];
		map->complete_x_offset[index] = value;
		map->complete_y_offset[index] = value;
	}
}


void
carmen_obstacle_distance_mapper_uncompress_compact_distance_map(carmen_prob_models_distance_map *map,
		carmen_obstacle_distance_mapper_compact_map_message *cmap)
{	// Esta funcao assume que map esta vazio
	map->config = cmap->config;

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
{	// Esta funcao assume que map esta vazio
	map->config = cmap->config;

	for (int i = 0; i < cmap->size; i++)
	{
		int index = cmap->coord_y[i] + map->config.y_size * cmap->coord_x[i];
		map->complete_x_offset[index] = cmap->x_offset[i];
		map->complete_y_offset[index] = cmap->y_offset[i];
	}
}


void
carmen_obstacle_distance_mapper_overwrite_distance_map_message_with_compact_distance_map(carmen_obstacle_distance_mapper_map_message *map,
		carmen_obstacle_distance_mapper_compact_map_message *cmap)
{
	for (int i = 0; i < cmap->size; i++)
	{
		cell_coords_t map_cell = carmen_obstacle_distance_mapper_get_map_cell_from_configs(map->config, cmap->config,
				cmap->coord_x[i], cmap->coord_y[i]);
		if ((map_cell.x >= 0) && (map_cell.x < map->config.x_size) && (map_cell.y >= 0) && (map_cell.y < map->config.y_size))
		{
			int index = map_cell.y + map->config.y_size * map_cell.x;
			map->complete_x_offset[index] = cmap->x_offset[i];
			map->complete_y_offset[index] = cmap->y_offset[i];
		}
	}
}


void
carmen_obstacle_distance_mapper_overwrite_distance_map_with_compact_distance_map(carmen_prob_models_distance_map *map,
		carmen_obstacle_distance_mapper_compact_map_message *cmap)
{
	for (int i = 0; i < cmap->size; i++)
	{
		cell_coords_t map_cell = carmen_obstacle_distance_mapper_get_map_cell_from_configs(map->config, cmap->config,
				cmap->coord_x[i], cmap->coord_y[i]);
		if ((map_cell.x >= 0) && (map_cell.x < map->config.x_size) && (map_cell.y >= 0) && (map_cell.y < map->config.y_size))
		{
			int index = map_cell.y + map->config.y_size * map_cell.x;
			map->complete_x_offset[index] = cmap->x_offset[i];
			map->complete_y_offset[index] = cmap->y_offset[i];
			map->complete_distance[index] = sqrt((double) cmap->x_offset[i] * (double) cmap->x_offset[i] +
					(double) cmap->y_offset[i] * (double) cmap->y_offset[i]);
		}
	}
}


static int
count_number_of_known_point_on_the_map(carmen_prob_models_distance_map *map, int value)
{
	int count = 0;
	int number_of_cells = map->config.x_size * map->config.y_size;

	for (int i = 0; i < number_of_cells; i++)
	{
		if ((map->complete_x_offset[i] != value) && (map->complete_y_offset[i] != value))
			count++;
	}

	return (count);
}


void
carmen_obstacle_distance_mapper_create_compact_distance_map(carmen_obstacle_distance_mapper_compact_map_message *cmap,
		carmen_prob_models_distance_map *map, int value)
{
	cmap->config = map->config;
	cmap->size = count_number_of_known_point_on_the_map(map, value);

	if (map->config.map_name != NULL)
	{
		cmap->config.map_name = (char *) calloc(strlen(map->config.map_name) + 1, sizeof(char));
		strcpy(cmap->config.map_name, map->config.map_name);
	}

	if (cmap->size == 0)
	{
		cmap->coord_x = NULL;
		cmap->coord_y = NULL;
		cmap->x_offset = NULL;
		cmap->y_offset = NULL;

		return;
	}

	cmap->coord_x = (short int *) malloc(cmap->size * sizeof(short int));
	cmap->coord_y = (short int *) malloc(cmap->size * sizeof(short int));
	cmap->x_offset = (char *) malloc(cmap->size * sizeof(char));
	cmap->y_offset = (char *) malloc(cmap->size * sizeof(char));

	int i, k;
	for (i = 0, k = 0; i < map->config.x_size * map->config.y_size; i++)
	{
		if ((map->complete_x_offset[i] != value) && (map->complete_y_offset[i] != value))
		{
			cmap->coord_x[k] = i / map->config.x_size;
			cmap->coord_y[k] = i % map->config.y_size;
			cmap->x_offset[k] = map->complete_x_offset[i];
			cmap->y_offset[k] = map->complete_y_offset[i];
			k++;
		}
	}
}


void
carmen_obstacle_distance_mapper_free_compact_distance_map(carmen_obstacle_distance_mapper_compact_map_message *map)
{
	if (map->coord_x != NULL)
		free(map->coord_x);

	if (map->coord_y != NULL)
		free(map->coord_y);

	if (map->x_offset != NULL)
		free(map->x_offset);

	if (map->y_offset != NULL)
		free(map->y_offset);

	if (map->config.map_name != NULL)
		free(map->config.map_name);

	map->coord_x = NULL;
	map->coord_y = NULL;
	map->x_offset = NULL;
	map->y_offset = NULL;
	map->config.map_name = NULL;
	map->size = 0;
}


void
carmen_obstacle_distance_mapper_create_distance_map_from_distance_map_message(carmen_prob_models_distance_map *map,
		carmen_obstacle_distance_mapper_map_message *message)
{
	map->config = message->config;

	int size = map->config.x_size * map->config.y_size;
	map->complete_x_offset = (short int *) malloc(size * sizeof(short int));
	map->complete_y_offset = (short int *) malloc(size * sizeof(short int));
	map->complete_distance = (double *) malloc(size * sizeof(double));
	memcpy(map->complete_x_offset, message->complete_x_offset, size * sizeof(short int));
	memcpy(map->complete_y_offset, message->complete_y_offset, size * sizeof(short int));
	for (int i = 0; i < size; i++)
		map->complete_distance[i] = sqrt((double) message->complete_x_offset[i] * (double) message->complete_x_offset[i] +
				(double) message->complete_y_offset[i] * (double) message->complete_y_offset[i]);

	map->distance = (double **) calloc(map->config.x_size, sizeof(double *));
	map->x_offset = (short int **) calloc(map->config.x_size, sizeof(short int *));
	map->y_offset = (short int **) calloc(map->config.x_size, sizeof(short int *));
	for (int i = 0; i < map->config.x_size; i++)
	{
		map->distance[i] = map->complete_distance + i * map->config.y_size;
		map->x_offset[i] = map->complete_x_offset + i * map->config.y_size;
		map->y_offset[i] = map->complete_y_offset + i * map->config.y_size;
	}
}


void
carmen_obstacle_distance_mapper_free_distance_map(carmen_prob_models_distance_map *map)
{
	if (map->complete_distance != NULL)
		free(map->complete_distance);
	if (map->complete_x_offset != NULL)
		free(map->complete_x_offset);
	if (map->complete_y_offset != NULL)
		free(map->complete_y_offset);
	if (map->distance != NULL)
		free(map->distance);
	if (map->x_offset != NULL)
		free(map->x_offset);
	if (map->y_offset != NULL)
		free(map->y_offset);
//	if (map->config.map_name != NULL)
//		free(map->config.map_name);
}
