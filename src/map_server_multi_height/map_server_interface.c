/*
 * map_server_interface.c
 *
 *  Created on: 26/09/2012
 *      Author: romulo
 */
#include "map_server_interface.h"
#include "map_server_messages.h"

/*
int
carmen_map_server_get_current_offline_map(carmen_map_t *current_map)
{
	static int firsttime = 1;
	static unsigned int timeout = 5000;
	carmen_grid_mapping_message *response;

	IPC_RETURN_TYPE err;

	if(firsttime)
	{
		err = IPC_defineMsg(CARMEN_MAP_SERVER_REQUEST_CURRENT_OFFLINE_MAP_NAME, IPC_VARIABLE_LENGTH,
				CARMEN_MAP_SERVER_REQUEST_CURRENT_OFFLINE_MAP_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_MAP_SERVER_REQUEST_CURRENT_OFFLINE_MAP_NAME);
		firsttime = 0;
	}

	err = IPC_queryResponseData(CARMEN_MAP_SERVER_REQUEST_CURRENT_OFFLINE_MAP_NAME, carmen_default_message_create(),
			(void **)&response, timeout);

	if (err != IPC_OK)
	{
		carmen_test_ipc(err, "Could not get map_request",
				CARMEN_MAP_SERVER_REQUEST_CURRENT_OFFLINE_MAP_NAME);
		carmen_warn("\nDid you remember to start the map_server?\n");

		current_map->config.x_size = 0;
		current_map->config.y_size = 0;

		return 0;
	}

	current_map->complete_map = response->complete_map;
	current_map->config = response->config;
	current_map->map = (double**) malloc(sizeof(double*) * current_map->config.x_size);

	for(int i = 0; i < current_map->config.x_size; i++)
	{
		current_map->map[i] = &current_map->complete_map[i * current_map->config.y_size];
	}

	return 1;
}
 */


void
carmen_map_server_copy_offline_map_from_message(carmen_map_t *current_map, carmen_map_server_offline_map_message *offline_map_message)
{
	int i;

	if (current_map->complete_map == NULL)
	{
		current_map->complete_map = (double *) malloc(offline_map_message->size * sizeof(double));
		carmen_test_alloc(current_map->complete_map);
		current_map->map = (double **) malloc(offline_map_message->config.x_size * sizeof(double *));
		carmen_test_alloc(current_map->map);
	}
	else if ((current_map->config.x_size != offline_map_message->config.x_size) || (current_map->config.y_size != offline_map_message->config.y_size))
	{
		free(current_map->complete_map);
		free(current_map->map);
		current_map->complete_map = (double *) malloc(offline_map_message->size * sizeof(double));
		carmen_test_alloc(current_map->complete_map);
		current_map->map = (double **) malloc(offline_map_message->config.x_size * sizeof(double *));
		carmen_test_alloc(current_map->map);
	}

	if (current_map->config.map_name != NULL)
		free(current_map->config.map_name);
	current_map->config = offline_map_message->config;
	if (offline_map_message->config.map_name != NULL)
	{
		current_map->config.map_name = (char *) malloc((strlen(offline_map_message->config.map_name) + 1) * sizeof(char));
		carmen_test_alloc(current_map->config.map_name);
		strcpy(current_map->config.map_name, offline_map_message->config.map_name);
	}

	memcpy(current_map->complete_map, offline_map_message->complete_map, sizeof(double) * offline_map_message->size);
	for (i = 0; i < current_map->config.x_size; i++)
		current_map->map[i] = current_map->complete_map + i * current_map->config.y_size;
}


void
carmen_map_server_subscribe_lane_map(
		carmen_map_server_lane_map *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_MAP_SERVER_LANE_MAP_NAME,
			CARMEN_MAP_SERVER_LANE_MAP_FMT,
			message, sizeof(carmen_map_server_lane_map),
			handler, subscribe_how);
}


void
carmen_map_server_subscribe_cost_map(
		carmen_map_server_cost_map *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_MAP_SERVER_COST_MAP_NAME,
			CARMEN_MAP_SERVER_COST_MAP_FMT,
			message, sizeof(carmen_map_server_cost_map),
			handler, subscribe_how);
}

void
carmen_map_server_subscribe_compact_cost_map(
		carmen_map_server_compact_cost_map_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_MAP_SERVER_COMPACT_COST_MAP_NAME,
			CARMEN_MAP_SERVER_COMPACT_COST_MAP_FMT,
			message, sizeof(carmen_map_server_compact_cost_map_message),
			handler, subscribe_how);
}


void
carmen_map_server_subscribe_compact_lane_map(
		carmen_map_server_compact_lane_map_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_MAP_SERVER_COMPACT_LANE_MAP_NAME,
			CARMEN_MAP_SERVER_COMPACT_LANE_MAP_FMT,
			message, sizeof(carmen_map_server_compact_lane_map_message),
			handler, subscribe_how);
}


void
carmen_map_server_subscribe_offline_map(carmen_map_server_offline_map_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_MAP_SERVER_OFFLINE_MAP_NAME,
			CARMEN_MAP_SERVER_OFFLINE_MAP_FMT,
			message, sizeof(carmen_map_server_offline_map_message),
			handler, subscribe_how);
}

void
carmen_map_server_subscribe_offline_map_level1(carmen_map_server_offline_map_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_MAP_SERVER_OFFLINE_MAP_LEVEL1_NAME,
			CARMEN_MAP_SERVER_OFFLINE_MAP_FMT,
			message, sizeof(carmen_map_server_offline_map_message),
			handler, subscribe_how);
}

void
carmen_map_server_subscribe_road_map(carmen_map_server_road_map_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_MAP_SERVER_ROAD_MAP_NAME,
			CARMEN_MAP_SERVER_ROAD_MAP_FMT,
			message, sizeof(carmen_map_server_road_map_message),
			handler, subscribe_how);
}

void
carmen_map_server_define_cost_map_message(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_MAP_SERVER_COST_MAP_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAP_SERVER_COST_MAP_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAP_SERVER_COST_MAP_NAME);
}


void
carmen_map_server_define_compact_cost_map_message(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_MAP_SERVER_COMPACT_COST_MAP_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAP_SERVER_COMPACT_COST_MAP_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAP_SERVER_COMPACT_COST_MAP_NAME);
}


void
carmen_map_server_define_compact_lane_map_message(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_MAP_SERVER_COMPACT_LANE_MAP_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAP_SERVER_COMPACT_LANE_MAP_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAP_SERVER_COMPACT_LANE_MAP_NAME);
}


void
carmen_map_server_define_lane_map_message(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_MAP_SERVER_LANE_MAP_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAP_SERVER_LANE_MAP_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAP_SERVER_LANE_MAP_NAME);
}


void
carmen_map_server_define_offline_map_message(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_MAP_SERVER_OFFLINE_MAP_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAP_SERVER_OFFLINE_MAP_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAP_SERVER_OFFLINE_MAP_NAME);
}

void
carmen_map_server_define_offline_map_level1_message(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_MAP_SERVER_OFFLINE_MAP_LEVEL1_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAP_SERVER_OFFLINE_MAP_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAP_SERVER_OFFLINE_MAP_LEVEL1_NAME);
}

void
carmen_map_server_define_road_map_message(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_MAP_SERVER_ROAD_MAP_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAP_SERVER_ROAD_MAP_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAP_SERVER_ROAD_MAP_NAME);
}

void
carmen_map_server_publish_cost_map_message(carmen_map_t *carmen_map, double timestamp)
{
	IPC_RETURN_TYPE err;
	static carmen_map_server_cost_map msg;

	strcpy(msg.config.origin, "from_mapping");
	msg.complete_map = carmen_map->complete_map;
	msg.size = carmen_map->config.x_size * carmen_map->config.y_size;
	msg.config = carmen_map->config;
	msg.host = carmen_get_host();
	msg.timestamp = timestamp;
//	msg.cost_map_type = cost_map_type;

	err = IPC_publishData(CARMEN_MAP_SERVER_COST_MAP_NAME, &msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_MAP_SERVER_COST_MAP_NAME);
}


void
carmen_map_server_publish_compact_cost_map_message(carmen_compact_map_t *carmen_map, double timestamp)
{
	IPC_RETURN_TYPE err;
	static carmen_map_server_compact_cost_map_message msg;

	strcpy(msg.config.origin, "from_mapping");
	msg.coord_x = carmen_map->coord_x;
	msg.coord_y = carmen_map->coord_y;
	msg.value = carmen_map->value;
	msg.size = carmen_map->number_of_known_points_on_the_map;
	msg.config = carmen_map->config;
	msg.host = carmen_get_host();
	msg.timestamp = timestamp;

	err = IPC_publishData(CARMEN_MAP_SERVER_COMPACT_COST_MAP_NAME, &msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_MAP_SERVER_COMPACT_COST_MAP_NAME);
}


void
carmen_map_server_publish_compact_lane_map_message(carmen_compact_map_t *carmen_map, double timestamp)
{
	IPC_RETURN_TYPE err;
	static carmen_map_server_compact_lane_map_message msg;

	strcpy(msg.config.origin, "from_mapping");
	msg.coord_x = carmen_map->coord_x;
	msg.coord_y = carmen_map->coord_y;
	msg.value = carmen_map->value;
	msg.size = carmen_map->number_of_known_points_on_the_map;
	msg.config = carmen_map->config;
	msg.host = carmen_get_host();
	msg.timestamp = timestamp;

	err = IPC_publishData(CARMEN_MAP_SERVER_COMPACT_LANE_MAP_NAME, &msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_MAP_SERVER_COMPACT_LANE_MAP_NAME);
}


void
carmen_cpy_compact_cost_message_to_compact_map(carmen_compact_map_t* compact_cost_map, carmen_map_server_compact_cost_map_message* message)
{
	compact_cost_map->coord_x = (int*) (malloc(message->size * sizeof(int)));
	compact_cost_map->coord_y = (int*) (malloc(message->size * sizeof(int)));
	compact_cost_map->value = (double*) (malloc(message->size * sizeof(double)));

	memcpy(compact_cost_map->coord_x, message->coord_x,	message->size * sizeof(int));
	memcpy(compact_cost_map->coord_y, message->coord_y,	message->size * sizeof(int));
	memcpy(compact_cost_map->value, message->value,	message->size * sizeof(double));

	compact_cost_map->number_of_known_points_on_the_map = message->size;
}


void
carmen_cpy_compact_lane_message_to_compact_map(carmen_compact_map_t* compact_lane_map, carmen_map_server_compact_lane_map_message* message)
{
	compact_lane_map->coord_x = (int*) (malloc(message->size * sizeof(int)));
	compact_lane_map->coord_y = (int*) (malloc(message->size * sizeof(int)));
	compact_lane_map->value = (double*) (malloc(message->size * sizeof(double)));

	memcpy(compact_lane_map->coord_x, message->coord_x,	message->size * sizeof(int));
	memcpy(compact_lane_map->coord_y, message->coord_y,	message->size * sizeof(int));
	memcpy(compact_lane_map->value, message->value,	message->size * sizeof(double));

	compact_lane_map->number_of_known_points_on_the_map = message->size;
}


void
carmen_map_server_publish_lane_map_message(carmen_map_t *carmen_map, double timestamp)
{
	IPC_RETURN_TYPE err;
	static carmen_map_server_lane_map msg;

	strcpy(msg.config.origin, "from_mapping");
	msg.complete_map = carmen_map->complete_map;
	msg.size = carmen_map->config.x_size * carmen_map->config.y_size;
	msg.config = carmen_map->config;
	msg.host = carmen_get_host();
	msg.timestamp = timestamp;

	err = IPC_publishData(CARMEN_MAP_SERVER_LANE_MAP_NAME, &msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_MAP_SERVER_LANE_MAP_NAME);
}


void
carmen_map_server_publish_offline_map_message(carmen_map_t *carmen_map, double timestamp)
{
	IPC_RETURN_TYPE err;
	static carmen_map_server_offline_map_message msg;

//	strcpy(msg.config.origin, carmen_map->config.origin);
	msg.complete_map = carmen_map->complete_map;
	msg.size = carmen_map->config.x_size * carmen_map->config.y_size;
	msg.config = carmen_map->config;
	msg.host = carmen_get_host();
	msg.timestamp = timestamp;

	err = IPC_publishData(CARMEN_MAP_SERVER_OFFLINE_MAP_NAME, &msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_MAP_SERVER_OFFLINE_MAP_NAME);
}

void
carmen_map_server_publish_offline_multi_height_map_message(carmen_map_t *carmen_map, double timestamp, int level)
{
	IPC_RETURN_TYPE err;
	static carmen_map_server_offline_map_message msg;

//	strcpy(msg.config.origin, carmen_map->config.origin);
	msg.complete_map = carmen_map->complete_map;
	msg.size = carmen_map->config.x_size * carmen_map->config.y_size;
	msg.config = carmen_map->config;
	msg.host = carmen_get_host();
	msg.timestamp = timestamp;

	if (level == 1)
	{
		err = IPC_publishData(CARMEN_MAP_SERVER_OFFLINE_MAP_LEVEL1_NAME, &msg);
		carmen_test_ipc_exit(err, "Could not publish", CARMEN_MAP_SERVER_OFFLINE_MAP_LEVEL1_NAME);
	}
}

void
carmen_map_server_publish_road_map_message(carmen_map_t *carmen_road_map, double timestamp)
{
	IPC_RETURN_TYPE err;
	static carmen_map_server_offline_map_message msg;

//	strcpy(msg.config.origin, carmen_map->config.origin);
	msg.complete_map = carmen_road_map->complete_map;
	msg.size = carmen_road_map->config.x_size * carmen_road_map->config.y_size;
	msg.config = carmen_road_map->config;
	msg.host = carmen_get_host();
	msg.timestamp = timestamp;

	err = IPC_publishData(CARMEN_MAP_SERVER_ROAD_MAP_NAME, &msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_MAP_SERVER_ROAD_MAP_NAME);
}

void
carmen_map_server_define_localize_map_message(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_MAP_SERVER_LOCALIZE_MAP_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_MAP_SERVER_LOCALIZE_MAP_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAP_SERVER_LOCALIZE_MAP_NAME);
}

void
carmen_map_server_subscribe_localize_map_message(
		carmen_map_server_localize_map_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_MAP_SERVER_LOCALIZE_MAP_NAME,
			CARMEN_MAP_SERVER_LOCALIZE_MAP_FMT,
			message, sizeof(carmen_map_server_localize_map_message),
			handler, subscribe_how);
}

void
carmen_map_server_publish_localize_map_message(carmen_localize_ackerman_map_t* localize_map)
{
	static int firsttime = 1;
	IPC_RETURN_TYPE err;
	carmen_map_server_localize_map_message msg;

	if (firsttime)
	{
		carmen_map_server_define_localize_map_message();
		firsttime = 1;
	}

	msg.config = localize_map->config;
	msg.size = localize_map->carmen_map.config.x_size * localize_map->carmen_map.config.y_size;
	msg.complete_mean_remission_map = localize_map->carmen_mean_remission_map.complete_map;
	msg.complete_variance_remission_map = localize_map->carmen_variance_remission_map.complete_map;
	msg.complete_map = localize_map->carmen_map.complete_map;
//	msg.complete_distance = localize_map->complete_distance;
	msg.complete_gprob = localize_map->complete_gprob;
	msg.complete_prob = localize_map->complete_prob;
	//msg.complete_x_offset = localize_map->complete_x_offset;
	//msg.complete_y_offset = localize_map->complete_y_offset;
	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_MAP_SERVER_LOCALIZE_MAP_NAME, &msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_MAP_SERVER_LOCALIZE_MAP_NAME);
}

void
carmen_map_server_localize_map_message_to_localize_map(carmen_map_server_localize_map_message *message, carmen_localize_ackerman_map_t* localize_map)
{
	if (message->config.x_size != localize_map->config.x_size || message->config.y_size != localize_map->config.y_size)
	{
		free(localize_map->carmen_variance_remission_map.complete_map);
		free(localize_map->carmen_mean_remission_map.complete_map);
		free(localize_map->carmen_map.complete_map);
		//free(localize_map->complete_distance);
		free(localize_map->complete_gprob);
		free(localize_map->complete_prob);
		//free(localize_map->complete_x_offset);
		//free(localize_map->complete_y_offset);

		free(localize_map->carmen_variance_remission_map.map);
		free(localize_map->carmen_mean_remission_map.map);
		free(localize_map->carmen_map.map);
		//free(localize_map->distance);
		free(localize_map->gprob);
		free(localize_map->prob);
	//	free(localize_map->x_offset);
	//	free(localize_map->y_offset);

		localize_map->carmen_variance_remission_map.complete_map = NULL;
		localize_map->carmen_mean_remission_map.complete_map = NULL;
		localize_map->carmen_map.complete_map = NULL;
		//localize_map->complete_distance = NULL;
		localize_map->complete_gprob = NULL;
		localize_map->complete_prob = NULL;
		//localize_map->complete_x_offset = NULL;
		//localize_map->complete_y_offset = NULL;

		localize_map->carmen_variance_remission_map.map = NULL;
		localize_map->carmen_mean_remission_map.map = NULL;
		localize_map->carmen_map.map = NULL;
		//localize_map->distance = NULL;
		localize_map->gprob = NULL;
		localize_map->prob = NULL;
		//localize_map->x_offset = NULL;
		//localize_map->y_offset = NULL;
	}

//	if (localize_map->complete_distance == NULL || localize_map->carmen_map.complete_map == NULL)
	if (localize_map->carmen_map.complete_map == NULL)
	{
		localize_map->carmen_variance_remission_map.complete_map = (double*) malloc(sizeof(double) * message->size);
		localize_map->carmen_mean_remission_map.complete_map = (double*) malloc(sizeof(double) * message->size);
		localize_map->carmen_map.complete_map = (double*) malloc(sizeof(double) * message->size);
//		localize_map->complete_distance = (double*) malloc(sizeof(double) * message->size);
		localize_map->complete_gprob = (double*) malloc(sizeof(double) * message->size);
		localize_map->complete_prob = (double*) malloc(sizeof(double) * message->size);
//		localize_map->complete_x_offset = (short int*) malloc(sizeof(short int) * message->size);
//		localize_map->complete_y_offset = (short int*) malloc(sizeof(short int) * message->size);

		localize_map->carmen_variance_remission_map.map = (double**) malloc(sizeof(double*) * message->config.x_size);
		localize_map->carmen_mean_remission_map.map = (double**) malloc(sizeof(double*) * message->config.x_size);
		localize_map->carmen_map.map = (double**) malloc(sizeof(double*) * message->config.x_size);
//		localize_map->distance = (double**) malloc(sizeof(double*) * message->config.x_size);
		localize_map->gprob = (double**) malloc(sizeof(double*) * message->config.x_size);
		localize_map->prob = (double**) malloc(sizeof(double*) * message->config.x_size);
//		localize_map->x_offset = (short int**) malloc(sizeof(short int*) * message->config.x_size);
//		localize_map->y_offset = (short int**) malloc(sizeof(short int*) * message->config.x_size);

		for (int i = 0; i < message->config.x_size; i++)
		{
			localize_map->carmen_variance_remission_map.map[i] = localize_map->carmen_variance_remission_map.complete_map + i * message->config.y_size;
			localize_map->carmen_mean_remission_map.map[i] = localize_map->carmen_mean_remission_map.complete_map + i * message->config.y_size;
			localize_map->carmen_map.map[i] = localize_map->carmen_map.complete_map + i * message->config.y_size;
//			localize_map->distance[i] = localize_map->complete_distance + i * message->config.y_size;
			localize_map->gprob[i] = localize_map->complete_gprob + i * message->config.y_size;
			localize_map->prob[i] = localize_map->complete_prob + i * message->config.y_size;
//			localize_map->x_offset[i] = localize_map->complete_x_offset + i * message->config.y_size;
//			localize_map->y_offset[i] = localize_map->complete_y_offset + i * message->config.y_size;
		}
	}

	localize_map->config = message->config;
	localize_map->carmen_map.config = message->config;
	localize_map->carmen_mean_remission_map.config = message->config;
	localize_map->carmen_variance_remission_map.config = message->config;

	memcpy(localize_map->carmen_variance_remission_map.complete_map, message->complete_variance_remission_map, sizeof(double) * message->size);
	memcpy(localize_map->carmen_mean_remission_map.complete_map, message->complete_mean_remission_map, sizeof(double) * message->size);
	memcpy(localize_map->carmen_map.complete_map, message->complete_map, sizeof(double) * message->size);
//	memcpy(localize_map->complete_distance, message->complete_distance, sizeof(double) * message->size);
	memcpy(localize_map->complete_gprob, message->complete_gprob, sizeof(double) * message->size);
	memcpy(localize_map->complete_prob, message->complete_prob, sizeof(double) * message->size);
//	memcpy(localize_map->complete_x_offset, message->complete_x_offset, sizeof(short int) * message->size);
//	memcpy(localize_map->complete_y_offset, message->complete_y_offset, sizeof(short int) * message->size);
}
