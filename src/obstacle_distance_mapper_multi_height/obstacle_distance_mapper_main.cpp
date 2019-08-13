#include <carmen/carmen.h>
#include "obstacle_distance_mapper_interface.h"
#include <carmen/mapper_interface.h>

#include <prob_measurement_model.h>
#include <prob_transforms.h>
#include <prob_map.h>
#include <iostream>
#include <string.h>

#include <carmen/behavior_selector_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/simulator_ackerman_interface.h>
#include <carmen/navigator_ackerman_interface.h>

double obstacle_probability_threshold 	= 0.5;
double obstacle_cost_distance 			= 1.0;

carmen_map_t 										map;
carmen_prob_models_distance_map 					distance_map;
carmen_obstacle_distance_mapper_compact_map_message compact_distance_map;
carmen_obstacle_distance_mapper_compact_map_message compact_lane_contents;
carmen_map_t 										cost_map;
carmen_compact_map_t 								compacted_cost_map;
carmen_map_server_compact_lane_map_message			*compact_lane_map = NULL;
carmen_obstacle_distance_mapper_compact_map_message *behaviour_selector_compact_lane_contents_message = NULL;

carmen_point_t g_goal_position;
carmen_point_t g_robot_position;
carmen_point_t goal_list_message;

int height_level;

using namespace std;


static void
build_obstacle_cost_map(carmen_map_t *cost_map, carmen_map_config_t config, carmen_prob_models_distance_map *distance_map,
		double distance_for_zero_cost)
{
	carmen_prob_models_initialize_cost_map(cost_map, config, config.resolution);

	double resolution = distance_map->config.resolution;
	for (int x = 0; x < distance_map->config.x_size; x++)
	{
		for (int y = 0; y < distance_map->config.y_size; y++)
		{
			double distance = distance_map->distance[x][y] * resolution;
			cost_map->map[x][y] = (distance > distance_for_zero_cost)? 0.0: 1.0 - (distance / distance_for_zero_cost);
		}
	}
}


static void
build_distance_map(carmen_mapper_map_message *map_message)
{
	if (distance_map.complete_distance == NULL)
		carmen_prob_models_initialize_distance_map(&distance_map, map_message->config);

	carmen_map_t occupancy_map;
	occupancy_map.config = map_message->config;
	occupancy_map.complete_map = map_message->complete_map;
	static double** occupancy_map_map = NULL;
	if (!occupancy_map_map)
		occupancy_map_map = (double **) (malloc(occupancy_map.config.x_size * sizeof(double *)));

	occupancy_map.map = occupancy_map_map;
	for (int i = 0; i < occupancy_map.config.x_size; i++)
		occupancy_map.map[i] = occupancy_map.complete_map + i * occupancy_map.config.y_size;

	carmen_prob_models_create_distance_map(&distance_map, &occupancy_map, obstacle_probability_threshold);
}


carmen_obstacle_distance_mapper_compact_map_message
clear_lane_in_distance_map()
{
	carmen_obstacle_distance_mapper_compact_map_message lane_contents;

	if (compact_lane_map != NULL)
	{
		lane_contents.config = distance_map.config;
		// Aloca o minimo necessario, mas pode nao precisar de tudo, ja que os mapas podem ter origens diferentes e a lane vazar o distance_map
		lane_contents.coord_x = (short int *) malloc(compact_lane_map->size * sizeof(short int));
		lane_contents.coord_y = (short int *) malloc(compact_lane_map->size * sizeof(short int));
		lane_contents.x_offset = (char *) malloc(compact_lane_map->size * sizeof(char));
		lane_contents.y_offset = (char *) malloc(compact_lane_map->size * sizeof(char));
		int k = 0;
		for (int i = 0; i < compact_lane_map->size; i++)
		{
			cell_coords_t map_cell = carmen_obstacle_distance_mapper_get_map_cell_from_configs(distance_map.config, compact_lane_map->config,
					compact_lane_map->coord_x[i], compact_lane_map->coord_y[i]);
			if ((map_cell.x >= 0) && (map_cell.x < distance_map.config.x_size) && (map_cell.y >= 0) && (map_cell.y < distance_map.config.y_size))
			{
				lane_contents.coord_x[k] = map_cell.x;
				lane_contents.coord_y[k] = map_cell.y;
				lane_contents.x_offset[k] = distance_map.x_offset[map_cell.x][map_cell.y];
				lane_contents.y_offset[k] = distance_map.y_offset[map_cell.x][map_cell.y];
				distance_map.x_offset[map_cell.x][map_cell.y] = DISTANCE_MAP_HUGE_DISTANCE;
				distance_map.y_offset[map_cell.x][map_cell.y] = DISTANCE_MAP_HUGE_DISTANCE;
				distance_map.distance[map_cell.x][map_cell.y] = (double) DISTANCE_MAP_HUGE_DISTANCE * 1.414213562; // DISTANCE_MAP_HUGE_DISTANCE * raiz de 2.0
				k++;
			}
		}
		lane_contents.size = k;
	}
	else
	{
		lane_contents.config = distance_map.config;
		lane_contents.size = 0;
		lane_contents.coord_x = NULL;
		lane_contents.coord_y = NULL;
		lane_contents.x_offset = NULL;
		lane_contents.y_offset = NULL;
	}

	return (lane_contents);
}


void
free_compact_lane_contents()
{
	if (compact_lane_contents.coord_x != NULL)
		free(compact_lane_contents.coord_x);
	if (compact_lane_contents.coord_y != NULL)
		free(compact_lane_contents.coord_y);
	if (compact_lane_contents.x_offset != NULL)
		free(compact_lane_contents.x_offset);
	if (compact_lane_contents.y_offset != NULL)
		free(compact_lane_contents.y_offset);
}
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
obstacle_distance_mapper_publish_distance_map(carmen_mapper_map_message *map_message)
{
	build_distance_map(map_message);
	carmen_obstacle_distance_mapper_publish_multi_height_distance_map_message(&distance_map, map_message->timestamp, height_level);
}


static void
obstacle_distance_mapper_publish_compact_distance_and_compact_lane_contents_maps(carmen_mapper_map_message *map_message)
{
	carmen_obstacle_distance_mapper_create_compact_distance_map(&compact_distance_map, &distance_map, DISTANCE_MAP_HUGE_DISTANCE);

	if (height_level)
	{
		carmen_obstacle_distance_mapper_publish_multi_height_compact_distance_map_message(&compact_distance_map, map_message->timestamp, height_level);
	}
	else
	{
		compact_lane_contents = clear_lane_in_distance_map();
		carmen_obstacle_distance_mapper_publish_compact_distance_map_message(&compact_distance_map, map_message->timestamp);
		carmen_obstacle_distance_mapper_publish_compact_lane_contents_message(&compact_lane_contents, map_message->timestamp);
		free_compact_lane_contents();
	}
//	carmen_obstacle_distance_mapper_free_compact_distance_map(&compact_distance_map);
}


void
obstacle_distance_mapper_publish_compact_cost_map_test_mode(double timestamp, carmen_obstacle_distance_mapper_compact_map_message *message)
{
	static carmen_obstacle_distance_mapper_compact_map_message *compact_distance_map = NULL;
	static carmen_obstacle_distance_mapper_map_message distance_map;

	if (compact_distance_map == NULL)
	{
		carmen_obstacle_distance_mapper_create_new_map(&distance_map, message->config, message->host, message->timestamp);
		compact_distance_map = (carmen_obstacle_distance_mapper_compact_map_message *) (calloc(1, sizeof(carmen_obstacle_distance_mapper_compact_map_message)));
		carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(compact_distance_map, message);
		carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(&distance_map, message);
	}
	else
	{
		carmen_obstacle_distance_mapper_clear_distance_map_message_using_compact_map(&distance_map, compact_distance_map, DISTANCE_MAP_HUGE_DISTANCE);
		carmen_obstacle_distance_mapper_free_compact_distance_map(compact_distance_map);
		carmen_obstacle_distance_mapper_cpy_compact_map_message_to_compact_map(compact_distance_map, message);
		carmen_obstacle_distance_mapper_uncompress_compact_distance_map_message(&distance_map, message);
	}

	carmen_prob_models_distance_map real_distance_map;
	carmen_obstacle_distance_mapper_create_distance_map_from_distance_map_message(&real_distance_map, &distance_map);

	build_obstacle_cost_map(&cost_map, real_distance_map.config, &real_distance_map, obstacle_cost_distance);
	carmen_prob_models_create_compact_map(&compacted_cost_map, &cost_map, 0.0);

	if (compacted_cost_map.number_of_known_points_on_the_map > 0)
	{
		carmen_map_server_publish_compact_cost_map_message(&compacted_cost_map,	timestamp);
//		carmen_prob_models_clear_carmen_map_using_compact_map(&cost_map, &compacted_cost_map, 0.0);
		carmen_prob_models_free_compact_map(&compacted_cost_map);
	}

	carmen_obstacle_distance_mapper_free_distance_map(&real_distance_map);
}


void
obstacle_distance_mapper_publish_compact_cost_map(double timestamp)
{
	build_obstacle_cost_map(&cost_map, distance_map.config, &distance_map, obstacle_cost_distance);
	carmen_prob_models_create_compact_map(&compacted_cost_map, &cost_map, 0.0);
	if (compacted_cost_map.number_of_known_points_on_the_map > 0)
	{
		carmen_map_server_publish_multi_height_compact_cost_map_message(&compacted_cost_map, timestamp, height_level);
//		carmen_prob_models_clear_carmen_map_using_compact_map(&cost_map, &compacted_cost_map, 0.0);
		carmen_prob_models_free_compact_map(&compacted_cost_map);

	}
}
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
carmen_mapper_map_message_handler(carmen_mapper_map_message *msg)
{
//	obstacle_distance_mapper_publish_distance_map(msg);
	build_distance_map(msg);
	if (height_level ==0)
		obstacle_distance_mapper_publish_compact_cost_map(msg->timestamp);
	obstacle_distance_mapper_publish_compact_distance_and_compact_lane_contents_maps(msg);
//	obstacle_distance_mapper_publish_compact_cost_map(msg->timestamp, &compact_distance_map);

	carmen_obstacle_distance_mapper_free_compact_distance_map(&compact_distance_map); // teste: remover depois e usar o em obstacle_distance_mapper_publish_compact_distance_map()

	//	static double last_timestamp = 0.0;
	//	double timestamp = carmen_get_time();
	//	printf("delta_t %lf\n", (1/(timestamp - last_timestamp)));
	//	last_timestamp = timestamp;
}


static void
map_server_compact_lane_map_message_handler(carmen_map_server_compact_lane_map_message *message)
{
	compact_lane_map = message;
}


static void
carmen_behaviour_selector_compact_lane_contents_message_handler(carmen_obstacle_distance_mapper_compact_map_message *message)
{
	behaviour_selector_compact_lane_contents_message = message;
}


static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("obstacle_distance_mapper: disconnected.\n");

		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Initialization                                                                            //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
register_handlers()
{
	switch (height_level)
	{
		case 1:
			carmen_mapper_subscribe_map_level1_message(NULL,
					(carmen_handler_t) carmen_mapper_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
			break;
		default:
			carmen_mapper_subscribe_map_message(NULL,
					(carmen_handler_t) carmen_mapper_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
	}
	carmen_map_server_subscribe_compact_lane_map(NULL,
			(carmen_handler_t) map_server_compact_lane_map_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_behaviour_selector_subscribe_compact_lane_contents_message(NULL,
			(carmen_handler_t) carmen_behaviour_selector_compact_lane_contents_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


static int
read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
			{(char *) "rrt",	(char *) "obstacle_cost_distance",			CARMEN_PARAM_DOUBLE,	&obstacle_cost_distance,			1, NULL},
			{(char *) "rrt",	(char *) "obstacle_probability_threshold",	CARMEN_PARAM_DOUBLE,	&obstacle_probability_threshold,	1, NULL}
	};

	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	carmen_param_allow_unfound_variables(1);

	carmen_param_t param_optional_list[] =
	{
		{(char *) "commandline", (char *) "height_level", CARMEN_PARAM_INT, &height_level, 0, NULL},
	};

	carmen_param_install_params(argc, argv, param_optional_list, sizeof(param_optional_list) / sizeof(param_optional_list[0]));

	carmen_param_allow_unfound_variables(0);

	if (height_level < 0 || height_level > 1)
	{
		carmen_die("Invalid map Height level (%d). Valid range 0-1", height_level);
	}

	return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////


int 
main(int argc, char **argv) 
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_module);

	read_parameters(argc, argv);
	register_handlers();

	carmen_ipc_dispatch();

	return (0);
}
