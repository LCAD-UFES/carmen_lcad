#include <carmen/carmen.h>
#include "obstacle_distance_mapper_interface.h"
#include <carmen/mapper_interface.h>

#include <prob_measurement_model.h>
#include <prob_transforms.h>
#include <prob_map.h>
#include <iostream>
#include <string.h>


double obstacle_probability_threshold 	= 0.5;
double obstacle_cost_distance 			= 1.0;

carmen_map_t 						 map;
carmen_map_t 						 cost_map;
carmen_prob_models_distance_map 	 distance_map;

using namespace std;


void
carmen_mapper_build_obstacle_cost_map(carmen_map_t *cost_map, carmen_map_t *map, carmen_prob_models_distance_map *distance_map, double distance_for_zero_cost_in_pixels)
{
	carmen_prob_models_initialize_cost_map(cost_map, map, map->config.resolution);

	double resolution = distance_map->config.resolution;
	for (int x = 0; x < distance_map->config.x_size; x++)
	{
		for (int y = 0; y < distance_map->config.y_size; y++)
		{
			double distance = distance_map->distance[x][y] * resolution;
			cost_map->map[x][y] = (distance > distance_for_zero_cost_in_pixels)? 0.0: 1.0 - (distance / distance_for_zero_cost_in_pixels);
		}
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void
mapper_publish_distance_map(double timestamp, double obstacle_probability_threshold)
{
	if (distance_map.complete_distance == NULL)
		carmen_prob_models_initialize_distance_map(&distance_map, &map);

	carmen_prob_models_create_distance_map(&distance_map, &map, obstacle_probability_threshold);
	carmen_obstacle_distance_mapper_publish_distance_map_message(&distance_map, timestamp);
}

///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void
carmen_mapper_map_handler(carmen_mapper_map_message *msg)
{
	carmen_compact_map_t compacted_cost_map;

	carmen_mapper_copy_map_from_message(&map, msg);

	mapper_publish_distance_map(msg->timestamp, obstacle_probability_threshold);
	carmen_mapper_build_obstacle_cost_map(&cost_map, &map, &distance_map, obstacle_cost_distance);
	carmen_prob_models_create_compact_map(&compacted_cost_map, &cost_map, 0.0);

	if (compacted_cost_map.number_of_known_points_on_the_map > 0)
	{
		carmen_map_server_publish_compact_cost_map_message(&compacted_cost_map,	msg->timestamp);
		carmen_prob_models_clear_carmen_map_using_compact_map(&cost_map, &compacted_cost_map, 0.0);
		carmen_prob_models_free_compact_map(&compacted_cost_map);
	}

//	static double last_timestamp = 0.0;
//	double timestamp = carmen_get_time();
//	printf("delta_t %lf\n", timestamp - last_timestamp);
//	last_timestamp = timestamp;
}


void 
shutdown_module(int signo)
{
  if (signo == SIGINT)
  {
    carmen_ipc_disconnect();
    printf("obstacle_distance_mapper: disconnected.\n");

    exit(0);
  }
}


static int
read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
			{(char *)"rrt",	(char *)"obstacle_cost_distance",	CARMEN_PARAM_DOUBLE,	&obstacle_cost_distance,	1, NULL},
			{(char *)"rrt",	(char *)"obstacle_probability_threshold",	CARMEN_PARAM_DOUBLE,	&obstacle_probability_threshold,	1, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}

  
int 
main(int argc, char **argv) 
{
  /* Connect to IPC Server */
  carmen_ipc_initialize(argc, argv);

  /* Check the param server version */
  carmen_param_check_version(argv[0]);

  /* Register shutdown cleaner handler */
  signal(SIGINT, shutdown_module);

  /* Read parameters */
  read_parameters(argc, argv);

  /* Subscribe to mapper messages */
  carmen_mapper_subscribe_message(NULL,
  			       (carmen_handler_t) carmen_mapper_map_handler,
  			       CARMEN_SUBSCRIBE_LATEST);

  /* Loop forever waiting for messages */
  carmen_ipc_dispatch();

  return (0);
}
