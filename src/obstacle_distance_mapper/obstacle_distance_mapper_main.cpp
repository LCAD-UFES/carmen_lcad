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

carmen_map_t 						 map;
carmen_map_t 						 cost_map;
carmen_prob_models_distance_map 	 distance_map;

carmen_point_t g_goal_position;
carmen_point_t g_robot_position;
carmen_point_t goal_list_message;

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

//	if ((g_goal_position.x != 0.0) && (g_goal_position.y != 0.0))
//		carmen_prob_models_create_masked_distance_map(&distance_map, &map, obstacle_probability_threshold, &g_robot_position, &g_goal_position);
//	else
		carmen_prob_models_create_distance_map(&distance_map, &map, obstacle_probability_threshold);

	carmen_obstacle_distance_mapper_publish_distance_map_message(&distance_map, timestamp);
}

///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	//printf("tempo da localizacao: %lf\n", msg->timestamp);

	g_robot_position = msg->globalpos;
}


static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{

	g_robot_position = msg->truepose;
}

static void
navigator_ackerman_set_goal_message_handler(carmen_navigator_ackerman_set_goal_message *msg)
{
	//na mensagem atual não é possível representar um goal nulo
	if (msg->x == -1 && msg->y == -1 && msg->theta == 0)
	{
		return;
	}
	g_goal_position.x = msg->x;
	g_goal_position.y = msg->y;
	g_goal_position.theta = msg->theta;
}

static void
behaviour_selector_goal_list_message_handler(carmen_behavior_selector_goal_list_message *msg)
{

	if ((msg->size <= 0) || !msg->goal_list)
	{
		printf("Empty goal list\n");
		return;
	}

	g_goal_position.x = msg->goal_list->x;
	g_goal_position.y = msg->goal_list->y;
	g_goal_position.theta = carmen_normalize_theta(msg->goal_list->theta);

}


void
lane_message_handler(carmen_behavior_selector_road_profile_message *message)
{
//	printf("RDDF NUM POSES: %d \n", message->number_of_poses);
	int size = message->number_of_poses;
	if (0 < size)
	{
		goal_list_message.x = message->poses[size-1].x;
		goal_list_message.y = message->poses[size-1].y;
		goal_list_message.theta = message->poses[size-1].theta;

//		printf("RDDF %d: x  = %lf, y = %lf , theta = %lf\n", i, message->poses[i].x, message->poses[i].y, message->poses[i].theta);
//		getchar();
	}
}


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
//	printf("delta_t %lf\n", (1/(timestamp - last_timestamp)));
//	last_timestamp = timestamp;
}

////////////////////////////////////////////////////////////////////////////////////////////////
//
//Subscribers
//
///////////////////////////////////////////////////////////////////////////////////////////////

void
register_handlers()
{

	carmen_simulator_ackerman_subscribe_truepos_message(NULL,
			(carmen_handler_t) simulator_ackerman_truepos_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_localize_ackerman_subscribe_globalpos_message(NULL,
			(carmen_handler_t) localize_ackerman_globalpos_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_mapper_subscribe_message(NULL,
			(carmen_handler_t) carmen_mapper_map_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_behavior_selector_subscribe_goal_list_message(NULL,
			(carmen_handler_t) behaviour_selector_goal_list_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *)CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_NAME,
			(char *)CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_FMT,
			NULL, sizeof(carmen_navigator_ackerman_set_goal_message),
			(carmen_handler_t)navigator_ackerman_set_goal_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	 carmen_subscribe_message(
			 (char *)CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME,
			 (char *)CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_FMT,
	    	 NULL, sizeof (carmen_behavior_selector_road_profile_message),
			 (carmen_handler_t) lane_message_handler, CARMEN_SUBSCRIBE_LATEST);

}

////////////////////////////////////////////////////////////////////////////////////////////////

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

  /* Subscribe desired messages */
  register_handlers();

  /* Read parameters */
  read_parameters(argc, argv);

  /* Loop forever waiting for messages */
  carmen_ipc_dispatch();

  return (0);
}
