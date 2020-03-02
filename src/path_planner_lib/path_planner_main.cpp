#include "path_planner.h"


using namespace std;


static int use_truepos = 0;
carmen_robot_ackerman_config_t robot_config;
carmen_localize_ackerman_globalpos_message *current_globalpos_msg = NULL;
carmen_simulator_ackerman_truepos_message *current_truepos_msg = NULL;
carmen_obstacle_distance_mapper_map_message distance_map;
carmen_point_t *final_goal = NULL;


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
pos_message_handler(carmen_point_t robot_pose)
{
	if (/*final_goal != NULL && */distance_map.complete_x_offset != NULL)
	{
//		velocity = 2;
		carmen_point_t fgoal;
//		fgoal.x = 7757757.31;     // Reta LCAD
//		fgoal.y = -363538.69;
//		fgoal.theta = 0.628;

//		fgoal.x = 7757833.72;     // Depois da preimeira curva
//		fgoal.y = -363533.71;
//		fgoal.theta = -0.692;

		fgoal.x = 7757865.52;     // Estacionamento Ambiental
		fgoal.y = -363539.93;
		fgoal.theta = -0.741;
		compute_path(&robot_pose, /*final_goal */ &fgoal, robot_config, &distance_map);

		final_goal = NULL;
	}
}


static void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	current_globalpos_msg = msg;
	pos_message_handler(msg->globalpos/*, msg->v, msg->timestamp*/);
}


static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
	current_truepos_msg = msg;
	pos_message_handler(msg->truepose/*, msg->v, msg->timestamp*/);
}


static void
carmen_obstacle_distance_mapper_compact_map_message_handler(carmen_obstacle_distance_mapper_compact_map_message *message)
{
	static carmen_obstacle_distance_mapper_compact_map_message *compact_distance_map = NULL;  // TODO is it possible to remove this line and the if

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
}


void
carmen_rddf_play_end_point_message_handler(carmen_rddf_end_point_message *rddf_end_point_message)
{
	final_goal = &(rddf_end_point_message->point);

//	printf ("Recebeu Goal!!!!!  %d\n", rddf_end_point_message->number_of_poses);
}


static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		exit(0);
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_rddf_play_subscribe_messages()
{
	if (!use_truepos)
		carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) simulator_ackerman_truepos_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_obstacle_distance_mapper_subscribe_compact_map_message(NULL, (carmen_handler_t) carmen_obstacle_distance_mapper_compact_map_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_rddf_subscribe_end_point_message(NULL, (carmen_handler_t) carmen_rddf_play_end_point_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


static void
carmen_rddf_play_get_parameters(int argc, char** argv)
{
	carmen_param_t param_list[] =
	{
			{(char *) "robot",	(char *) "length",								  		CARMEN_PARAM_DOUBLE, &robot_config.length,							 			1, NULL},
			{(char *) "robot",	(char *) "width",								  		CARMEN_PARAM_DOUBLE, &robot_config.width,								 			1, NULL},
			{(char *) "robot", 	(char *) "distance_between_rear_wheels",		  		CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_wheels,			 		1, NULL},
			{(char *) "robot", 	(char *) "distance_between_front_and_rear_axles", 		CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_and_rear_axles, 		1, NULL},
			{(char *) "robot", 	(char *) "distance_between_front_car_and_front_wheels",	CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_car_and_front_wheels,	1, NULL},
			{(char *) "robot", 	(char *) "distance_between_rear_car_and_rear_wheels",	CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_car_and_rear_wheels,		1, NULL},
			{(char *) "robot", 	(char *) "max_velocity",						  		CARMEN_PARAM_DOUBLE, &robot_config.max_v,									 		1, NULL},
			{(char *) "robot", 	(char *) "max_steering_angle",					  		CARMEN_PARAM_DOUBLE, &robot_config.max_phi,								 		1, NULL},
			{(char *) "robot", 	(char *) "maximum_acceleration_forward",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_forward,					1, NULL},
			{(char *) "robot", 	(char *) "maximum_acceleration_reverse",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_reverse,					1, NULL},
			{(char *) "robot", 	(char *) "maximum_deceleration_forward",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_deceleration_forward,					1, NULL},
			{(char *) "robot", 	(char *) "maximum_deceleration_reverse",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_deceleration_reverse,					1, NULL},
			{(char *) "robot", 	(char *) "maximum_steering_command_rate",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_steering_command_rate,					1, NULL},
			{(char *) "robot", 	(char *) "understeer_coeficient",						CARMEN_PARAM_DOUBLE, &robot_config.understeer_coeficient,							1, NULL}
		};

	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	carmen_rddf_play_get_parameters(argc, argv);

	carmen_rddf_play_subscribe_messages();

	signal (SIGINT, shutdown_module);

	carmen_ipc_dispatch();

	return (0);
}
