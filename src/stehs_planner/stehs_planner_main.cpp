#include "stehs_planner.hpp"


StehsPlanner stehs_planner;


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	stehs_planner.start.x = msg->globalpos.x;
	stehs_planner.start.y = msg->globalpos.y;
	stehs_planner.start.theta = msg->globalpos.theta;
	stehs_planner.start.v = msg->v;
	stehs_planner.start.phi = msg->phi;

//	printf("GLOBAL POS x: %f y: %f theta: %f v: %f phi: %f\n", stehs_planner.start.x, stehs_planner.start.y,
//			stehs_planner.start.theta, stehs_planner.start.v, stehs_planner.start.phi);

	if(stehs_planner.lane_ready && stehs_planner.distance_map_ready && stehs_planner.goal_ready)
	{
		double time = carmen_get_time();
		stehs_planner.RDDFSpaceExploration();
		printf("%f tam %ld\n", carmen_get_time() - time, stehs_planner.circle_path.size());
	}
	// chamar funcao principal aqui
}

/*

static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
	Pose pose = Util::convert_to_pose(msg->truepose);
	GlobalState::set_robot_pose(pose, msg->timestamp);

	build_and_follow_path();
}
*/

/*

static void
navigator_ackerman_set_goal_message_handler(carmen_navigator_ackerman_set_goal_message *msg)
{
	// Na mensagem atual não é possível representar um goal nulo. Coordenadas do mundo são grandes.
	if (msg->x == -1 && msg->y == -1 && msg->theta == 0)
	{
		GlobalState::goal_pose = NULL;
		return;
	}

	Pose goal_pose;
	goal_pose.x		= msg->x;
	goal_pose.y		= msg->y;
	goal_pose.theta = carmen_normalize_theta(msg->theta);
	GlobalState::set_goal_pose(goal_pose);

	GlobalState::last_goal = true;
}
*/


/*static void
base_ackerman_odometry_message_handler(carmen_base_ackerman_odometry_message *msg)
{
	stehs_planner.start_pose.v = msg->v;
	stehs_planner.start_pose.phi = msg->phi;
}*/


static void
behaviour_selector_goal_list_message_handler(carmen_behavior_selector_goal_list_message *msg)
{
	if ((msg->size <= 0) || !msg->goal_list)  // Precisa? || !GlobalState::localizer_pose)
	{
		printf("Empty goal list or localize not received\n");
		return;
	}

	//GlobalState::last_goal = (msg->size == 1)? true: false;

	stehs_planner.goal = msg->goal_list[0];

	stehs_planner.goal.theta = carmen_normalize_theta(msg->goal_list->theta);

	stehs_planner.goal.v = fmin(msg->goal_list->v, stehs_planner.robot_config.max_v);

	stehs_planner.goal_ready = true;

	//GlobalState::set_goal_pose(goal_pose);   // Criar uma flag para verificar se chegou goal ou não?
}


static void
carmen_obstacle_distance_mapper_message_handler(carmen_obstacle_distance_mapper_message *message)
{
	stehs_planner.distance_map = message;

	stehs_planner.distance_map_ready = true;
}


static void
navigator_ackerman_go_message_handler()
{
	stehs_planner.active = true;
}


static void
navigator_ackerman_stop_message_handler()
{
	stehs_planner.active = false;

	// Fazer funcao que para o robo
}


static void
signal_handler(int sig)
{
	printf("Signal %d received, exiting program ...\n", sig);

	exit(1);
}


void
lane_message_handler(carmen_behavior_selector_road_profile_message *message)
{
	stehs_planner.goal_list_message = message;
	stehs_planner.lane_ready = true;

//	printf("RDDF NUM POSES: %d \n", stehs_planner.goal_list_message->number_of_poses);
//
//	for (int i = 0; i < stehs_planner.goal_list_message->number_of_poses; i++)
//	{
//		printf("RDDF %d: x  = %lf, y = %lf , theta = %lf\n", i, stehs_planner.goal_list_message->poses[i].x, stehs_planner.goal_list_message->poses[i].y, stehs_planner.goal_list_message->poses[i].theta);
//		getchar();
//	}
}


///////////////////////////////////////////////////////////////////////////////////////////////


void
register_handlers()
{
	signal(SIGINT, signal_handler);

//	if (!stehs_planner.cheat)
		carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
//	else
//		carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) simulator_ackerman_truepos_message_handler, CARMEN_SUBSCRIBE_LATEST);

//	carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) base_ackerman_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);

//	carmen_behavior_selector_subscribe_current_state_message(NULL, (carmen_handler_t) behavior_selector_state_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_behavior_selector_subscribe_goal_list_message(NULL, (carmen_handler_t) behaviour_selector_goal_list_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_obstacle_distance_mapper_subscribe_message(NULL,	(carmen_handler_t) carmen_obstacle_distance_mapper_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME, (char *) CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_FMT,
    		NULL, sizeof (carmen_behavior_selector_road_profile_message), (carmen_handler_t) lane_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
		(char *) CARMEN_NAVIGATOR_ACKERMAN_GO_NAME,
		(char *) CARMEN_DEFAULT_MESSAGE_FMT,
		NULL, sizeof(carmen_navigator_ackerman_go_message),
		(carmen_handler_t)navigator_ackerman_go_message_handler,
		CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
		(char *) CARMEN_NAVIGATOR_ACKERMAN_STOP_NAME,
		(char *) CARMEN_DEFAULT_MESSAGE_FMT,
		NULL, sizeof(carmen_navigator_ackerman_stop_message),
		(carmen_handler_t)navigator_ackerman_stop_message_handler,
		CARMEN_SUBSCRIBE_LATEST);

//	carmen_subscribe_message(
//		(char *) CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_NAME,
//		(char *) CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_FMT,
//		NULL, sizeof(carmen_navigator_ackerman_set_goal_message),
//		(carmen_handler_t)navigator_ackerman_set_goal_message_handler,
//		CARMEN_SUBSCRIBE_LATEST);

}


void
read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] = {
		{(char *) "robot",	(char *) "length",								  		CARMEN_PARAM_DOUBLE, &stehs_planner.robot_config.length,								 			1, NULL},
		{(char *) "robot",	(char *) "width",								  		CARMEN_PARAM_DOUBLE, &stehs_planner.robot_config.width,								 			1, NULL},
		{(char *) "robot", 	(char *) "distance_between_rear_wheels",		  		CARMEN_PARAM_DOUBLE, &stehs_planner.robot_config.distance_between_rear_wheels,			 		1, NULL},
		{(char *) "robot", 	(char *) "distance_between_front_and_rear_axles", 		CARMEN_PARAM_DOUBLE, &stehs_planner.robot_config.distance_between_front_and_rear_axles, 			1, NULL},
		{(char *) "robot", 	(char *) "distance_between_front_car_and_front_wheels",	CARMEN_PARAM_DOUBLE, &stehs_planner.robot_config.distance_between_front_car_and_front_wheels,	1, NULL},
		{(char *) "robot", 	(char *) "distance_between_rear_car_and_rear_wheels",	CARMEN_PARAM_DOUBLE, &stehs_planner.robot_config.distance_between_rear_car_and_rear_wheels,		1, NULL},
		{(char *) "robot", 	(char *) "max_velocity",						  		CARMEN_PARAM_DOUBLE, &stehs_planner.robot_config.max_v,									 		1, NULL},
		{(char *) "robot", 	(char *) "max_steering_angle",					  		CARMEN_PARAM_DOUBLE, &stehs_planner.robot_config.max_phi,								 		1, NULL},
		{(char *) "robot", 	(char *) "maximum_acceleration_forward",				CARMEN_PARAM_DOUBLE, &stehs_planner.robot_config.maximum_acceleration_forward,					1, NULL},
		{(char *) "robot", 	(char *) "maximum_acceleration_reverse",				CARMEN_PARAM_DOUBLE, &stehs_planner.robot_config.maximum_acceleration_reverse,					1, NULL},
		{(char *) "robot", 	(char *) "maximum_deceleration_forward",				CARMEN_PARAM_DOUBLE, &stehs_planner.robot_config.maximum_deceleration_forward,					1, NULL},
		{(char *) "robot", 	(char *) "maximum_deceleration_reverse",				CARMEN_PARAM_DOUBLE, &stehs_planner.robot_config.maximum_deceleration_reverse,					1, NULL},
		{(char *) "robot", 	(char *) "maximum_steering_command_rate",				CARMEN_PARAM_DOUBLE, &stehs_planner.robot_config.maximum_steering_command_rate,					1, NULL},
		{(char *) "robot", 	(char *) "understeer_coeficient",						CARMEN_PARAM_DOUBLE, &stehs_planner.robot_config.understeer_coeficient,							1, NULL},

		{(char *) "rrt",	(char *) "show_debug_info",								CARMEN_PARAM_ONOFF,	 &stehs_planner.show_debug_info,												1, NULL},
		{(char *) "rrt",	(char *) "cheat",										CARMEN_PARAM_ONOFF,	 &stehs_planner.cheat,														1, NULL},
		//{(char *) "rrt",	(char *) "use_obstacle_avoider", 						CARMEN_PARAM_ONOFF,	 &stehs_planner.use_obstacle_avoider, 												1, NULL},
		//{(char *) "rrt",	(char *) "use_mpc",										CARMEN_PARAM_ONOFF,	 &stehs_planner.use_mpc, 															0, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);

	register_handlers();

	carmen_ipc_dispatch();
}
