#include "stehs_planner.hpp"
#include "stehs_planner_messages.h"
#include <carmen/robot_ackerman_interface.h>

StehsPlanner stehs_planner;

double localizer_pose_timestamp;


static void
define_path_message()
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(RRT_PATH_NAME, IPC_VARIABLE_LENGTH,
			RRT_PATH_FMT);
	carmen_test_ipc_exit(err, "Could not define", RRT_PATH_NAME);
}


void
publish_rrt_path_message(rrt_path_message *msg)
{
	static int firsttime = 1;
	IPC_RETURN_TYPE err;

	if (firsttime)
	{
		define_path_message();
		firsttime = 0;
	}

	err = IPC_publishData(RRT_PATH_NAME, msg);
	carmen_test_ipc(err, "Could not publish",
			RRT_PATH_NAME);
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
stehs_planner_publish_plan_tree_message()
{
	int i = 0;
	rrt_path_message msg;
	std::list<carmen_ackerman_path_point_t>::iterator it,next, end = stehs_planner.state_list.end();

	msg.host  = carmen_get_host();
	msg.timestamp = localizer_pose_timestamp;
	msg.last_goal = 0; //GlobalState::last_goal ? 1 : 0;

	carmen_ackerman_path_point_t &goal = stehs_planner.state_list.back(); // coloca o ultimo estado como goal
	msg.goal.x = goal.x;
	msg.goal.y = goal.y;
	msg.goal.theta = goal.theta;

	msg.size = stehs_planner.state_list.size();
	msg.path = (Edge_Struct *) malloc(sizeof(Edge_Struct) * msg.size);

	it = stehs_planner.state_list.begin();
	next = it;
	next++;

	for (; next != end; it++, i++, next++)
	{
		msg.path[i].p1.x     = it->x;
		msg.path[i].p1.y     = it->y;
		msg.path[i].p1.theta = it->theta;
		msg.path[i].p1.v     = it->v;
		msg.path[i].p1.phi   = it->phi;

		msg.path[i].p2.x     = next->x;
		msg.path[i].p2.y     = next->y;
		msg.path[i].p2.theta = next->theta;
		msg.path[i].p2.v     = next->v;
		msg.path[i].p2.phi   = next->phi;

		msg.path[i].v    = next->v;
		msg.path[i].phi  = next->phi;
		msg.path[i].time = next->time;

//		printf( "p1.x = %lf, p1.y = %lf, p1.theta = %lf, p1.v = %lf, p1.phi = %lf\n"
//				"p2.x = %lf, p2.y = %lf, p2.theta = %lf, p2.v = %lf, p2.phi = %lf\n"
//				"command.v = %lf, command.phi = %lf, command.time = %lf\n",
//				msg.path[i].p1.x, msg.path[i].p1.y, msg.path[i].p1.theta, msg.path[i].p1.v, msg.path[i].p1.phi,
//				msg.path[i].p2.x, msg.path[i].p2.y, msg.path[i].p2.theta, msg.path[i].p2.v, msg.path[i].p2.phi,
//				msg.path[i].v,  msg.path[i].phi,  msg.path[i].time);

	}

	publish_rrt_path_message(&msg);
	free(msg.path);
}


void
publish_model_predictive_planner_motion_commands()
{
	carmen_ackerman_motion_command_t* commands =
			(carmen_ackerman_motion_command_t*) (malloc(stehs_planner.state_list.size() * sizeof(carmen_ackerman_motion_command_t)));
	int i = 0;
	for (std::list<carmen_ackerman_path_point_t>::iterator it = stehs_planner.state_list.begin();	it != stehs_planner.state_list.end(); ++it)
	{
		commands[i].v = it->v;
		commands[i].phi = it->phi;
		commands[i].time = it->time;
		commands[i].x = it->x;
		commands[i].y = it->y;
		commands[i].theta = it->theta;

		i++;
	}

	int num_commands = stehs_planner.state_list.size();

	if (stehs_planner.use_obstacle_avoider)
		carmen_robot_ackerman_publish_motion_command(commands, num_commands, localizer_pose_timestamp);
	else
		carmen_base_ackerman_publish_motion_command(commands, num_commands, localizer_pose_timestamp);

	free(commands);
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
	stehs_planner.start.x = msg->globalpos.x;
	stehs_planner.start.y = msg->globalpos.y;
	stehs_planner.start.theta = msg->globalpos.theta;
	stehs_planner.start.v = msg->v;
	stehs_planner.start.phi = msg->phi;
	localizer_pose_timestamp = msg->timestamp;

//	printf("GLOBAL POS x: %f y: %f theta: %f v: %f phi: %f\n", stehs_planner.start.x, stehs_planner.start.y,
//			stehs_planner.start.theta, stehs_planner.start.v, stehs_planner.start.phi);

	if (stehs_planner.lane_ready && stehs_planner.distance_map_ready && stehs_planner.goal_ready)
	{
		stehs_planner.lane_ready = stehs_planner.distance_map_ready = stehs_planner.goal_ready = false;
		double time = carmen_get_time();
		//stehs_planner.RDDFSpaceExploration();
		stehs_planner.GeneratePath();
		printf("Tempo %f Ncirc %ld Nstate %ld\n", carmen_get_time() - time, stehs_planner.circle_path.size(),
				stehs_planner.state_list.size());
		if (!stehs_planner.state_list.empty())
		{
			if (stehs_planner.use_mpc)
			{
				publish_model_predictive_planner_motion_commands();
			}
			else
			{
				stehs_planner_publish_plan_tree_message();
			}
		}
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

	cv::destroyAllWindows();

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
		{(char *) "rrt",	(char *) "use_obstacle_avoider", 						CARMEN_PARAM_ONOFF,	 &stehs_planner.use_obstacle_avoider, 												1, NULL},
		{(char *) "rrt",	(char *) "use_mpc",										CARMEN_PARAM_ONOFF,	 &stehs_planner.use_mpc, 															0, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}


void
define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	define_messages();

	read_parameters(argc, argv);

	register_handlers();

	carmen_ipc_dispatch();
}
