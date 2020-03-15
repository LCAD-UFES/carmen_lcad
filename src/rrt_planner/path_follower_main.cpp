/*
 * rrt_path_follower.cpp
 *
 *  Created on: 04/12/2012
 *      Author: romulo
 */

#include <carmen/behavior_selector_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/carmen.h>

#include <carmen/rddf_messages.h>
#include <carmen/rddf_interface.h>

#include "model/robot_config.h"
#include "model/global_state.h"

#include "util/util.h"
#include "util/ackerman.h"
#include "util/publisher_util.h"
#include "util/obstacle_detection.h"

#include "path_follower/follower.h"
#include "path_follower/path_follower_ackerman.h"


int g_teacher_mode = 0;
Path_Follower_Ackerman follower;

double t0 = 0.0;


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
Path_Follower_Ackerman::publish_path_follower_motion_commands(carmen_ackerman_motion_command_t *commands, int num_commands, double timestamp)
{
//	system("clear");
//	for (int i = 0; (i < num_commands) && (i < 20); i++)
//		printf("v = %2.2lf, phi = %2.2lf, t = %2.3lf\n", commands[i].v, carmen_radians_to_degrees(commands[i].phi), commands[i].time);
//	fflush(stdout);

	if (use_obstacle_avoider)
	{
		if (!g_teacher_mode)  // standard operation
			carmen_robot_ackerman_publish_motion_command(commands, num_commands, timestamp);
		else
			carmen_robot_ackerman_publish_teacher_motion_command(commands, num_commands, timestamp);
	}
	else
		carmen_base_ackerman_publish_motion_command(commands, num_commands, timestamp);
}


void
Path_Follower_Ackerman::publish_path_follower_single_motion_command(double v, double phi, double timestamp)
{
	carmen_ackerman_motion_command_t commands[2];

	commands[0].v = v;
	commands[0].phi = phi;
	commands[0].time = 0.5;
	commands[1] = commands[0];
	publish_path_follower_motion_commands(commands, 2, timestamp);
}


void
publish_navigator_ackerman_plan_message(list<RRT_Path_Edge> &path)
{
	carmen_navigator_ackerman_plan_message msg;
	list<RRT_Path_Edge>::iterator it;
	list<RRT_Path_Edge>::reverse_iterator rit;
	int i;

	msg.host = carmen_get_host();
	msg.timestamp = carmen_get_time();

	if (path.empty())
		return;

	msg.path_length = path.size() + 1;
	msg.path = (carmen_ackerman_traj_point_t *) malloc(sizeof(carmen_ackerman_traj_point_t) * (msg.path_length));

	it = path.begin();

	for (i = 0; it != path.end(); it++)
	{
		msg.path[i].x	  = it->p1.pose.x;
		msg.path[i].y	  = it->p1.pose.y;
		msg.path[i].theta = it->p1.pose.theta;
		msg.path[i].v	  = it->command.v;
		msg.path[i].phi	  = it->command.phi;
		i++;
	}

	rit = path.rbegin();

	msg.path[i].x	  = rit->p2.pose.x;
	msg.path[i].y	  = rit->p2.pose.y;
	msg.path[i].theta = rit->p2.pose.theta;
	msg.path[i].v	  = rit->command.v;
	msg.path[i].phi	  = rit->command.phi;


	Publisher_Util::publish_navigator_ackerman_plan_message(msg);

	free(msg.path);
}


void
publish_navigator_ackerman_status_message()
{
	if (!GlobalState::localize_pose)
	{
		return;
	}

	IPC_RETURN_TYPE err = IPC_OK;

	static bool first_time = true;

	if (first_time)
	{
		err = IPC_defineMsg(
				(char *)CARMEN_NAVIGATOR_ACKERMAN_STATUS_NAME,
				IPC_VARIABLE_LENGTH,
				(char *)CARMEN_NAVIGATOR_ACKERMAN_STATUS_FMT);
		carmen_test_ipc_exit(err, "Could not define", CARMEN_NAVIGATOR_ACKERMAN_STATUS_NAME);
	}

	carmen_navigator_ackerman_status_message msg;
	msg.autonomous = GlobalState::following_path;
	msg.goal_set   = GlobalState::goal_pose != NULL;

	if (msg.goal_set)
	{
		msg.goal.x = GlobalState::goal_pose->x;
		msg.goal.y = GlobalState::goal_pose->y;
		msg.goal.theta = GlobalState::goal_pose->theta;
	}
	else
	{
		msg.goal.theta = 0;
		msg.goal.y	   = 0;
		msg.goal.x	   = 0;
	}

	msg.host		= carmen_get_host();
	msg.robot.x		= GlobalState::localize_pose->x;
	msg.robot.y		= GlobalState::localize_pose->y;
	msg.robot.theta = GlobalState::localize_pose->theta;
	msg.robot.v		= GlobalState::last_odometry.v;
	msg.robot.phi	= GlobalState::last_odometry.phi;
	msg.timestamp	= carmen_get_time();

	err = IPC_publishData(CARMEN_NAVIGATOR_ACKERMAN_STATUS_NAME, &msg);

	carmen_test_ipc(err, "Could not publish", CARMEN_NAVIGATOR_ACKERMAN_STATUS_NAME);
}
///////////////////////////////////////////////////////////////////////////////////////////////


namespace RRT_IPC
{

static void
build_and_follow_path(carmen_point_t point, double pose_timestamp)
{
	printf("\n *** entrou em build_and_follow_path() - %lf  %lf\n", carmen_get_time(), carmen_get_time() - t0);
	t0 = carmen_get_time();
	Robot_State initial_robot_state;
	Pose pose = Util::convert_to_pose(point);

	GlobalState::set_robot_pose(pose, pose_timestamp);

	initial_robot_state = GlobalState::estimate_initial_robot_state();

	*GlobalState::localize_pose = initial_robot_state.pose;

	follower.update_path();
	printf("follower.update_path() - %lf\n", carmen_get_time() - t0);
	fflush(stdout);


	follower.build_and_send_refined_path();
	printf("follower.build_and_send_refined_path() - %lf\n", carmen_get_time() - t0);
	fflush(stdout);

//	printf("n %d\n", (int) follower.get_path().size());
	if (GlobalState::current_algorithm == CARMEN_BEHAVIOR_SELECTOR_RRT)
	{
		publish_navigator_ackerman_plan_message(follower.get_path());
//		publish_navigator_ackerman_status_message();
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	if (g_teacher_mode)
		Follower::go();

	build_and_follow_path(msg->globalpos, msg->timestamp);
}


static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
	GlobalState::last_odometry.v = msg->v;
	GlobalState::last_odometry.phi = msg->phi;

	build_and_follow_path(msg->truepose, msg->timestamp);
}


void
rrt_path_message_handler(rrt_path_message *msg)
{
	list<RRT_Path_Edge> path;
	RRT_Path_Edge edge;

//	double tm_to;
//	tm_to = carmen_get_time();
//
//	printf("diff: %lf\n", tm_to - msg->tm_from);

	if (msg->size > 0)
	{
		GlobalState::last_rrt_path_message_timestamp = msg->timestamp;

		free(GlobalState::goal_pose);

		GlobalState::goal_pose = NULL;

		GlobalState::goal_pose = new Pose();
		GlobalState::goal_pose->x = msg->goal.x;
		GlobalState::goal_pose->y = msg->goal.y;
		GlobalState::goal_pose->theta = msg->goal.theta;

		GlobalState::last_path_received_is_empty = false;
	}
	else
		GlobalState::last_path_received_is_empty = true;

	for (int i = 0; i < msg->size; i++)
	{
		edge.p1.pose.x = msg->path[i].p1.x;
		edge.p1.pose.y = msg->path[i].p1.y;
		edge.p1.pose.theta = msg->path[i].p1.theta;
		edge.p1.v_and_phi.v = msg->path[i].p1.v;
		edge.p1.v_and_phi.phi = msg->path[i].p1.phi;

		edge.p2.pose.x = msg->path[i].p2.x;
		edge.p2.pose.y = msg->path[i].p2.y;
		edge.p2.pose.theta = msg->path[i].p2.theta;
		edge.p2.v_and_phi.v = msg->path[i].p2.v;
		edge.p2.v_and_phi.phi = msg->path[i].p2.phi;

		edge.command.v = msg->path[i].v;
		edge.command.phi = msg->path[i].phi;
		edge.time = msg->path[i].time;

		path.push_back(edge);
	}

	GlobalState::last_goal = msg->last_goal == 1;

	follower.set_path(path);
}


static void
base_ackerman_odometry_message_handler(carmen_base_ackerman_odometry_message *msg)
{
	GlobalState::last_odometry.v = msg->v;
	GlobalState::last_odometry.phi = msg->phi;
}


static void
behaviour_selector_goal_list_message_handler(carmen_behavior_selector_goal_list_message *msg)
{
	Pose goal_pose;

	if ((msg->size <= 0) || !msg->goal_list || !GlobalState::localize_pose)
		return;

	GlobalState::last_goal = (msg->size == 1)? true: false;

	goal_pose.x = msg->goal_list->x;
	goal_pose.y = msg->goal_list->y;
	goal_pose.theta = carmen_normalize_theta(msg->goal_list->theta);

	GlobalState::robot_config.max_vel = fmin(msg->goal_list->v, GlobalState::param_max_vel);
}


static void
behavior_selector_state_message_handler(carmen_behavior_selector_state_message *msg)
{
	GlobalState::behavior_selector_state = msg->state;
	GlobalState::current_algorithm = msg->algorithm;
}


static void
navigator_ackerman_go_message_handler()
{
	Follower::go();
}


static void
navigator_ackerman_stop_message_handler()
{
	Follower::stop();
}


static void
signal_handler(int sig)
{
	printf("Signal %d received, exiting program ...\n", sig);
	exit(1);
}
///////////////////////////////////////////////////////////////////////////////////////////////


void
register_handlers_specific()
{
	carmen_subscribe_message(
			(char *)CARMEN_NAVIGATOR_ACKERMAN_GO_NAME,
			(char *)CARMEN_DEFAULT_MESSAGE_FMT,
			NULL, sizeof(carmen_navigator_ackerman_go_message),
			(carmen_handler_t)navigator_ackerman_go_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *)CARMEN_NAVIGATOR_ACKERMAN_STOP_NAME,
			(char *)CARMEN_DEFAULT_MESSAGE_FMT,
			NULL, sizeof(carmen_navigator_ackerman_stop_message),
			(carmen_handler_t)navigator_ackerman_stop_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *)RRT_PATH_NAME,
			(char *)RRT_PATH_FMT,
			NULL, sizeof(rrt_path_message),
			(carmen_handler_t)rrt_path_message_handler,
			CARMEN_SUBSCRIBE_LATEST);
}


void
register_handlers()
{
	signal(SIGINT, signal_handler);

	if (!GlobalState::use_truepos)
	{
		carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
		carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) base_ackerman_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);
	}
	else
		carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) simulator_ackerman_truepos_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_behavior_selector_subscribe_current_state_message(NULL, (carmen_handler_t) behavior_selector_state_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_behavior_selector_subscribe_goal_list_message(NULL, (carmen_handler_t) behaviour_selector_goal_list_message_handler, CARMEN_SUBSCRIBE_LATEST);

	register_handlers_specific();

}
}//end namespace RRT_IPC


void
read_parameters_specific(int argc, char **argv)
{
	carmen_param_t optional_param_list[] = {
		{(char *) "rrt", (char *) "use_obstacle_avoider", 	CARMEN_PARAM_ONOFF,	&Path_Follower_Ackerman::use_obstacle_avoider, 1, NULL},
		{(char *) "rrt", (char *) "log_mode", 				CARMEN_PARAM_ONOFF,	&GlobalState::log_mode, 1, NULL},
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, optional_param_list, sizeof(optional_param_list) / sizeof(optional_param_list[0]));
}


void
read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] = {
		{(char *) "robot",	(char *) "length",								  		CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.length,								 			1, NULL},
		{(char *) "robot",	(char *) "width",								  		CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.width,								 			1, NULL},
		{(char *) "robot", 	(char *) "distance_between_rear_wheels",		  			CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.distance_between_rear_wheels,			 		1, NULL},
		{(char *) "robot", 	(char *) "distance_between_front_and_rear_axles", 		CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.distance_between_front_and_rear_axles, 			1, NULL},
		{(char *) "robot", 	(char *) "distance_between_front_car_and_front_wheels",	CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.distance_between_front_car_and_front_wheels,	1, NULL},
		{(char *) "robot", 	(char *) "distance_between_rear_car_and_rear_wheels",	CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.distance_between_rear_car_and_rear_wheels,		1, NULL},
		{(char *) "robot", 	(char *) "max_velocity",						  			CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.max_vel,								 		1, NULL},
		{(char *) "robot", 	(char *) "max_steering_angle",					  		CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.max_phi,								 		1, NULL},
		{(char *) "robot", 	(char *) "maximum_acceleration_forward",					CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_acceleration_forward,					1, NULL},
		{(char *) "robot", 	(char *) "maximum_acceleration_reverse",					CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_acceleration_reverse,					1, NULL},
		{(char *) "robot", 	(char *) "maximum_deceleration_forward",					CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_deceleration_forward,					1, NULL},
		{(char *) "robot", 	(char *) "maximum_deceleration_reverse",					CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_deceleration_reverse,					1, NULL},
		{(char *) "robot", 	(char *) "desired_decelaration_forward",					CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.desired_decelaration_forward,					1, NULL},
		{(char *) "robot", 	(char *) "desired_decelaration_reverse",					CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.desired_decelaration_reverse,					1, NULL},
		{(char *) "robot", 	(char *) "desired_acceleration",							CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.desired_acceleration,							1, NULL},
		{(char *) "robot", 	(char *) "desired_steering_command_rate",				CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.desired_steering_command_rate,					1, NULL},
		{(char *) "robot", 	(char *) "understeer_coeficient",						CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.understeer_coeficient,							1, NULL},
		{(char *) "behavior_selector", (char *) "use_truepos", 						CARMEN_PARAM_ONOFF, &GlobalState::use_truepos, 0, NULL}
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	GlobalState::param_max_vel = GlobalState::robot_config.max_vel;

	//initialize default parameters values
	GlobalState::timeout = 0.8;
	GlobalState::distance_interval = 1.5;
	GlobalState::plan_time = 0.08;

	carmen_param_t optional_param_list[] = {
			{(char *) "rrt",	(char *) "show_debug_info",		CARMEN_PARAM_ONOFF,		&GlobalState::show_debug_info,		1, NULL},
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, optional_param_list, sizeof(optional_param_list) / sizeof(optional_param_list[0]));

	read_parameters_specific(argc, argv);

	carmen_param_t param_optional_list[] =
	{
		{(char *) "commandline", (char *) "teacher_mode", CARMEN_PARAM_ONOFF, &g_teacher_mode, 0, NULL}
	};

	carmen_param_install_params(argc, argv, param_optional_list, sizeof(param_optional_list) / sizeof(param_optional_list[0]));
}


int
main(int argc, char **argv)
{
	Follower::path_follower = &follower;

	follower.set_path_lost_condition(0.8, carmen_degrees_to_radians(10));

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);

	RRT_IPC::register_handlers();

	carmen_ipc_dispatch();
}
