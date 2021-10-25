/*
 * main.c
 *
 *  Created on: 16/12/2011
 *      Author: rradaelli
 */

#include <signal.h>
#include <string.h>
#include <stdio.h>

#include <carmen/carmen.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/map_server_interface.h>
#include <carmen/motion_planner_interface.h>
#include <carmen/navigator_gui_interface.h>
#include <carmen/rddf_interface.h>
#include <carmen/collision_detection.h>

#include <prob_measurement_model.h>
#include <prob_map.h>
#include <prob_interface.h>
#include <prob_measurement_model.h>
#include <prob_transforms.h>

#include "model/robot_config.h"
#include "model/global_state.h"
#include "message/rrt_planner_interface.h"
#include "util/util.h"
#include "util/ackerman.h"
#include "util/dijkstra.h"
#include "util/publisher_util.h"
#include "util/obstacle_detection.h"
#include "path_follower/follower.h"
#include "path_follower/path_follower_ackerman.h"
#include "util/lane.h"

#include "rrt_parking.h"
#include "rrt_lane.h"
#include "rrt.h"
#include "rs.h"


RRT_Parking *rrt_parking;
RRT_Lane *rrt_lane;
RRT *selected_rrt = 0;

void
print_path(list<RRT_Path_Edge> path, char *path_name);


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void
publish_rrt_path_message(list<RRT_Path_Edge> path)
{
	int i = 0;
	rrt_path_message msg;
	list<RRT_Path_Edge>::iterator it;

	msg.host  = carmen_get_host();
	msg.timestamp = GlobalState::rrt_planner_timestamp;
	msg.last_goal = GlobalState::last_goal ? 1 : 0;

	if (GlobalState::goal_pose)
	{
		msg.goal.x = GlobalState::goal_pose->x;
		msg.goal.y = GlobalState::goal_pose->y;
		msg.goal.theta = GlobalState::goal_pose->theta;
	}
	else
	{
		msg.goal.x = msg.goal.y = msg.goal.theta = 0.0;
	}

	if (path.empty())
	{
		// return;
		msg.size = 0;
		msg.path = NULL;
	}
	else
	{
		msg.size = path.size();
		msg.path = (Edge_Struct *) malloc(sizeof(Edge_Struct) * msg.size);
	}

	for (it = path.begin(); it != path.end(); it++, i++)
	{
		msg.path[i].p1.x     = it->p1.pose.x;
		msg.path[i].p1.y     = it->p1.pose.y;
		msg.path[i].p1.theta = it->p1.pose.theta;
		msg.path[i].p1.v     = it->p1.v_and_phi.v;
		msg.path[i].p1.phi   = it->p1.v_and_phi.phi;

		msg.path[i].p2.x     = it->p2.pose.x;
		msg.path[i].p2.y     = it->p2.pose.y;
		msg.path[i].p2.theta = it->p2.pose.theta;
		msg.path[i].p2.v     = it->p2.v_and_phi.v;
		msg.path[i].p2.phi   = it->p2.v_and_phi.phi;

		msg.path[i].v    = it->command.v;
		msg.path[i].phi  = it->command.phi;
		msg.path[i].time = it->time;

//		printf( "p1.x = %lf, p1.y = %lf, p1.theta = %lf, p1.v = %lf, p1.phi = %lf\n"
//				"p2.x = %lf, p2.y = %lf, p2.theta = %lf, p2.v = %lf, p2.phi = %lf\n"
//				"command.v = %lf, command.phi = %lf, command.time = %lf\n",
//				msg.path[i].p1.x, msg.path[i].p1.y, msg.path[i].p1.theta, msg.path[i].p1.v, msg.path[i].p1.phi,
//				msg.path[i].p2.x, msg.path[i].p2.y, msg.path[i].p2.theta, msg.path[i].p2.v, msg.path[i].p2.phi,
//				msg.path[i].v,  msg.path[i].phi,  msg.path[i].time);

		if (GlobalState::show_debug_info)
			printf("v = %2.2lf, phi = %2.2lf, t = %2.3lf, p1.v = %2.2lf, p1.phi = %2.2lf, p2.v = %2.2lf, p2.phi = %2.2lf\n",
					it->command.v, carmen_radians_to_degrees(it->command.phi), it->time,
					it->p1.v_and_phi.v, carmen_radians_to_degrees(it->p1.v_and_phi.phi),
					it->p2.v_and_phi.v, carmen_radians_to_degrees(it->p2.v_and_phi.phi));
	}

	if (GlobalState::show_debug_info)
	{
		printf("\n");
		fflush(stdout);
	}

	print_path(path, (char *) "====== PATH SEND ======");

	Publisher_Util::publish_rrt_path_message(&msg);
	free(msg.path);
}


void
RRT::publish_status_message()
{
	if (GlobalState::current_algorithm == CARMEN_BEHAVIOR_SELECTOR_RRT)
	{
		if (GlobalState::publish_tree)
		{
			Publisher_Util::publish_plan_tree_message(tree, reaches_goal_nodes);
			//Publisher_Util::publish_principal_path_message(selected_rrt->path);
		}
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////


namespace RRT_IPC
{

static void
set_rrt_current_goal(Pose goal_pose)
{
	if (selected_rrt->set_goal(goal_pose))
	{
		GlobalState::set_goal_pose(goal_pose);

		GlobalState::gradient_cost_map_old = true;
		GlobalState::goal_just_set = true;
//		selected_rrt->clear_the_search_tree_and_current_path();
	}
}



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{	// Este handler implementa a funcionalidade principal do modulo
	Robot_State initial_robot_state;

	// estimate the initial robot state based on the localize pose and last odometry received
	initial_robot_state = GlobalState::predict_initial_robot_state(msg->globalpos, msg->timestamp);

	if (GlobalState::goal_pose)
		selected_rrt->build_rrt_path_from_robot_pose(initial_robot_state);

//	publish_rrt_path_message(selected_rrt->path);
	selected_rrt->publish_status_message();
}


static void
simulator_ackerman_truepos_message_handler(carmen_simulator_ackerman_truepos_message *msg)
{
	Robot_State initial_robot_state;

	//estimate the initial robot state based on the localize pose and last odometry received
	initial_robot_state = GlobalState::predict_initial_robot_state(msg->truepose, msg->timestamp);

	if (GlobalState::goal_pose)
		selected_rrt->build_rrt_path_from_robot_pose(initial_robot_state);

//	publish_rrt_path_message(selected_rrt->path);
	selected_rrt->publish_status_message();
}


static void
base_ackerman_odometry_message_handler(carmen_base_ackerman_odometry_message *msg)
{
	GlobalState::last_odometry.v = msg->v;
	GlobalState::last_odometry.phi = msg->phi;
//
//	if (fabs(GlobalState::last_odometry.v) < GlobalState::param_distance_interval)
//		GlobalState::distance_interval = GlobalState::param_distance_interval;
//	else
//		GlobalState::distance_interval = fabs(GlobalState::last_odometry.v);
}


static void
navigator_ackerman_set_goal_message_handler(carmen_navigator_ackerman_set_goal_message *msg)
{
	Pose goal_pose;

	//na mensagem atual não é possível representar um goal nulo
	if (msg->x == -1 && msg->y == -1 && msg->theta == 0)
	{
		GlobalState::goal_pose = NULL;
		selected_rrt->clear_the_search_tree_and_current_path();

		if (selected_rrt->goal_pose)
		{
			delete selected_rrt->goal_pose;
			selected_rrt->goal_pose = NULL;
		}

		return;
	}

	goal_pose.x		= msg->x;
	goal_pose.y		= msg->y;
	goal_pose.theta = carmen_normalize_theta(msg->theta);

	GlobalState::last_goal = true;

	set_rrt_current_goal(goal_pose);
}


static void
carmen_rddf_play_end_point_message_handler(carmen_rddf_end_point_message *rddf_end_point_message)
{
	Pose goal_pose;

	goal_pose.x		= rddf_end_point_message->point.x;
	goal_pose.y		= rddf_end_point_message->point.y;
	goal_pose.theta		= rddf_end_point_message->point.theta;

	GlobalState::last_goal = true;

	set_rrt_current_goal(goal_pose);
}


static void
path_goals_and_annotations_message_handler(carmen_behavior_selector_path_goals_and_annotations_message *msg)
{
	Pose goal_pose;

	if ((msg->goal_list_size <= 0) || !msg->goal_list || !GlobalState::localize_pose)
		return;

	GlobalState::last_goal = (msg->goal_list_size == 1)? true: false;

	goal_pose.x = msg->goal_list->x;
	goal_pose.y = msg->goal_list->y;
	goal_pose.theta = carmen_normalize_theta(msg->goal_list->theta);

	GlobalState::robot_config.max_vel = fmin(msg->goal_list->v, GlobalState::param_max_vel);

	set_rrt_current_goal(goal_pose);
}


static void
behavior_selector_state_message_handler(carmen_behavior_selector_state_message *msg)
{
	if (1)//msg->algorithm != GlobalState::current_algorithm)
	{
		GlobalState::current_algorithm = msg->algorithm;

		if (1)//GlobalState::current_algorithm == CARMEN_BEHAVIOR_SELECTOR_RRT)
		{
			GlobalState::gradient_cost_map_old = true;

			selected_rrt->clear_the_search_tree_and_current_path();
		}
	}

	if (GlobalState::behavior_selector_task != msg->task)
	{
		GlobalState::behavior_selector_task = msg->task;

		if (GlobalState::behavior_selector_task == BEHAVIOR_SELECTOR_FOLLOW_ROUTE)
			selected_rrt = rrt_lane;
		else
			selected_rrt = rrt_parking;

		GlobalState::gradient_cost_map_old = true;
		selected_rrt->clear_the_search_tree_and_current_path();
	}

	if (msg->low_level_state_flags & CARMEN_BEHAVIOR_SELECTOR_ENGAGE_COLLISION_GEOMETRY)
		carmen_collision_detection_set_robot_collision_config(ENGAGE_GEOMETRY);
	else
		carmen_collision_detection_set_robot_collision_config(DEFAULT_GEOMETRY);
}


static void
map_server_compact_cost_map_message_handler(carmen_map_server_compact_cost_map_message *message)
{
	static carmen_compact_map_t *compact_cost_map = NULL;

	if (compact_cost_map == NULL)
	{
		carmen_grid_mapping_create_new_map(&GlobalState::cost_map, message->config.x_size, message->config.y_size, message->config.resolution, 'm');
		memset(GlobalState::cost_map.complete_map, 0, GlobalState::cost_map.config.x_size * GlobalState::cost_map.config.y_size * sizeof(double));

		compact_cost_map = (carmen_compact_map_t *) (calloc(1, sizeof(carmen_compact_map_t)));
		carmen_cpy_compact_map_message_to_compact_map(compact_cost_map, message);
		carmen_prob_models_uncompress_compact_map(&GlobalState::cost_map, compact_cost_map);
	}
	else
	{
		carmen_prob_models_clear_carmen_map_using_compact_map(&GlobalState::cost_map, compact_cost_map, 0.0);
		carmen_prob_models_free_compact_map(compact_cost_map);
		carmen_cpy_compact_map_message_to_compact_map(compact_cost_map, message);
		carmen_prob_models_uncompress_compact_map(&GlobalState::cost_map, compact_cost_map);
	}

	GlobalState::cost_map.config = message->config;

	GlobalState::cost_map_initialized = true;
}


static void
map_server_compact_lane_map_message_handler(carmen_map_server_compact_lane_map_message *message)
{
	static carmen_compact_map_t *compact_lane_map = NULL;

	if (compact_lane_map == NULL)
	{
		carmen_grid_mapping_create_new_map(&GlobalState::lane_map, message->config.x_size, message->config.y_size, message->config.resolution, 'm');

		for (int i = 0; i < GlobalState::lane_map.config.x_size * GlobalState::lane_map.config.y_size; ++i)
			GlobalState::lane_map.complete_map[i] = 1.0;

		compact_lane_map = (carmen_compact_map_t *) (calloc(1, sizeof(carmen_compact_map_t)));
		carmen_cpy_compact_lane_message_to_compact_map(compact_lane_map, message);
		carmen_prob_models_uncompress_compact_map(&GlobalState::lane_map, compact_lane_map);
	}
	else
	{
		carmen_prob_models_clear_carmen_map_using_compact_map(&GlobalState::lane_map, compact_lane_map, 1.0);
		carmen_prob_models_free_compact_map(compact_lane_map);
		carmen_cpy_compact_lane_message_to_compact_map(compact_lane_map, message);
		carmen_prob_models_uncompress_compact_map(&GlobalState::lane_map, compact_lane_map);
	}

	GlobalState::lane_map.config = message->config;
}


static void
signal_handler(int sig)
{
	printf("Signal %d received, exiting program ...\n", sig);
	exit(1);
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
register_handlers_specific()
{
	carmen_map_server_subscribe_compact_cost_map(
			NULL,
			(carmen_handler_t) map_server_compact_cost_map_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_map_server_subscribe_compact_lane_map(
			NULL, (carmen_handler_t) map_server_compact_lane_map_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_behavior_selector_subscribe_path_goals_and_annotations_message(
			NULL,
			(carmen_handler_t) path_goals_and_annotations_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *)CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_NAME,
			(char *)CARMEN_NAVIGATOR_ACKERMAN_SET_GOAL_FMT,
			NULL, sizeof(carmen_navigator_ackerman_set_goal_message),
			(carmen_handler_t)navigator_ackerman_set_goal_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_rddf_subscribe_end_point_message(NULL, (carmen_handler_t) (carmen_rddf_play_end_point_message_handler), CARMEN_SUBSCRIBE_LATEST);
}


void
register_handlers()
{
	signal(SIGINT, signal_handler);

	if (!GlobalState::use_truepos)
		carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
	else
		carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) simulator_ackerman_truepos_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) base_ackerman_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_behavior_selector_subscribe_current_state_message(NULL, (carmen_handler_t) behavior_selector_state_message_handler, CARMEN_SUBSCRIBE_LATEST);

	register_handlers_specific();

}
} // end namespace RRT_IPC


void
read_parameters_specific(int argc, char **argv)
{
	carmen_param_t optional_param_list[] =
	{
			{(char *) "rrt",	(char *) "rddf",					CARMEN_PARAM_STRING,	&GlobalState::rddf_path,				1, NULL},
			{(char *) "rrt",	(char *) "timeout",					CARMEN_PARAM_DOUBLE,	&GlobalState::timeout,					1, NULL},
			{(char *) "rrt",	(char *) "plan_time",				CARMEN_PARAM_DOUBLE,	&GlobalState::plan_time,				1, NULL},
			{(char *) "rrt",	(char *) "distance_interval",		CARMEN_PARAM_DOUBLE,	&GlobalState::distance_interval,		1, NULL},
			{(char *) "rrt",	(char *) "publish_lane_map",		CARMEN_PARAM_ONOFF,		&GlobalState::publish_lane_map,			1, NULL},
			{(char *) "rrt",	(char *) "publish_tree",			CARMEN_PARAM_ONOFF,		&GlobalState::publish_tree,				1, NULL},
			{(char *) "rrt",	(char *) "reuse_last_path",			CARMEN_PARAM_ONOFF,		&GlobalState::reuse_last_path,			1, NULL},
			{(char *) "rrt",	(char *) "obstacle_cost_distance",	CARMEN_PARAM_DOUBLE,	&GlobalState::obstacle_cost_distance,	1, NULL}
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, optional_param_list, sizeof(optional_param_list) / sizeof(optional_param_list[0]));

	GlobalState::param_distance_interval = GlobalState::distance_interval;

	printf("show_debug_info=%d\npublish_lane_map=%d\npublish_tree=%d\n",
			GlobalState::show_debug_info, GlobalState::publish_lane_map, GlobalState::publish_tree);
}


void
read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] =
	{
			{(char *) "robot",	(char *) "length",								  		CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.length,								 			1, NULL},
			{(char *) "robot",	(char *) "width",								  		CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.width,								 			1, NULL},
			{(char *) "robot", 	(char *) "distance_between_rear_wheels",		  		CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.distance_between_rear_wheels,			 		1, NULL},
			{(char *) "robot", 	(char *) "distance_between_front_and_rear_axles", 		CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.distance_between_front_and_rear_axles, 			1, NULL},
			{(char *) "robot", 	(char *) "distance_between_front_car_and_front_wheels",	CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.distance_between_front_car_and_front_wheels,	1, NULL},
			{(char *) "robot", 	(char *) "distance_between_rear_car_and_rear_wheels",	CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.distance_between_rear_car_and_rear_wheels,		1, NULL},
			{(char *) "robot", 	(char *) "max_velocity",						  		CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.max_vel,								 		1, NULL},
			{(char *) "robot", 	(char *) "max_steering_angle",					  		CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.max_phi,								 		1, NULL},
			{(char *) "robot", 	(char *) "maximum_acceleration_forward",				CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_acceleration_forward,					1, NULL},
			{(char *) "robot", 	(char *) "maximum_acceleration_reverse",				CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_acceleration_reverse,					1, NULL},
			{(char *) "robot", 	(char *) "maximum_deceleration_forward",				CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_deceleration_forward,					1, NULL},
			{(char *) "robot", 	(char *) "maximum_deceleration_reverse",				CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.maximum_deceleration_reverse,					1, NULL},
			{(char *) "robot", 	(char *) "desired_decelaration_forward",				CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.desired_decelaration_forward,					1, NULL},
			{(char *) "robot", 	(char *) "desired_decelaration_reverse",				CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.desired_decelaration_reverse,					1, NULL},
			{(char *) "robot", 	(char *) "desired_acceleration",						CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.desired_acceleration,							1, NULL},
			{(char *) "robot", 	(char *) "desired_steering_command_rate",				CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.desired_steering_command_rate,					1, NULL},
			{(char *) "robot", 	(char *) "understeer_coeficient",						CARMEN_PARAM_DOUBLE, &GlobalState::robot_config.understeer_coeficient,							1, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	GlobalState::param_max_vel = GlobalState::robot_config.max_vel;

	//initialize default parameters values
	GlobalState::use_truepos = 0;

	GlobalState::timeout = 0.8;
	GlobalState::distance_interval = 1.5;
	GlobalState::plan_time = 0.08;

	carmen_param_t optional_param_list[] =
	{
			{(char *) "rrt",	(char *) "cheat",				CARMEN_PARAM_ONOFF,		&GlobalState::use_truepos,			1, NULL},
			{(char *) "rrt",	(char *) "show_debug_info",		CARMEN_PARAM_ONOFF,		&GlobalState::show_debug_info,		1, NULL},
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, optional_param_list, sizeof(optional_param_list) / sizeof(optional_param_list[0]));

	read_parameters_specific(argc, argv);
}


int
main(int argc, char **argv)
{
	rrt_parking = new RRT_Parking();
	rrt_lane = new RRT_Lane();

	selected_rrt = rrt_lane;

	rrt_lane->set_change_path_condition(0.8, carmen_degrees_to_radians(8));
	rrt_lane->set_stop_condition(0.8, carmen_degrees_to_radians(30));

	rrt_parking->set_change_path_condition(0.8, carmen_degrees_to_radians(8));
	rrt_parking->set_stop_condition(1.0, carmen_degrees_to_radians(30));
	rrt_parking->set_max_distance_grad(12.0);
	rrt_parking->set_distance_near(12.0);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);

	Ackerman::init_search_parameters();
	rs_init_parameters(GlobalState::robot_config.max_phi, GlobalState::robot_config.distance_between_front_and_rear_axles);

	GlobalState::lane_points = Lane::get_street(GlobalState::rddf_path);

	RRT_IPC::register_handlers();

	printf("===============RRT Parameters===============\n");
	printf("Timeout: %f s\n", GlobalState::timeout);
	printf("Max distance interval %f m\n", GlobalState::distance_interval);
	printf("Use pose from simulator: %s\n", GlobalState::use_truepos ? "Yes" : "No");

	carmen_ipc_dispatch();

	return 0;
}
