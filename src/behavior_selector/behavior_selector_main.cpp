
#include <carmen/carmen.h>
#include <carmen/rddf_messages.h>
#include <carmen/path_planner_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/grid_mapping.h>
#include <prob_map.h>
#include <carmen/map_server_interface.h>
#include <carmen/obstacle_avoider_interface.h>
#include "g2o/types/slam2d/se2.h"

#include "behavior_selector.h"
#include "behavior_selector_messages.h"

using namespace g2o;

static int necessary_maps_available = 0;
static bool obstacle_avoider_active_recently = false;
static int activate_tracking = 0;


double param_distance_between_waypoints;
double param_change_goal_distance;
double param_distance_interval;

carmen_obstacle_avoider_robot_will_hit_obstacle_message last_obstacle_avoider_robot_hit_obstacle_message;
carmen_rddf_annotation_message last_rddf_annotation_message;
carmen_behavior_selector_goal_source_t last_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL;
carmen_behavior_selector_goal_source_t goal_list_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL;
//carmen_behavior_selector_goal_source_t goal_list_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_PATH_PLANNER_GOAL;
static int param_goal_source_onoff = 0;

int
annotation_is_forward(carmen_ackerman_traj_point_t robot_pose, carmen_vector_3D_t annotation_point)
{
	SE2 robot_pose_mat(robot_pose.x, robot_pose.y, robot_pose.theta);
	SE2 annotation_point_mat(annotation_point.x, annotation_point.y, 0.0);
	SE2 annotation_in_car_reference = robot_pose_mat.inverse() * annotation_point_mat;

	if (annotation_in_car_reference[0] > 0.0)
		return 1;
	else
		return 0;
}



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_current_state()
{
	IPC_RETURN_TYPE err;
	carmen_behavior_selector_state_message msg;
	carmen_behavior_selector_state_t current_state;
	carmen_behavior_selector_algorithm_t following_lane_planner;
	carmen_behavior_selector_algorithm_t parking_planner;
	carmen_behavior_selector_goal_source_t current_goal_source;

	behavior_selector_get_state(&current_state, &following_lane_planner, &parking_planner, &current_goal_source);

	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	msg.algorithm = get_current_algorithm();
	msg.state = current_state;

	msg.following_lane_algorithm = following_lane_planner;
	msg.parking_algorithm = parking_planner;

	msg.goal_source = current_goal_source;

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME, &msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME);
}


void
publish_goal_list()
{
	IPC_RETURN_TYPE err;
	carmen_behavior_selector_goal_list_message goal_list_msg;
	carmen_ackerman_traj_point_t *goal_list;
	int goal_list_size;
	int goal_list_index;
	double goal_list_time;

	behavior_selector_get_goal_list(&goal_list, &goal_list_size, &goal_list_index, &goal_list_time);

	goal_list_msg.timestamp = goal_list_time;
	goal_list_msg.host = carmen_get_host();
	goal_list_msg.goal_list = goal_list;
	goal_list_msg.size = goal_list_size;

	if (goal_list_msg.size > 0)
	{
		goal_list_msg.goal_list += goal_list_index;
		goal_list_msg.size -= goal_list_index;
	}

	carmen_ackerman_traj_point_t current_robot_pose_v_and_phi = get_robot_pose();
	double distance_to_act_on_annotation = (fabs(current_robot_pose_v_and_phi.v) < 2.0)? 10.0: fabs(current_robot_pose_v_and_phi.v) * 5.0;
	// Map annotations handling
	double distance_to_annotation = DIST2D(last_rddf_annotation_message.annotation_point, get_robot_pose());
	if ((distance_to_annotation < distance_to_act_on_annotation) &&
		annotation_is_forward(get_robot_pose(), last_rddf_annotation_message.annotation_point))
	{
		if ((last_rddf_annotation_message.annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			(last_rddf_annotation_message.annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_0))
			goal_list_msg.goal_list->v = 0.0;
		else if ((last_rddf_annotation_message.annotation_type == RDDF_ANNOTATION_TYPE_BUMP) ||
			(last_rddf_annotation_message.annotation_type == RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK))
			goal_list_msg.goal_list->v = carmen_fmin(3.0, goal_list_msg.goal_list->v);

		else if (last_rddf_annotation_message.annotation_type == RDDF_ANNOTATION_TYPE_BARRIER)
			goal_list_msg.goal_list->v = carmen_fmin(3.0, goal_list_msg.goal_list->v);

		else if ((last_rddf_annotation_message.annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			(last_rddf_annotation_message.annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_5))
			goal_list_msg.goal_list->v = carmen_fmin(5.0 / 3.6, goal_list_msg.goal_list->v);

		else if ((last_rddf_annotation_message.annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			(last_rddf_annotation_message.annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_10))
			goal_list_msg.goal_list->v = carmen_fmin(10.0 / 3.6, goal_list_msg.goal_list->v);

		else if ((last_rddf_annotation_message.annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			(last_rddf_annotation_message.annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_15))
			goal_list_msg.goal_list->v = carmen_fmin(15.0 / 3.6, goal_list_msg.goal_list->v);

		else if ((last_rddf_annotation_message.annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			(last_rddf_annotation_message.annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_20))
			goal_list_msg.goal_list->v = carmen_fmin(20.0 / 3.6, goal_list_msg.goal_list->v);

		else if ((last_rddf_annotation_message.annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			(last_rddf_annotation_message.annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_30))
			goal_list_msg.goal_list->v = carmen_fmin(30.0 / 3.6, goal_list_msg.goal_list->v);

		else if ((last_rddf_annotation_message.annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			(last_rddf_annotation_message.annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_40))
			goal_list_msg.goal_list->v = carmen_fmin(40.0 / 3.6, goal_list_msg.goal_list->v);

		else if ((last_rddf_annotation_message.annotation_type == RDDF_ANNOTATION_TYPE_SPEED_LIMIT) &&
			(last_rddf_annotation_message.annotation_code == RDDF_ANNOTATION_CODE_SPEED_LIMIT_60))
			goal_list_msg.goal_list->v = carmen_fmin(60.0 / 3.6, goal_list_msg.goal_list->v);
	}
	else if (obstacle_avoider_active_recently)
		goal_list_msg.goal_list->v = carmen_fmin(2.5, goal_list_msg.goal_list->v);
//	else
//		goal_list_msg.goal_list->v = 11.11;

	if (goal_list_msg.size > 0)
	{
		if (goal_list_road_profile_message == last_road_profile_message)
		{
			err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME, &goal_list_msg);
			carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME);
		}

		if (last_road_profile_message == CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL)
		{
			err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_RDDF_NAME, &goal_list_msg);
			carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_RDDF_NAME);
		}
	}
}


void
behavior_selector_publish_periodic_messages()
{
	publish_current_state();
	publish_goal_list();
}

///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	int update_state;
	static double last_time_obstacle_avoider_detected_obstacle = 0.0;
	carmen_ackerman_traj_point_t current_robot_pose_v_and_phi;

	if (!necessary_maps_available)
		return;

	current_robot_pose_v_and_phi.x = msg->globalpos.x;
	current_robot_pose_v_and_phi.y = msg->globalpos.y;
	current_robot_pose_v_and_phi.theta = msg->globalpos.theta;
	current_robot_pose_v_and_phi.v = msg->v;
	current_robot_pose_v_and_phi.phi = msg->phi;

	if (fabs(current_robot_pose_v_and_phi.v) < param_distance_interval)
		change_distance_between_waypoints_and_goals(param_distance_between_waypoints, param_change_goal_distance);
	else
		change_distance_between_waypoints_and_goals(4.0 * fabs(current_robot_pose_v_and_phi.v), 4.0 * fabs(current_robot_pose_v_and_phi.v));

	behavior_selector_update_robot_pose(current_robot_pose_v_and_phi, &update_state);
	if (update_state)
		publish_current_state();

	if (last_obstacle_avoider_robot_hit_obstacle_message.robot_will_hit_obstacle)
	{
		obstacle_avoider_active_recently = true;
		last_time_obstacle_avoider_detected_obstacle = last_obstacle_avoider_robot_hit_obstacle_message.timestamp;
	}
	else
	{
		if ((msg->timestamp - last_time_obstacle_avoider_detected_obstacle) > 2.0)
			obstacle_avoider_active_recently = false;
	}
	publish_goal_list();
}


static void
rddf_handler(carmen_rddf_road_profile_message *rddf_msg)
{
	if (!necessary_maps_available)
		return;

	behavior_selector_update_rddf(rddf_msg);
	last_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL;

	if (goal_list_road_profile_message == CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL)
	{
		carmen_behavior_selector_road_profile_message msg;
		msg.annotations = rddf_msg->annotations;
		msg.number_of_poses = rddf_msg->number_of_poses;
		msg.number_of_poses_back = rddf_msg->number_of_poses_back;
		msg.poses = rddf_msg->poses;
		msg.poses_back = rddf_msg->poses_back;
		msg.timestamp = carmen_get_time();
		msg.host = carmen_get_host();

		IPC_RETURN_TYPE err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME, &msg);
		carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME);
	}
}

static void
path_planner_road_profile_handler(carmen_path_planner_road_profile_message *rddf_msg)
{
	if (!necessary_maps_available)
		return;

	behavior_selector_update_rddf((carmen_rddf_road_profile_message *) rddf_msg);
	last_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_PATH_PLANNER_GOAL;

	if (goal_list_road_profile_message == CARMEN_BEHAVIOR_SELECTOR_PATH_PLANNER_GOAL)
	{
		carmen_behavior_selector_road_profile_message msg;
		msg.annotations = rddf_msg->annotations;
		msg.number_of_poses = rddf_msg->number_of_poses;
		msg.number_of_poses_back = rddf_msg->number_of_poses_back;
		msg.poses = rddf_msg->poses;
		msg.poses_back = rddf_msg->poses_back;
		msg.timestamp = carmen_get_time();
		msg.host = carmen_get_host();

		IPC_RETURN_TYPE err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME, &msg);
		carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME);
	}
}


//static void
//grid_mapping_handler(carmen_grid_mapping_message *msg)
//{
//	carmen_map_t map;
//	int goal_list_updated;
//
//	map.complete_map = msg->complete_map;
//	map.config = msg->config;
//
//	behavior_selector_update_map(&map, &goal_list_updated);
//
//	necessary_maps_available = 1;
//}


static void
map_server_compact_cost_map_message_handler(carmen_map_server_compact_cost_map_message *message)
{
	static carmen_compact_map_t *compact_cost_map = NULL;
	static carmen_map_t cost_map;
	int goal_list_updated;

	if (compact_cost_map == NULL)
	{
		carmen_grid_mapping_create_new_map(&cost_map, message->config.x_size, message->config.y_size, message->config.resolution);
		memset(cost_map.complete_map, 0, cost_map.config.x_size * cost_map.config.y_size * sizeof(double));

		compact_cost_map = (carmen_compact_map_t*) (calloc(1, sizeof(carmen_compact_map_t)));
		carmen_cpy_compact_cost_message_to_compact_map(compact_cost_map, message);
		carmen_prob_models_uncompress_compact_map(&cost_map, compact_cost_map);
	}
	else
	{
		carmen_prob_models_clear_carmen_map_using_compact_map(&cost_map, compact_cost_map, 0.0);
		carmen_prob_models_free_compact_map(compact_cost_map);
		carmen_cpy_compact_cost_message_to_compact_map(compact_cost_map, message);
		carmen_prob_models_uncompress_compact_map(&cost_map, compact_cost_map);
	}

	cost_map.config = message->config;

	behavior_selector_update_map(&cost_map, &goal_list_updated);

	necessary_maps_available = 1;
}


static void
obstacle_avoider_robot_hit_obstacle_message_handler(carmen_obstacle_avoider_robot_will_hit_obstacle_message *robot_hit_obstacle_message)
{
	last_obstacle_avoider_robot_hit_obstacle_message = *robot_hit_obstacle_message;
}


static void
rddf_annotation_message_handler(carmen_rddf_annotation_message *message)
{
	double distance_to_last_annotation = DIST2D(last_rddf_annotation_message.annotation_point, get_robot_pose());
	double distance_to_new_annotation = DIST2D(message->annotation_point, get_robot_pose());

	if (distance_to_new_annotation < distance_to_last_annotation)
		last_rddf_annotation_message = *message; // TODO: tratar isso direito
}


static void
set_algorith_handler(carmen_behavior_selector_set_algorithm_message *msg)
{
	behavior_selector_set_algorithm(msg->algorithm, msg->state);
	publish_current_state();
}


static void
set_goal_source_handler(carmen_behavior_selector_set_goal_source_message *msg)
{
	if (behavior_selector_set_goal_source(msg->goal_source))
		publish_current_state();
}


static void
set_state_handler(carmen_behavior_selector_set_state_message *msg)
{
	if (behavior_selector_set_state(msg->state))
		publish_current_state();
}


static void
add_goal_handler(carmen_behavior_selector_add_goal_message *msg)
{
	behavior_selector_add_goal(msg->goal);
}


static void
clear_goal_list_handler()
{
	behavior_selector_clear_goal_list();
}


static void
remove_goal_handler()
{
	behavior_selector_remove_goal();
}


static void
signal_handler(int signo __attribute__ ((unused)) )
{
	carmen_ipc_disconnect();
	exit(0);
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
register_handlers()
{
	carmen_obstacle_avoider_subscribe_robot_hit_obstacle_message(
			NULL,
			(carmen_handler_t) obstacle_avoider_robot_hit_obstacle_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_rddf_subscribe_road_profile_message(
			NULL,(carmen_handler_t)rddf_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_localize_ackerman_subscribe_globalpos_message(
			NULL, (carmen_handler_t) localize_globalpos_handler,
			CARMEN_SUBSCRIBE_LATEST);

//	carmen_grid_mapping_subscribe_message(
//			NULL, (carmen_handler_t) grid_mapping_handler,
//			CARMEN_SUBSCRIBE_LATEST);
//
	carmen_map_server_subscribe_compact_cost_map(
			NULL, (carmen_handler_t) map_server_compact_cost_map_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	// esse handler eh subscribe_all porque todas as anotacoes precisam ser recebidas!
	carmen_rddf_subscribe_annotation_message(NULL,
			(carmen_handler_t) rddf_annotation_message_handler,
			CARMEN_SUBSCRIBE_ALL);

	// **************************************************
	// filipe:: TODO: criar funcoes de subscribe no interfaces!
	// **************************************************

    carmen_subscribe_message((char *) CARMEN_PATH_PLANNER_ROAD_PROFILE_MESSAGE_NAME,
			(char *) CARMEN_PATH_PLANNER_ROAD_PROFILE_MESSAGE_FMT,
			NULL, sizeof (carmen_path_planner_road_profile_message),
			(carmen_handler_t)path_planner_road_profile_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_NAME,
			(char *) CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_FMT,
			NULL, sizeof(carmen_behavior_selector_set_algorithm_message),
			(carmen_handler_t)set_algorith_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_SET_GOAL_SOURCE_NAME,
			(char *) CARMEN_BEHAVIOR_SELECTOR_SET_GOAL_SOURCE_FMT,
			NULL, sizeof(carmen_behavior_selector_set_goal_source_message),
			(carmen_handler_t)set_goal_source_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_SET_STATE_NAME,
			(char *) CARMEN_BEHAVIOR_SELECTOR_SET_STATE_FMT,
			NULL, sizeof(carmen_behavior_selector_set_state_message),
			(carmen_handler_t)set_state_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_ADD_GOAL_NAME,
			(char *) CARMEN_BEHAVIOR_SELECTOR_ADD_GOAL_FMT,
			NULL, sizeof(carmen_behavior_selector_add_goal_message),
			(carmen_handler_t)add_goal_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_CLEAR_GOAL_LIST_NAME,
			(char *) CARMEN_BEHAVIOR_SELECTOR_CLEAR_GOAL_LIST_FMT,
			NULL, sizeof(carmen_behavior_selector_clear_goal_list_message),
			(carmen_handler_t)clear_goal_list_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_REMOVE_GOAL_NAME,
			(char *) CARMEN_BEHAVIOR_SELECTOR_REMOVE_GOAL_FMT,
			NULL, sizeof(carmen_behavior_selector_remove_goal_message),
			(carmen_handler_t)remove_goal_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_addPeriodicTimer(1, (TIMER_HANDLER_TYPE) behavior_selector_publish_periodic_messages, NULL);
}


static void
define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_FMT);
	carmen_test_ipc_exit(err, "Could not define message",
			CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME);

	err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_FMT);
	carmen_test_ipc_exit(err, "Could not define message",
			CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME);

	err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_RDDF_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_RDDF_FMT);
	carmen_test_ipc_exit(err, "Could not define message",
			CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_RDDF_NAME);
}


static void
read_parameters(int argc, char **argv)
{
	carmen_robot_ackerman_config_t robot_config;
	//	carmen_map_t map;
	double distance_between_waypoints, change_goal_distance, distance_to_remove_annotation_goal;
	carmen_behavior_selector_algorithm_t parking_planner, following_lane_planner;

	carmen_param_t param_list[] =
	{
			{(char *) "robot", (char *) "max_v", CARMEN_PARAM_DOUBLE, &robot_config.max_v, 1, NULL},
			{(char *) "robot", (char *) "max_steering_angle", CARMEN_PARAM_DOUBLE, &robot_config.max_phi, 1, NULL},
			{(char *) "robot", (char *) "length", CARMEN_PARAM_DOUBLE, &robot_config.length, 0, NULL},
			{(char *) "robot", (char *) "width", CARMEN_PARAM_DOUBLE, &robot_config.width, 0, NULL},
			{(char *) "robot", (char *) "maximum_acceleration_forward", CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_forward, 1, NULL},
			{(char *) "robot", (char *) "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_and_rear_axles, 1, NULL},
			{(char *) "robot", (char *) "distance_between_rear_car_and_rear_wheels", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_car_and_rear_wheels, 1, NULL}, 			
			{(char *) "behavior_selector", (char *) "distance_between_waypoints", CARMEN_PARAM_DOUBLE, &distance_between_waypoints, 1, NULL},
			{(char *) "behavior_selector", (char *) "change_goal_distance", CARMEN_PARAM_DOUBLE, &change_goal_distance, 1, NULL},
			{(char *) "behavior_selector", (char *) "following_lane_planner", CARMEN_PARAM_INT, &following_lane_planner, 1, NULL},
			{(char *) "behavior_selector", (char *) "parking_planner", CARMEN_PARAM_INT, &parking_planner, 1, NULL},
			{(char *) "behavior_selector", (char *) "goal_source_path_planner", CARMEN_PARAM_ONOFF, &param_goal_source_onoff, 0, NULL},
			{(char *) "behavior_selector", (char *) "distance_to_remove_annotation_goal", CARMEN_PARAM_DOUBLE, &distance_to_remove_annotation_goal, 0, NULL},
			{(char *) "rrt",   (char *) "distance_interval", CARMEN_PARAM_DOUBLE, &param_distance_interval, 1, NULL}
	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list)/sizeof(param_list[0]));


	carmen_param_allow_unfound_variables(1);
	carmen_param_t optional_param_list[] =
	{
			{(char *) "commandline", (char *) "activate_tracking", CARMEN_PARAM_ONOFF, &activate_tracking, 0, NULL}
	};
	carmen_param_install_params(argc, argv, optional_param_list, sizeof(optional_param_list) / sizeof(optional_param_list[0]));

	printf("activate_tracking %d\n", activate_tracking);

	param_distance_between_waypoints = distance_between_waypoints;
	param_change_goal_distance = change_goal_distance;
	behavior_selector_initialize(robot_config, distance_between_waypoints,
		change_goal_distance, following_lane_planner, parking_planner, distance_to_remove_annotation_goal);

	if (param_goal_source_onoff)
		goal_list_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_PATH_PLANNER_GOAL;
	else
		goal_list_road_profile_message = CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL;
}


int
main(int argc, char **argv)
{
	signal(SIGINT, signal_handler);
	carmen_ipc_initialize(argc, argv);
	define_messages();
	read_parameters(argc, argv);

	register_handlers();

	memset(&last_rddf_annotation_message, 0, sizeof(last_rddf_annotation_message));

	carmen_ipc_dispatch();

	return 0;
}
