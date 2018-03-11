
#include "carmen_comm.h"

#include <cstdio>
#include <unistd.h>
#include <carmen/carmen.h>
#include "g2o/types/slam2d/se2.h"
#include <carmen/collision_detection.h>
#include <carmen/robot_ackerman_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/obstacle_avoider_interface.h>
#include <carmen/obstacle_distance_mapper_interface.h>


const double MAX_V = 20.0;
const double MAX_PHI = M_PI;
const double MAX_DELTA = 10.0; // max dx or dy between the goal and the pose


carmen_localize_ackerman_globalpos_message global_localize_ackerman_message;
carmen_obstacle_distance_mapper_compact_map_message global_obstacle_distance_mapper_compact_map_message;
carmen_obstacle_distance_mapper_map_message global_obstacle_distance_map;
carmen_robot_ackerman_config_t global_robot_ackerman_config;


void
publish_starting_pose(double x, double y, double th)
{
	carmen_localize_ackerman_globalpos_message localize_ackerman_message;
	memset(&localize_ackerman_message, 0, sizeof(localize_ackerman_message));

	carmen_point_t pose;
	carmen_point_t std;

	pose.x = x;
	pose.y = y;
	pose.theta = th;

	std.x = 0.001;
	std.y = 0.001;
	std.theta = carmen_degrees_to_radians(0.01);

	carmen_localize_ackerman_initialize_gaussian_command(pose, std);
}


void
publish_command(double v, double phi)
{
	static const int NUM_MOTION_COMMANDS = 20;
	static carmen_ackerman_motion_command_t *motion_commands = NULL;

	if (motion_commands == NULL)
		motion_commands = (carmen_ackerman_motion_command_t *) calloc (NUM_MOTION_COMMANDS, sizeof(carmen_ackerman_motion_command_t));

	for (int i = 0; i < NUM_MOTION_COMMANDS; i++)
	{
			motion_commands[i].time = 0.1 * i;
			motion_commands[i].v = v;
			motion_commands[i].phi = phi;
	}

	carmen_robot_ackerman_publish_motion_command(motion_commands,
		NUM_MOTION_COMMANDS, global_localize_ackerman_message.timestamp);
}


void
publish_goal_list(double x, double y, double th, double v, double phi, double timestamp)
{
	static carmen_ackerman_traj_point_t *goal_list = NULL;
	static carmen_behavior_selector_goal_list_message goal_list_msg;

	if (goal_list == NULL)
		goal_list = (carmen_ackerman_traj_point_t *) calloc (10, sizeof(carmen_ackerman_traj_point_t));

	goal_list[0].x = x;
	goal_list[0].y = y;
	goal_list[0].theta = th;
	goal_list[0].v = v;
	goal_list[0].phi = phi;

	goal_list_msg.goal_list = goal_list;
	goal_list_msg.size = 1;
	goal_list_msg.timestamp = timestamp;
	goal_list_msg.host = carmen_get_host();

	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME, &goal_list_msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME);
}


void
publish_current_state()
{
	IPC_RETURN_TYPE err;

	carmen_behavior_selector_state_message msg;

	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	msg.algorithm = CARMEN_BEHAVIOR_SELECTOR_RRT;
	msg.state = BEHAVIOR_SELECTOR_FOLLOWING_LANE;

	msg.following_lane_algorithm = CARMEN_BEHAVIOR_SELECTOR_RRT;
	msg.parking_algorithm = CARMEN_BEHAVIOR_SELECTOR_RRT;

	msg.goal_source = CARMEN_BEHAVIOR_SELECTOR_RDDF_GOAL;

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME, &msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME);
}


double
normalize(double x, double max_x)
{
	return x / max_x;
}


std::vector<double>
read_state()
{
	carmen_ipc_sleep(1e-4);
	std::vector<double> state;

	carmen_localize_ackerman_globalpos_message globalpos = global_localize_ackerman_message;

	state.push_back(normalize(globalpos.globalpos.x, MAX_DELTA));
	state.push_back(normalize(globalpos.globalpos.y, MAX_DELTA));
	state.push_back(normalize(globalpos.globalpos.theta, M_PI));
	state.push_back(normalize(globalpos.v, MAX_V));
	state.push_back(normalize(globalpos.phi, MAX_PHI));

	return state;
}


int
car_hit_obstacle()
{
	carmen_ackerman_traj_point_t pose;

	pose.x = global_localize_ackerman_message.globalpos.x;
	pose.y = global_localize_ackerman_message.globalpos.y;
	pose.theta = global_localize_ackerman_message.globalpos.theta;
	pose.phi = global_localize_ackerman_message.phi;
	pose.v = global_localize_ackerman_message.v;

	int hit = trajectory_pose_hit_obstacle(pose,
		global_robot_ackerman_config.obstacle_avoider_obstacles_safe_distance,
		&global_obstacle_distance_map, &global_robot_ackerman_config);

	return hit;
}


void
env_destroy()
{
	publish_command(0, 0);
	carmen_ipc_disconnect();
	exit(0);
}


void
define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME, IPC_VARIABLE_LENGTH,
		CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_FMT);

	carmen_test_ipc_exit(err, "Could not define message",
		CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME);
}


void
signal_handler(int signo)
{
	if (signo == SIGINT)
		env_destroy();
}


void
process_map_message(carmen_obstacle_distance_mapper_compact_map_message *message)
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

	global_obstacle_distance_map = distance_map;
}


int
read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
		{"obstacle_avoider", "obstacles_safe_distance", CARMEN_PARAM_DOUBLE,	&global_robot_ackerman_config.obstacle_avoider_obstacles_safe_distance, 1, NULL},
		{"robot", "max_velocity", CARMEN_PARAM_DOUBLE,	&global_robot_ackerman_config.max_v, 1, NULL},
		{"robot", "max_steering_angle", CARMEN_PARAM_DOUBLE, &global_robot_ackerman_config.max_phi, 1, NULL},
		{"robot", "min_approach_dist", CARMEN_PARAM_DOUBLE,	&global_robot_ackerman_config.approach_dist, 1, NULL},
		{"robot", "min_side_dist", CARMEN_PARAM_DOUBLE,	&global_robot_ackerman_config.side_dist, 1, NULL},
		{"robot", "length", CARMEN_PARAM_DOUBLE, &global_robot_ackerman_config.length, 0, NULL},
		{"robot", "width", CARMEN_PARAM_DOUBLE, &global_robot_ackerman_config.width, 0, NULL},
		{"robot", "maximum_acceleration_forward", CARMEN_PARAM_DOUBLE, &global_robot_ackerman_config.maximum_acceleration_forward, 1, NULL},
		{"robot", "maximum_deceleration_forward", CARMEN_PARAM_DOUBLE, &global_robot_ackerman_config.maximum_deceleration_forward, 1, NULL},
		{"robot", "reaction_time", CARMEN_PARAM_DOUBLE, &global_robot_ackerman_config.reaction_time, 0, NULL},
		{"robot", "distance_between_rear_wheels", CARMEN_PARAM_DOUBLE, &global_robot_ackerman_config.distance_between_rear_wheels, 1,NULL},
		{"robot", "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &global_robot_ackerman_config.distance_between_front_and_rear_axles, 1, NULL},
		{"robot", "distance_between_front_car_and_front_wheels", CARMEN_PARAM_DOUBLE, &global_robot_ackerman_config.distance_between_front_car_and_front_wheels, 1, NULL},
		{"robot", "distance_between_rear_car_and_rear_wheels", CARMEN_PARAM_DOUBLE, &global_robot_ackerman_config.distance_between_rear_car_and_rear_wheels, 1, NULL},
		{"robot", "maximum_steering_command_rate", CARMEN_PARAM_DOUBLE, &global_robot_ackerman_config.maximum_steering_command_rate, 1, NULL},
		{"robot", "understeer_coeficient", CARMEN_PARAM_DOUBLE, &global_robot_ackerman_config.understeer_coeficient, 1, NULL},
		{"robot", "allow_rear_motion", CARMEN_PARAM_ONOFF, &global_robot_ackerman_config.allow_rear_motion, 1, NULL},
		{"robot", "interpolate_odometry", CARMEN_PARAM_ONOFF, &global_robot_ackerman_config.interpolate_odometry, 1, NULL},
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}


void
env_init()
{
	char *argv[] = {"rl_motion_planner"};
	carmen_ipc_initialize(1, argv);
	read_parameters(1, argv);

	define_messages();

    carmen_localize_ackerman_subscribe_globalpos_message(&global_localize_ackerman_message,
    	NULL, CARMEN_SUBSCRIBE_LATEST);

	carmen_obstacle_distance_mapper_subscribe_compact_map_message(&global_obstacle_distance_mapper_compact_map_message,
		NULL, CARMEN_SUBSCRIBE_LATEST);

	signal(SIGINT, signal_handler);
}


std::vector<double>
env_reset(double pos_x, double pos_y, double pos_th,
		double goal_x, double goal_y, double goal_th,
		double goal_v, double goal_phi)
{
	memset(&global_localize_ackerman_message, 0, sizeof(carmen_localize_ackerman_globalpos_message));

	do
	{
		publish_command(0, 0);
		publish_starting_pose(pos_x, pos_y, pos_th);
		publish_goal_list(goal_x, goal_y, goal_th, goal_v, goal_phi, carmen_get_time());
		publish_current_state();
		carmen_ipc_sleep(1e-4);
	}
	while (global_localize_ackerman_message.timestamp == 0);

	return read_state();
}


std::vector<double>
env_step(double v, double phi, double goal_x, double goal_y, double goal_th, double goal_v, double goal_phi)
{
	publish_command(v, phi);
	publish_goal_list(goal_x, goal_y, goal_th, goal_v, goal_phi, carmen_get_time());
	publish_current_state();

	usleep(1e4);

	process_map_message(&global_obstacle_distance_mapper_compact_map_message);
	return read_state();
}


bool
env_done()
{
	int hit = car_hit_obstacle();
	printf("hit: %d\n", hit);
	return hit;
}

