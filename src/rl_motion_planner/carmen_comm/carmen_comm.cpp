
#include "carmen_comm.h"

#include <cmath>
#include <cstdio>
#include <unistd.h>
#include <carmen/carmen.h>
#include <carmen/control.h>
#include "g2o/types/slam2d/se2.h"
#include <carmen/laser_interface.h>
#include <carmen/objects_ackerman.h>
#include <carmen/collision_detection.h>
#include <carmen/mapper_interface.h>
#include <carmen/robot_ackerman_interface.h>
#include <carmen/obstacle_avoider_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/simulator_ackerman_simulation.h>
#include <carmen/simulator_ackerman_interface.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/navigator_ackerman_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/rddf_interface.h>
#include <locale.h>
#include "util.h"
#include <assert.h>

// Flags
const int VIEW_LASER = 0;
const int VIEW_SIMULATED_LASER = 0;

// Parameters
carmen_robot_ackerman_config_t global_robot_ackerman_config;

// Messages
carmen_laser_laser_message global_front_laser_message;
carmen_laser_laser_message global_rear_laser_message;
carmen_mapper_map_message global_mapper_map_message;
carmen_behavior_selector_goal_list_message global_goal_list_message;
carmen_obstacle_distance_mapper_map_message global_obstacle_distance_map;
carmen_simulator_ackerman_truepos_message global_truepos_message;
carmen_localize_ackerman_globalpos_message global_localize_ackerman_message;
carmen_obstacle_distance_mapper_compact_map_message global_obstacle_distance_mapper_compact_map_message;
carmen_robot_ackerman_motion_command_message global_motion_command_message;
carmen_rddf_road_profile_message global_rddf_message;

// Termination signal handling
int global_destroy_already_requested = 0;
int autonomous_mode = 0;

// Simulation
carmen_simulator_ackerman_config_t simulator_config;
carmen_laser_laser_message global_simulated_front_laser;


/**
 * *******************************************************************************
 * The functions below were copied from simulator_ackerman.c
 * TODO: Improve the simulator_ackerman interface to make these functions public.
 * *******************************************************************************
 */

void
fill_laser_config_data(carmen_simulator_ackerman_laser_config_t *lasercfg)
{
	lasercfg->num_lasers = 1 + carmen_round(lasercfg->fov / lasercfg->angular_resolution);
	lasercfg->start_angle = -0.5*lasercfg->fov;

	/* give a warning if it is not a standard configuration */

	if (fabs(lasercfg->fov - M_PI) > 1e-6 &&
			fabs(lasercfg->fov - 100.0/180.0 * M_PI) > 1e-6 &&
			fabs(lasercfg->fov -  90.0/180.0 * M_PI) > 1e-6)
		carmen_warn("Warning: You are not using a standard SICK configuration (fov=%.4f deg)\n",
				carmen_radians_to_degrees(lasercfg->fov));

	if (fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(1.0)) > 1e-6 &&
			fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(0.5)) > 1e-6 &&
			fabs(lasercfg->angular_resolution - carmen_degrees_to_radians(0.25)) > 1e-6)
		carmen_warn("Warning: You are not using a standard SICK configuration (res=%.4f deg)\n",
				carmen_radians_to_degrees(lasercfg->angular_resolution));
}


int
apply_system_latencies(carmen_ackerman_motion_command_p current_motion_command_vector, int nun_motion_commands)
{
	int i, j;

	for (i = 0; i < nun_motion_commands; i++)
	{
		j = i;
		for (double lat = 0.0; lat < 0.2; j++)
		{
			if (j >= nun_motion_commands)
				break;
			lat += current_motion_command_vector[j].time;
		}
		if (j >= nun_motion_commands)
			break;
		current_motion_command_vector[i].phi = current_motion_command_vector[j].phi;
//		current_motion_command_vector[i].phi = current_motion_command_vector[j].phi;
//		current_motion_command_vector[i].v = current_motion_command_vector[j].v;
//		current_motion_command_vector[i].x = current_motion_command_vector[j].x;
//		current_motion_command_vector[i].y = current_motion_command_vector[j].y;
//		current_motion_command_vector[i].theta = current_motion_command_vector[j].theta;
//		current_motion_command_vector[i].time = current_motion_command_vector[j].time;
	}

	for (i = 0; i < nun_motion_commands; i++)
	{
		j = i;
		for (double lat = 0.0; lat < 0.6; j++)
		{
			if (j >= nun_motion_commands)
				break;
			lat += current_motion_command_vector[j].time;
		}
		if (j >= nun_motion_commands)
			break;
		current_motion_command_vector[i].v = current_motion_command_vector[j].v;
	}

	return (i);
}

int
hit_something_in_the_map(carmen_simulator_ackerman_config_t *simulator_config, carmen_point_t new_true)
{
	//	Verificando colisão completa
	//	carmen_robot_ackerman_config_t robot_config;
	//
	//	robot_config.distance_between_rear_car_and_rear_wheels = simulator_config->distance_between_rear_car_and_rear_wheels;
	//	robot_config.width = simulator_config->width;
	//	robot_config.length = simulator_config->length;
	//
	//	return pose_hit_obstacle(new_true, &simulator_config->map, &robot_config);

	carmen_map_p map;
	int map_x, map_y;

	map = &(simulator_config->map);
	map_x = (new_true.x - map->config.x_origin) / map->config.resolution;
	map_y = (new_true.y - map->config.y_origin) / map->config.resolution;

	if (map_x < 0 || map_x >= map->config.x_size ||
			map_y < 0 || map_y >= map->config.y_size ||
			map->map[map_x][map_y] > .15 ||					// @@@ Tem que tratar o robo retangular...
			carmen_simulator_object_too_close(new_true.x, new_true.y, -1)) 	// @@@ Tem que tratar o robo retangular...
		return (1);

	return (0);
}

/*
 * *******************************************************************************
 * End of copy.
 * *******************************************************************************
 */


void
publish_behavior_selector_state()
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
	msg.low_level_state = Free_Running;

	msg.behaviour_seletor_mode = none;

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME, &msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME);
}


// TODO: copied from model_predictive_planner/publisher_util because model predictive doesn't
// expose this function and the Tree type.
void
publish_navigator_ackerman_plan_message(std::vector<double> &vs, std::vector<double> &phis,
		std::vector<double> &xs, std::vector<double> &ys, std::vector<double> &ths,
		double timestamp)
{
	static int path_size = 0;
	static carmen_ackerman_traj_point_t *path = NULL;
	static carmen_navigator_ackerman_plan_message msg;

	static bool first_time = true;
	IPC_RETURN_TYPE err;

	if (first_time)
	{
		err = IPC_defineMsg(CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME,
				IPC_VARIABLE_LENGTH, CARMEN_NAVIGATOR_ACKERMAN_PLAN_FMT);

		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME);

		path = (carmen_ackerman_traj_point_t *) calloc (vs.size(), sizeof(carmen_ackerman_traj_point_t));
		path_size = vs.size();

		first_time = false;
	}

	if (path_size != vs.size())
	{
		path_size = vs.size();
		path = (carmen_ackerman_traj_point_t *) realloc (path, vs.size() * sizeof(carmen_ackerman_traj_point_t));
	}

	// copy path and mask
	for (int i = 0; i < vs.size(); i++)
	{
		path[i].x = xs[i];
		path[i].y = ys[i];
		path[i].theta = ths[i];
		path[i].v = vs[i];
		path[i].phi = phis[i];
	}

	msg.host = carmen_get_host();
	msg.timestamp = timestamp;
	msg.path_length = vs.size();
	msg.path = path;

	err = IPC_publishData(CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME, &msg);
	carmen_test_ipc(err, "Could not publish",
			CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME);
}


void
publish_plan_tree_message(std::vector<double> vs, std::vector<double> phis,
	std::vector<double> xs, std::vector<double> ys, std::vector<double> ths)
{
	IPC_RETURN_TYPE err = IPC_OK;

	carmen_navigator_ackerman_plan_tree_message plan_tree_msg;

	int size = (int) vs.size();

	if (size <= 0)
		return;

	memset(plan_tree_msg.path_size, 0, 500 * sizeof(int));
	for (int i = 0; i < 500; i++)
		for (int j = 0; j < 100; j++)
			memset(&(plan_tree_msg.paths[i][j]), 0, sizeof(carmen_ackerman_traj_point_t));

	plan_tree_msg.num_path = 1;
	plan_tree_msg.path_size[0] = size;

	// copy path and mask
	printf("size: %d\n", size);
	for (int i = 0; i < size; i++)
	{
		plan_tree_msg.paths[0][i].x = xs[i];
		plan_tree_msg.paths[0][i].y = ys[i];
		plan_tree_msg.paths[0][i].theta = ths[i];
		plan_tree_msg.paths[0][i].v = vs[i];
		plan_tree_msg.paths[0][i].phi = phis[i];
		printf("%f %f %f\n", xs[i], ys[i], ths[i]);
	}

	plan_tree_msg.num_edges = 0;
	plan_tree_msg.p1 = NULL;
	plan_tree_msg.p2 = NULL;
	plan_tree_msg.mask = NULL;

	plan_tree_msg.timestamp = carmen_get_time();
	plan_tree_msg.host = carmen_get_host();

	printf("publishing!\n");
	err = IPC_publishData(CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME, &plan_tree_msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME);
	printf("done.\n");
}


void
publish_path_to_draw(std::vector<double> vs, std::vector<double> phis,
		std::vector<double> xs, std::vector<double> ys, std::vector<double> ths)
{
	carmen_navigator_ackerman_plan_to_draw_message  message;

	int path_size = vs.size();
	carmen_ackerman_traj_point_t *path = (carmen_ackerman_traj_point_t *) calloc (vs.size(), sizeof(carmen_ackerman_traj_point_t));

	for (int i = 0; i < vs.size(); i++)
	{
		path[i].x = xs[i];
		path[i].y = ys[i];
		path[i].theta = ths[i];
		path[i].v = vs[i];
		path[i].phi = phis[i];
	}

	message.host = carmen_get_host();
	message.timestamp = carmen_get_time();
	message.path_size = path_size;
	message.path = path;

	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_NAVIGATOR_ACKERMAN_PLAN_TO_DRAW_NAME, &message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_NAVIGATOR_ACKERMAN_PLAN_TO_DRAW_NAME);

	free(path);
}


void
publish_command(std::vector<double> v, std::vector<double> phi, std::vector<double> dt,
		int publish_behavior_selector_state_flag, double x, double y, double th)
{
	if (v.size() == 0)
	{
		printf("Empty command list! Ignoring commands!\n");
		return;
	}

	setlocale(LC_ALL, "en_US.UTF-8");

	static int n_motion_commands = 0;
	static carmen_ackerman_motion_command_t *motion_commands = NULL;

	if (motion_commands == NULL)
		motion_commands = (carmen_ackerman_motion_command_t *) calloc (v.size(), sizeof(carmen_ackerman_motion_command_t));
	else if (n_motion_commands != v.size())
		motion_commands = (carmen_ackerman_motion_command_t *) realloc (motion_commands, v.size() * sizeof(carmen_ackerman_motion_command_t));

	n_motion_commands = v.size();

	double base_time = carmen_get_time();

	std::vector<double> xs, ys, ths;

	for (int i = 0; i < n_motion_commands; i++)
	{
		ackerman_motion_model(&x, &y, &th,
			v[i], phi[i], dt[i],
			simulator_config.distance_between_front_and_rear_axles);

		motion_commands[i].x = x;
		motion_commands[i].y = y;
		motion_commands[i].theta = th;

		motion_commands[i].time = dt[i];

		if (autonomous_mode)
		{
		    //printf("AUTONOMOUS MODE ON.\n");
			motion_commands[i].v = v[i];
			motion_commands[i].phi = phi[i];
		}
		else
		{
		    //printf("AUTONOMOUS MODE OFF.\n");
			motion_commands[i].v = 0.0;
			motion_commands[i].phi = 0.0;
		}


		xs.push_back(x);
		ys.push_back(y);
		ths.push_back(th);
	}

	if (publish_behavior_selector_state_flag)
		publish_behavior_selector_state();

	assert(v.size() == xs.size());
	assert(v.size() == ys.size());
	assert(v.size() == ths.size());

	publish_path_to_draw(v, phi, xs, ys, ths);

	carmen_robot_ackerman_publish_motion_command(motion_commands,
		n_motion_commands, base_time);
}


void
publish_stop_command()
{
	std::vector<double> v;
	std::vector<double> phi;
	std::vector<double> dt;

	double base_time = carmen_get_time();

	for (int i = 0; i < 20; i++)
	{
		v.push_back(0.);
		phi.push_back(0.);
		dt.push_back(base_time + i * 0.1);
	}

	publish_command(v, phi, dt, 0);
}


void
publish_goal_list(std::vector<double> x, std::vector<double> y,
		std::vector<double> th, std::vector<double> v, std::vector<double> phi,
		double timestamp)
{
	static carmen_ackerman_traj_point_t *goal_list = NULL;
	static carmen_behavior_selector_goal_list_message goal_list_msg;

	if (goal_list == NULL)
		goal_list = (carmen_ackerman_traj_point_t *) calloc (x.size(), sizeof(carmen_ackerman_traj_point_t));

	for (int i = 0; i < x.size(); i++)
	{
		goal_list[i].x = x[i];
		goal_list[i].y = y[i];
		goal_list[i].theta = th[i];
		goal_list[i].v = v[i];
		goal_list[i].phi = phi[i];
	}

	goal_list_msg.goal_list = goal_list;
	goal_list_msg.size = 1;
	goal_list_msg.timestamp = timestamp;
	goal_list_msg.host = carmen_get_host();

	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME, &goal_list_msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME);
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


void
handle_messages(double how_long)
{
	double time_previous_globalpos = global_localize_ackerman_message.timestamp;

	// PARE AQUI !!!!!
	// TRATAR TRUEPOS !!
	do
	{
		carmen_ipc_sleep(how_long);
	}
	while (global_localize_ackerman_message.timestamp == time_previous_globalpos);

	carmen_navigator_ackerman_go();
	process_map_message(&global_obstacle_distance_mapper_compact_map_message);

	carmen_mapper_copy_map_from_message(&(simulator_config.map), &global_mapper_map_message);
	// carmen_map_server_copy_offline_map_from_message(&(simulator_config.map), &global_offline_map_message);
}


int
obstacle_distance_map_is_invalid(carmen_obstacle_distance_mapper_compact_map_message *map, double pos_x, double pos_y)
{
	if (map->timestamp == 0 ||
		map->config.x_origin == 0 ||
		map->config.y_origin == 0 ||
		map->config.x_size == 0 ||
		map->config.y_size == 0 ||
		map->config.x_origin > pos_x ||
		map->config.y_origin > pos_y ||
		pos_x > map->config.x_origin + map->config.x_size ||
		pos_y > map->config.y_origin + map->config.y_size)
		return 1;

	//printf("obstacle distance mapper invalid\n");
	return 0;
}


int
map_is_invalid(carmen_mapper_map_message *map, double pos_x, double pos_y)
{
	if (map->timestamp == 0 ||
		map->config.x_origin == 0 ||
		map->config.y_origin == 0 ||
		map->config.x_size == 0 ||
		map->config.y_size == 0 ||
		map->config.x_origin > pos_x ||
		map->config.y_origin > pos_y ||
		pos_x > map->config.x_origin + map->config.x_size ||
		pos_y > map->config.y_origin + map->config.y_size)
	{
		return 1;
	}

	//printf("mapper map invalid\n");
	return 0;
}


int
pose_is_invalid(carmen_localize_ackerman_globalpos_message *msg, double pos_x, double pos_y)
{
	if (msg->timestamp == 0 ||
		fabs(msg->globalpos.x - pos_x) > 5.0 ||
		fabs(msg->globalpos.y != pos_y) > 5.0)
		return 1;

	return 0;
}


int
laser_reading_is_invalid(carmen_laser_laser_message *laser_message)
{
	if (laser_message->timestamp == 0 ||
		laser_message->num_readings == 0)
		return 1;

	return 0;
}


int
goal_list_is_invalid(carmen_behavior_selector_goal_list_message *goal_list_message, double px, double py, double pth)
{
	if (goal_list_message->timestamp == 0 || goal_list_message->size == 0)
		return 1;

	g2o::SE2 pose(0., 0., pth);
	g2o::SE2 goal(goal_list_message->goal_list[0].x - px,
			goal_list_message->goal_list[0].y - py,
			goal_list_message->goal_list[0].theta);

	goal = pose.inverse() * goal;

	// if the goal is behind the car, or more than 100m ahead or to the side, we consider
	// the message invalid.
	if (goal[0] < 0 || goal[0] > 100.0 || fabs(goal[1]) > 100.0)
		return 1;

	//printf("goal list invalid\n");
	return 0;
}


int
rddf_is_invalid(carmen_rddf_road_profile_message *rddf_message, double x, double y, double th)
{
	if (rddf_message->timestamp == 0 || rddf_message->number_of_poses == 0)
		return 1;

	g2o::SE2 pose(0., 0., th);
	g2o::SE2 goal(rddf_message->poses[0].x - x,
				  rddf_message->poses[0].y - y,
				  rddf_message->poses[0].theta);

	goal = pose.inverse() * goal;

	// if the goal is behind the car, or more than 100m ahead or to the side, we consider
	// the message invalid.
	if (goal[0] < 0 || goal[0] > 30.0 || fabs(goal[1]) > 30.0)
		return 1;

	//printf("rddf invalid\n");
	return 0;
}


// TODO: Merge this function with the reset_initial_pose.
void
reset_without_initial_pose()
{
	double time_last_message = carmen_get_time();

	double x = 0.;
	double y = 0.;
	double th = 0.;

	do
	{
		publish_stop_command();
		carmen_ipc_sleep(0.1);

		// If the messages are taking too long to arrive, warn the user.
		double curr_time = carmen_get_time();
		if (curr_time - time_last_message > 1.0)
		{
			printf("Waiting for initialization messages.\n");
			time_last_message = curr_time;
		}

		x = global_localize_ackerman_message.globalpos.x;
		y = global_localize_ackerman_message.globalpos.y;
		th = global_localize_ackerman_message.globalpos.theta;

	} while (global_localize_ackerman_message.timestamp == 0 ||
			obstacle_distance_map_is_invalid(&global_obstacle_distance_mapper_compact_map_message, x, y) ||
			// laser_reading_is_invalid(&global_front_laser_message) ||
			map_is_invalid(&global_mapper_map_message, x, y) ||
			goal_list_is_invalid(&global_goal_list_message, x, y, th) ||
			rddf_is_invalid(&global_rddf_message, x, y, th));
}


void
reset_initial_pose(double x, double y, double th)
{
	publish_stop_command();

	carmen_point_t pose;
	carmen_point_t std;

	pose.x = x;
	pose.y = y;
	pose.theta = th;

	std.x = 0.001;
	std.y = 0.001;
	std.theta = carmen_degrees_to_radians(0.01);

	double time_last_message = carmen_get_time();

	do
	{
		publish_stop_command();
		carmen_localize_ackerman_initialize_gaussian_command(pose, std);
		carmen_ipc_sleep(0.5);

		// If the messages are taking too long to arrive, warn the user.
		double curr_time = carmen_get_time();
		if (curr_time - time_last_message > 1.0)
		{
			printf("Waiting for valid messages after reseting initial pose.\n");
			time_last_message = curr_time;
		}

	} while (pose_is_invalid(&global_localize_ackerman_message, x, y) ||
			obstacle_distance_map_is_invalid(&global_obstacle_distance_mapper_compact_map_message, x, y) ||
			laser_reading_is_invalid(&global_front_laser_message) ||
			map_is_invalid(&global_mapper_map_message, x, y)); // ||
			//goal_list_is_invalid(&global_goal_list_message, x, y, th) ||
			//rddf_is_invalid(&global_rddf_message, x, y, th));
}


std::vector<double>
read_pose()
{
	std::vector<double> state;

	carmen_localize_ackerman_globalpos_message globalpos = global_localize_ackerman_message;

	state.push_back(globalpos.globalpos.x);
	state.push_back(globalpos.globalpos.y);
	state.push_back(globalpos.globalpos.theta);
	state.push_back(globalpos.v);
	state.push_back(globalpos.phi);

	return state;
}


std::vector<double>
read_truepos()
{
	std::vector<double> state;

	carmen_simulator_ackerman_truepos_message truepos = global_truepos_message;

	state.push_back(truepos.truepose.x);
	state.push_back(truepos.truepose.y);
	state.push_back(truepos.truepose.theta);
	state.push_back(truepos.v);
	state.push_back(truepos.phi);

	return state;
}


std::vector<double>
read_goal()
{
	double x = global_localize_ackerman_message.globalpos.x;
	double y = global_localize_ackerman_message.globalpos.y;
	double th = global_localize_ackerman_message.globalpos.theta;

	return next_goal_from_list(x, y, th, &global_goal_list_message);
}


std::vector<double>
read_laser()
{
	std::vector<double> ranges = laser_to_vec(global_front_laser_message,
		global_localize_ackerman_message.globalpos.theta, VIEW_LASER);

	if (simulator_config.use_rear_laser)
	{
		std::vector<double> rear_ranges = laser_to_vec(global_rear_laser_message,
			global_localize_ackerman_message.globalpos.theta, VIEW_LASER);

		// append rear_ranges to the end of ranges
		ranges.insert(ranges.end(), rear_ranges.begin(), rear_ranges.end());
	}

	return ranges;
}


std::vector<double>
read_rddf()
{
	std::vector<double> v;

	for (int i = 0; i < 100; i++)
	{
		if (i < global_rddf_message.number_of_poses)
		{
			v.push_back(global_rddf_message.poses[i].x);
			v.push_back(global_rddf_message.poses[i].y);
			v.push_back(global_rddf_message.poses[i].theta);
		}
		else
		{
			v.push_back(0.);
			v.push_back(0.);
			v.push_back(0.);
		}
	}

	return v;
}


std::vector<double>
read_commands()
{
	std::vector<double> commands;

	for (int i = 0; i < global_motion_command_message.num_motion_commands; i++)
	{
		commands.push_back(global_motion_command_message.motion_command[i].v);
		commands.push_back(global_motion_command_message.motion_command[i].phi);
		commands.push_back(global_motion_command_message.motion_command[i].time);
	}

	return commands;
}


int
hit_obstacle()
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

	return (hit == 1);
}


void
simulation_destroy()
{
	free(global_simulated_front_laser.range);
}


void
destroy()
{
	if (!global_destroy_already_requested)
	{
		publish_stop_command();
		simulation_destroy();
		global_destroy_already_requested = 1;
		carmen_ipc_disconnect();
	}

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

	err = IPC_defineMsg(CARMEN_NAVIGATOR_ACKERMAN_PLAN_TO_DRAW_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_NAVIGATOR_ACKERMAN_PLAN_TO_DRAW_FMT);

	carmen_test_ipc_exit(err, "Could not define", CARMEN_NAVIGATOR_ACKERMAN_PLAN_TO_DRAW_NAME);
}


void
signal_handler(int signo)
{
	if (signo == SIGINT)
		destroy();
}


int
read_robot_ackerman_parameters(int argc, char **argv)
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


static void
read_simulator_parameters(int argc, char *argv[], carmen_simulator_ackerman_config_t *config)
{
	int num_items;

	carmen_param_t param_list[]=
	{
			{"simulator", "time", CARMEN_PARAM_DOUBLE, &(config->real_time), 1, NULL},
			{"simulator", "sync_mode", CARMEN_PARAM_ONOFF, &(config->sync_mode), 1, NULL},
			{"simulator", "motion_timeout", CARMEN_PARAM_DOUBLE, &(config->motion_timeout),1, NULL},
			{"robot", "frontlaser_use", CARMEN_PARAM_ONOFF, &(config->use_front_laser), 1, NULL},
			{"robot", "frontlaser_id", CARMEN_PARAM_INT, &(config->front_laser_config.id), 0, NULL},
			{"robot", "rearlaser_use", CARMEN_PARAM_ONOFF, &(config->use_rear_laser), 1, NULL},
			{"robot", "rearlaser_id", CARMEN_PARAM_INT, &(config->rear_laser_config.id), 0, NULL},
			{"robot", "width", CARMEN_PARAM_DOUBLE, &(config->width), 1, NULL},
			{"robot", "length", CARMEN_PARAM_DOUBLE, &(config->length), 1, NULL},
			{"robot", "distance_between_rear_wheels", CARMEN_PARAM_DOUBLE, &(config->distance_between_rear_wheels), 1, NULL},
			{"robot", "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &(config->distance_between_front_and_rear_axles), 1, NULL},
			{"robot", "max_velocity", CARMEN_PARAM_DOUBLE, &(config->max_v), 1,NULL},
			{"robot", "max_steering_angle", CARMEN_PARAM_DOUBLE, &(config->max_phi), 1, NULL},
			{"robot", "maximum_steering_command_rate", CARMEN_PARAM_DOUBLE, &(config->maximum_steering_command_rate), 0, NULL},
			{"robot", "understeer_coeficient", CARMEN_PARAM_DOUBLE, &(config->understeer_coeficient), 0, NULL},
			{"robot", "understeer_coeficient2", CARMEN_PARAM_DOUBLE, &(config->understeer_coeficient2), 0, NULL},
			{"robot", "maximum_speed_forward", CARMEN_PARAM_DOUBLE, &(config->maximum_speed_forward), 0, NULL},
			{"robot", "maximum_speed_reverse", CARMEN_PARAM_DOUBLE, &(config->maximum_speed_reverse), 0, NULL},
			{"robot", "maximum_acceleration_forward", CARMEN_PARAM_DOUBLE, &(config->maximum_acceleration_forward), 0, NULL},
			{"robot", "maximum_deceleration_forward", CARMEN_PARAM_DOUBLE, &(config->maximum_deceleration_forward), 0, NULL},
			{"robot", "maximum_acceleration_reverse", CARMEN_PARAM_DOUBLE, &(config->maximum_acceleration_reverse), 0, NULL},
			{"robot", "maximum_deceleration_reverse", CARMEN_PARAM_DOUBLE, &(config->maximum_deceleration_reverse), 0, NULL},
			{"robot", "distance_between_rear_car_and_rear_wheels", CARMEN_PARAM_DOUBLE, &(config->distance_between_rear_car_and_rear_wheels), 0, NULL},
			{"rrt",   "use_mpc",                    CARMEN_PARAM_ONOFF, &(config->use_mpc), 0, NULL},
			{"rrt",   "use_rlpid",                  CARMEN_PARAM_ONOFF, &(config->use_rlpid), 0, NULL},
	};


	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	static char frontlaser_fov_string[256];
	static char frontlaser_res_string[256];
	static char rearlaser_fov_string[256];
	static char rearlaser_res_string[256];

	sprintf(frontlaser_fov_string, "laser%d_fov", config->front_laser_config.id);
	sprintf(frontlaser_res_string, "laser%d_resolution", config->front_laser_config.id);

	sprintf(rearlaser_fov_string, "laser%d_fov", config->rear_laser_config.id);
	sprintf(rearlaser_res_string, "laser%d_resolution", config->rear_laser_config.id);

	carmen_param_t param_list_front_laser[] =
	{
			{"simulator", "frontlaser_maxrange", CARMEN_PARAM_DOUBLE, &(config->front_laser_config.max_range), 1, NULL},
			{"robot", "frontlaser_offset", CARMEN_PARAM_DOUBLE, &(config->front_laser_config.offset), 1, NULL},
			{"robot", "frontlaser_side_offset", CARMEN_PARAM_DOUBLE, &(config->front_laser_config.side_offset), 1, NULL},
			{"robot", "frontlaser_angular_offset", CARMEN_PARAM_DOUBLE, &(config->front_laser_config.angular_offset), 1, NULL},
			{"laser", frontlaser_fov_string, CARMEN_PARAM_DOUBLE, &(config->front_laser_config.fov), 0, NULL},
			{"laser", frontlaser_res_string, CARMEN_PARAM_DOUBLE, &(config->front_laser_config.angular_resolution), 0, NULL},
			{"simulator", "laser_probability_of_random_max", CARMEN_PARAM_DOUBLE, &(config->front_laser_config.prob_of_random_max), 1, NULL},
			{"simulator", "laser_probability_of_random_reading", CARMEN_PARAM_DOUBLE, &(config->front_laser_config.prob_of_random_reading), 1, NULL},
			{"simulator", "laser_sensor_variance", CARMEN_PARAM_DOUBLE, &(config->front_laser_config.variance), 1, NULL}
	};

	carmen_param_t param_list_rear_laser[] =
	{
			{"simulator", "rearlaser_maxrange", CARMEN_PARAM_DOUBLE, &(config->rear_laser_config.max_range), 1, NULL},
			{"robot", "rearlaser_offset", CARMEN_PARAM_DOUBLE, &(config->rear_laser_config.offset), 1, NULL},
			{"robot", "rearlaser_side_offset", CARMEN_PARAM_DOUBLE, &(config->rear_laser_config.side_offset), 1, NULL},
			{"robot", "rearlaser_angular_offset", CARMEN_PARAM_DOUBLE, &(config->rear_laser_config.angular_offset), 1, NULL},
			{"laser", rearlaser_fov_string, CARMEN_PARAM_DOUBLE, &(config->rear_laser_config.fov), 0, NULL},
			{"laser", rearlaser_res_string, CARMEN_PARAM_DOUBLE, &(config->rear_laser_config.angular_resolution), 0, NULL},
			{"simulator", "laser_probability_of_random_max", CARMEN_PARAM_DOUBLE, &(config->rear_laser_config.prob_of_random_max), 1, NULL},
			{"simulator", "laser_probability_of_random_reading", CARMEN_PARAM_DOUBLE, &(config->rear_laser_config.prob_of_random_reading), 1, NULL},
			{"simulator", "laser_sensor_variance", CARMEN_PARAM_DOUBLE, &(config->rear_laser_config.variance), 1, NULL}
	};

	if (config->use_front_laser)
	{
		num_items = sizeof(param_list_front_laser)/
				sizeof(param_list_front_laser[0]);
		carmen_param_install_params(argc, argv, param_list_front_laser,
				num_items);
		config->front_laser_config.angular_resolution =
				carmen_degrees_to_radians(config->front_laser_config.angular_resolution);

		config->front_laser_config.fov =
				carmen_degrees_to_radians(config->front_laser_config.fov);
	}

	if (config->use_rear_laser)
	{
		num_items = sizeof(param_list_rear_laser)/
				sizeof(param_list_rear_laser[0]);
		carmen_param_install_params(argc, argv, param_list_rear_laser,
				num_items);
		config->rear_laser_config.angular_resolution =
				carmen_degrees_to_radians(config->rear_laser_config.angular_resolution);

		config->rear_laser_config.fov =
				carmen_degrees_to_radians(config->rear_laser_config.fov);
	}

	fill_laser_config_data( &(config->front_laser_config) );

	if(config->use_rear_laser)
		fill_laser_config_data( &(config->rear_laser_config));


	// TODO REMOVER ESTA PARTE E LER OS PARAMETROS DIRETAMENTE PARA O ROBO_CONFIG
	config->robot_config.maximum_steering_command_rate = config->maximum_steering_command_rate;
	config->robot_config.understeer_coeficient = config->understeer_coeficient;
	config->robot_config.maximum_acceleration_forward = config->maximum_acceleration_forward;
	config->robot_config.maximum_deceleration_forward = config->maximum_deceleration_forward;
	config->robot_config.maximum_acceleration_reverse = config->maximum_acceleration_reverse;
	config->robot_config.maximum_deceleration_reverse = config->maximum_deceleration_reverse;
	config->robot_config.distance_between_rear_car_and_rear_wheels = config->distance_between_rear_car_and_rear_wheels;
	config->robot_config.distance_between_front_and_rear_axles = config->distance_between_front_and_rear_axles;
	config->robot_config.max_phi = config->max_phi;
	config->robot_config.length = config->length;
}


void
initialize_global_data()
{
	global_simulated_front_laser.host = carmen_get_host();
	global_simulated_front_laser.num_readings = simulator_config.front_laser_config.num_lasers;

	global_simulated_front_laser.range = (double *) calloc (simulator_config.front_laser_config.num_lasers, sizeof(double));
	carmen_test_alloc(global_simulated_front_laser.range);

	global_simulated_front_laser.num_remissions = 0;
	global_simulated_front_laser.remission = 0;

	memset(&global_obstacle_distance_mapper_compact_map_message, 0, sizeof(global_obstacle_distance_mapper_compact_map_message));
	memset(&global_localize_ackerman_message, 0, sizeof(global_localize_ackerman_message));
	memset(&global_front_laser_message, 0, sizeof(global_front_laser_message));
	memset(&global_rear_laser_message, 0, sizeof(global_rear_laser_message));
	memset(&global_mapper_map_message, 0, sizeof(global_mapper_map_message));
	memset(&global_motion_command_message, 0, sizeof(global_motion_command_message));
	memset(&global_truepos_message, 0, sizeof(global_truepos_message));
}


void
go_message_handler()
{
	autonomous_mode = 1;
}


void
stop_message_handler()
{
	autonomous_mode = 0;
}


void
init()
{
	int argc = 1;
	char *argv[] = {"rl_motion_planner"};
	carmen_ipc_initialize(1, argv);

	read_robot_ackerman_parameters(argc, argv);
	read_simulator_parameters(argc, argv, &simulator_config);
	carmen_libpid_read_PID_parameters(argc, argv);

	initialize_global_data();
	define_messages();

    carmen_localize_ackerman_subscribe_globalpos_message(&global_localize_ackerman_message,
    	NULL, CARMEN_SUBSCRIBE_LATEST);

	carmen_obstacle_distance_mapper_subscribe_compact_map_message(&global_obstacle_distance_mapper_compact_map_message,
		NULL, CARMEN_SUBSCRIBE_LATEST);

	carmen_behavior_selector_subscribe_goal_list_message(&global_goal_list_message,
		NULL, CARMEN_SUBSCRIBE_LATEST);

	carmen_laser_subscribe_frontlaser_message(&global_front_laser_message,
		NULL, CARMEN_SUBSCRIBE_LATEST);

	carmen_laser_subscribe_rearlaser_message(&global_rear_laser_message,
		NULL, CARMEN_SUBSCRIBE_LATEST);

//	carmen_map_server_subscribe_offline_map(&global_offline_map_message,
//		NULL, CARMEN_SUBSCRIBE_LATEST);

	carmen_mapper_subscribe_map_message(&global_mapper_map_message,
		NULL, CARMEN_SUBSCRIBE_LATEST);

	carmen_rddf_subscribe_road_profile_message(&global_rddf_message,
		NULL, CARMEN_SUBSCRIBE_LATEST);

	carmen_robot_ackerman_subscribe_teacher_motion_command(&global_motion_command_message,
		NULL, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *) CARMEN_NAVIGATOR_ACKERMAN_GO_NAME,
			(char *) CARMEN_DEFAULT_MESSAGE_FMT,
			NULL, sizeof(carmen_navigator_ackerman_go_message),
			(carmen_handler_t) go_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(
			(char *) CARMEN_NAVIGATOR_ACKERMAN_STOP_NAME,
			(char *) CARMEN_DEFAULT_MESSAGE_FMT,
			NULL, sizeof(carmen_navigator_ackerman_stop_message),
			(carmen_handler_t) stop_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_simulator_ackerman_subscribe_truepos_message(&global_truepos_message,
		NULL, CARMEN_SUBSCRIBE_LATEST);

	signal(SIGINT, signal_handler);
}


void
simulation_reset(double p_x, double p_y, double p_th, double p_v, double p_phi)
{
	carmen_simulator_ackerman_config_t *simulator_config_ptr = &simulator_config;

	simulator_config_ptr->true_pose.x = p_x;
	simulator_config_ptr->true_pose.y = p_y;
	simulator_config_ptr->true_pose.theta = p_th;
	simulator_config_ptr->odom_pose.theta = p_th;

	simulator_config_ptr->v = p_v;
	simulator_config_ptr->phi = p_phi;
	simulator_config_ptr->target_v = p_v;
	simulator_config_ptr->target_phi = p_phi;

	simulator_config_ptr->current_motion_command_vector = NULL;
	simulator_config_ptr->nun_motion_commands = 0;
	simulator_config_ptr->current_motion_command_vector_index = 0;
	simulator_config_ptr->initialize_neural_networks = 1;
}


void
simulation_step(double v, double phi, double delta_t)
{
	carmen_simulator_ackerman_config_t *simulator_config_ptr = &simulator_config;
	double base_time = carmen_get_time();

	carmen_ackerman_motion_command_t cmd;

	cmd.x = cmd.y = cmd.theta = 0.0;
	cmd.v = v;
	cmd.phi = phi;
	cmd.time = base_time + 0.1;

	simulator_config_ptr->current_motion_command_vector = &cmd;
	simulator_config_ptr->nun_motion_commands = 1;
	simulator_config_ptr->time_of_last_command = base_time;

	if (simulator_config_ptr->use_mpc)
		simulator_config_ptr->nun_motion_commands = apply_system_latencies(simulator_config_ptr->current_motion_command_vector, simulator_config_ptr->nun_motion_commands);

	simulator_config_ptr->delta_t = delta_t;

	/*
	if (!simulator_config_ptr->sync_mode)
	{
		delta_time = timestamp - simulator_config_ptr->time_of_last_command;
		if ((simulator_config_ptr->v > 0 || simulator_config_ptr->phi > 0) && (delta_time > simulator_config_ptr->motion_timeout))
		{
			simulator_config_ptr->current_motion_command_vector = NULL;
			simulator_config_ptr->nun_motion_commands = 0;
			simulator_config_ptr->current_motion_command_vector_index = 0;
			simulator_config_ptr->target_v = 0;
			simulator_config_ptr->target_phi = 0;
		}
	}
	*/

	/**
	 * *************************************************************************************************
	 * IMPORTANT: The list of commands used as input to the neural simulator is maintained internally,
	 * and it seems that there isn't an easy way of reinitializing it to some command list besides to
	 * zero. It seems that the simulator was designed to be reinitialized to zero once in the beginning
	 * and then used without being reinitialized until the end of execution.
	 * Because of that, I'm just doing a simple Ackerman prediction here, and ignoring latency.
	 * *************************************************************************************************
	 */
	// carmen_simulator_ackerman_recalc_pos(simulator_config_ptr);
	simulator_config_ptr->v = cmd.v;
	simulator_config_ptr->phi = cmd.phi;
	// simulator_config_ptr->true_pose.x +=  simulator_config_ptr->v * simulator_config_ptr->delta_t * cos(simulator_config_ptr->true_pose.theta);
	// simulator_config_ptr->true_pose.y +=  simulator_config_ptr->v * simulator_config_ptr->delta_t * sin(simulator_config_ptr->true_pose.theta);
	// simulator_config_ptr->true_pose.theta += simulator_config_ptr->v * simulator_config_ptr->delta_t * tan(simulator_config_ptr->phi) / simulator_config_ptr->distance_between_front_and_rear_axles;
	// simulator_config_ptr->true_pose.theta = carmen_normalize_theta(simulator_config_ptr->true_pose.theta);

	ackerman_motion_model(&(simulator_config_ptr->true_pose.x),
		&(simulator_config_ptr->true_pose.y),
		&(simulator_config_ptr->true_pose.theta),
		simulator_config_ptr->v,
		simulator_config_ptr->phi,
		simulator_config_ptr->delta_t,
		simulator_config_ptr->distance_between_front_and_rear_axles);

	// carmen_simulator_ackerman_update_objects(simulator_config_ptr);
	carmen_simulator_ackerman_calc_laser_msg(&global_simulated_front_laser,
		simulator_config_ptr, 0);
}


int
simulation_hit_obstacle()
{
	if (hit_something_in_the_map(&simulator_config, simulator_config.true_pose))
		return 1;

	return 0;
}


std::vector<double>
simulation_read_laser()
{
	return laser_to_vec(global_simulated_front_laser,
		simulator_config.true_pose.theta, VIEW_SIMULATED_LASER);
}


std::vector<double>
simulation_read_goal()
{
	double x = simulator_config.true_pose.x;
	double y = simulator_config.true_pose.y;
	double th = simulator_config.true_pose.theta;

	return next_goal_from_list(x, y, th, &global_goal_list_message);
}


std::vector<double>
simulation_read_pose()
{
	std::vector<double> pose;

	pose.push_back(simulator_config.true_pose.x);
	pose.push_back(simulator_config.true_pose.y);
	pose.push_back(simulator_config.true_pose.theta);
	pose.push_back(simulator_config.v);
	pose.push_back(simulator_config.phi);

	return pose;
}

