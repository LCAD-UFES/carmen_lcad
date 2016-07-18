/*
 * global_state.cpp
 *
 *  Created on: 07/03/2012
 *      Author: romulo
 */

#include "model/global_state.h"
#include "util.h"
#include "trajectory_lookup_table.h"

Pose *GlobalState::localizer_pose = 0;
Pose *GlobalState::last_plan_pose = 0;
double GlobalState::localizer_pose_timestamp = 0;
double GlobalState::rrt_planner_timestamp = 0;
double GlobalState::last_rrt_path_message_timestamp = 0;

Command GlobalState::last_odometry;

Robot_State GlobalState::initial_robot_state;

int GlobalState::show_debug_info;
Pose *GlobalState::goal_pose  = NULL;
bool GlobalState::last_goal = true;

bool GlobalState::last_path_received_is_empty = false;

carmen_robot_ackerman_config_t GlobalState::robot_config;
double GlobalState::param_max_vel = 0.0;

double GlobalState::max_phi_velocity = 1.0 * 0.48;		// Equivalente a rodar o volante todo para um lado (27.7 graus = 0.48 radianos) em 1 segundo.
														// A velocidade de mudanca de phi nao pode ultrapassar este valor
double GlobalState::max_phi_acceleration = 0.48 / (0.2 * 0.2); // Alcanca a velocidade maxima em 0.2 segundos (s = at²; a = s/t²).
														// A velocidade de phi pode aumentar no maximo deste valor por segundo
double GlobalState::time_to_change_gears = 1.0;

carmen_map_t GlobalState::cost_map;
bgi::rtree< occupied_cell, bgi::quadratic<16> > GlobalState::obstacles_rtree;

carmen_grid_mapping_distance_map_message *GlobalState::localize_map = NULL;
KDTree2D GlobalState::obstacles_kdtree;
vector<vector<cell_coords_t>> GlobalState::cell_mask;

bool GlobalState::cost_map_initialized 	= false;

bool GlobalState::use_obstacle_avoider = true;

int    GlobalState::cheat 				= 0;
bool   GlobalState::following_path		= false;

double GlobalState::obstacle_threshold	= 0.5;

bool GlobalState::ford_escape_online = false;
carmen_ford_escape_status_message GlobalState::ford_escape_status;

int GlobalState::current_algorithm = CARMEN_BEHAVIOR_SELECTOR_RRT;
int GlobalState::behavior_selector_state = BEHAVIOR_SELECTOR_FOLLOWING_LANE;

int GlobalState::publish_tree = 1;
int GlobalState::reuse_last_path = 0;

double GlobalState::obstacle_cost_distance = 1.5; // distancia para zero custo (os custos sao lineares com a distancia para obstaculos)

RRT_Node *GlobalState::goal_node = NULL;

void GlobalState::set_goal_pose(Pose goal_pose)
{
	if (!GlobalState::goal_pose)
	{
		GlobalState::goal_pose = new Pose();
	}

	*GlobalState::goal_pose = goal_pose;
}

void GlobalState::set_robot_pose(Pose robot_pose, double timestamp)
{
	if (!GlobalState::localizer_pose)
	{
		GlobalState::localizer_pose = new Pose();
	}

	*GlobalState::localizer_pose = robot_pose;
	localizer_pose_timestamp = timestamp;
}


Robot_State
GlobalState::estimate_initial_robot_state()
{
	GlobalState::rrt_planner_timestamp = carmen_get_time();
	// Para testar tocando log
	// time_elapsed_since_last_localizer_pose = 0.1;
	double time_elapsed_since_last_localizer_pose = GlobalState::rrt_planner_timestamp - GlobalState::localizer_pose_timestamp;

	Robot_State initial_robot_pose;
	initial_robot_pose.pose = *GlobalState::localizer_pose;
	initial_robot_pose.v_and_phi = GlobalState::last_odometry;

	initial_robot_pose = TrajectoryLookupTable::predict_next_pose(initial_robot_pose,
			initial_robot_pose.v_and_phi, time_elapsed_since_last_localizer_pose, NULL, 0.01);

	return initial_robot_pose;
}
