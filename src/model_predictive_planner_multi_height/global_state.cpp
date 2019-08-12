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

Command GlobalState::last_odometry;

Pose *GlobalState::goal_pose  = NULL;
bool GlobalState::last_goal = true;

bool GlobalState::last_path_received_is_empty = false;

carmen_robot_ackerman_config_t GlobalState::robot_config;

double GlobalState::robot_max_centripetal_acceleration = 0.0;

double GlobalState::param_max_vel = 0.0;

double GlobalState::max_phi_velocity = 1.0 * 0.48;		// Equivalente a rodar o volante todo para um lado (27.7 graus = 0.48 radianos) em 1 segundo.
														// A velocidade de mudanca de phi nao pode ultrapassar este valor
double GlobalState::time_to_change_gears = 1.0;

carmen_map_t GlobalState::cost_map;

carmen_obstacle_distance_mapper_map_message *GlobalState::distance_map = NULL;
carmen_obstacle_distance_mapper_map_message *GlobalState::distance_map_level1 = NULL;

bool GlobalState::cost_map_initialized 	= false;

bool GlobalState::use_obstacle_avoider = true;
bool GlobalState::use_mpc = false;

carmen_moving_objects_point_clouds_message *GlobalState::objects_message = NULL;
bool GlobalState::moving_objects_initialized = false;
std::vector<carmen_ackerman_traj_point_t*> GlobalState::moving_objects_trajectories;

int    GlobalState::use_truepos 				= 0;
bool   GlobalState::following_path		= false;

double GlobalState::obstacle_threshold	= 0.5;

bool GlobalState::ford_escape_online = false;
carmen_ford_escape_status_message GlobalState::ford_escape_status;

int GlobalState::current_algorithm = CARMEN_BEHAVIOR_SELECTOR_RRT;
int GlobalState::behavior_selector_state = BEHAVIOR_SELECTOR_FOLLOWING_LANE;
int GlobalState::behavior_selector_low_level_state = Initializing;

int GlobalState::publish_tree = 1;
int GlobalState::reuse_last_path = 0;

int GlobalState::use_path_planner = 0;
int GlobalState::use_tracker_goal_and_lane = 0;

double GlobalState::obstacle_cost_distance = 1.5; // distancia para zero custo (os custos sao lineares com a distancia para obstaculos)

double GlobalState::max_square_distance_to_lane = 2.0;

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
