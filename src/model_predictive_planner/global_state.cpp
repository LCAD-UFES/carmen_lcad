/*
 * global_state.cpp
 *
 *  Created on: 07/03/2012
 *      Author: romulo
 */

#include "model/global_state.h"
#include "util.h"

carmen_robot_and_trailers_pose_t *GlobalState::localizer_pose = NULL;
carmen_robot_and_trailers_pose_t *GlobalState::last_plan_pose = NULL;
double GlobalState::localizer_pose_timestamp = 0;

Command GlobalState::last_odometry;

Pose *GlobalState::goal_pose  = NULL;
bool GlobalState::last_goal = true;

bool GlobalState::last_path_received_is_empty = false;

carmen_robot_ackerman_config_t GlobalState::robot_config;
carmen_semi_trailers_config_t GlobalState::semi_trailer_config;

double GlobalState::robot_max_centripetal_acceleration = 0.0;

double GlobalState::param_max_vel = 0.0;

double GlobalState::max_phi_velocity = 1.0 * 0.48;		// Equivalente a rodar o volante todo para um lado (27.7 graus = 0.48 radianos) em 1 segundo.
														// A velocidade de mudanca de phi nao pode ultrapassar este valor
// Robot delay
int GlobalState::eliminate_path_follower = 1;
double GlobalState::eliminate_path_follower_transition_v = 4.16666;
double GlobalState::robot_velocity_delay = 0.46;
double GlobalState::robot_min_v_distance_ahead = 0.10;
double GlobalState::robot_steering_delay = 0.20;
double GlobalState::robot_min_s_distance_ahead = 0.10;

// Optimization weights
double GlobalState::w1 = 30.0  ; // end_of_path_to_goal_distance
double GlobalState::w2 = 15.0  ; // end_of_path_to_goal_angular_distance
double GlobalState::w3 = 15.0  ; // end_of_path_to_goal_delta_theta
double GlobalState::w4 = 3.0   ; // path_to_lane_distance
double GlobalState::w5 = 20.0  ; // proximity_to_obstacles
double GlobalState::w6 = 0.0025; // traveled_distance
double GlobalState::w7 = 0.0; // look_ahead_based_on_stanley_method
double GlobalState::look_ahead_horizon = 5.0; //fraction of the path (in meters it is about N*0.10m) based_on_stanley_method

double GlobalState::time_to_change_gears = 1.0;

carmen_map_t GlobalState::cost_map;

carmen_obstacle_distance_mapper_map_message *GlobalState::distance_map = NULL;

bool GlobalState::cost_map_initialized 	= false;

bool GlobalState::use_obstacle_avoider = true;
bool GlobalState::use_mpc = false;

carmen_moving_objects_point_clouds_message *GlobalState::objects_message = NULL;
bool GlobalState::moving_objects_initialized = false;
std::vector<carmen_robot_and_trailers_traj_point_t*> GlobalState::moving_objects_trajectories;

int  GlobalState::use_truepos = 0;
bool GlobalState::following_path = false;
bool GlobalState::path_has_collision_or_phi_exceeded = false;

int GlobalState::reverse_driving_flag = 0;
int GlobalState::reverse_planning = 0;
double GlobalState::distance_between_waypoints = 0.0;

double GlobalState::param_max_vel_reverse = 1.0;
double GlobalState::param_parking_speed_limit = 1.0;

double GlobalState::obstacle_threshold	= 0.5;

bool GlobalState::ford_escape_online = false;
carmen_ford_escape_status_message GlobalState::ford_escape_status;

int GlobalState::current_algorithm = CARMEN_BEHAVIOR_SELECTOR_FRENET;
int GlobalState::behavior_selector_task = BEHAVIOR_SELECTOR_FOLLOW_ROUTE;
int GlobalState::behavior_selector_low_level_state = Initializing;
int GlobalState::route_planner_state = IDLE;
int GlobalState::offroad_planner_request = NO_REQUEST;

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

