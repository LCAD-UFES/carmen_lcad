/*
 * global_state.cpp
 *
 *  Created on: 07/03/2012
 *      Author: romulo
 */

#include "global_state.h"
#include "../util/ackerman.h"

Pose *GlobalState::localize_pose = 0;
double GlobalState::localizer_pose_timestamp = 0;
double GlobalState::rrt_planner_timestamp = 0;
double GlobalState::last_rrt_path_message_timestamp = 0;

Command GlobalState::last_odometry;

Robot_State GlobalState::initial_robot_state;

int GlobalState::show_debug_info;
Pose *GlobalState::goal_pose  = 0;
bool GlobalState::goal_just_set = false;
bool GlobalState::last_goal = true;

bool GlobalState::last_path_received_is_empty = false;

Robot_Config GlobalState::robot_config;
carmen_semi_trailer_config_t GlobalState::semi_trailer_config;

double GlobalState::param_max_vel = 0.0;

double GlobalState::max_phi_velocity = 1.0 * 0.48;		// Equivalente a rodar o volante todo para um lado (27.7 graus = 0.48 radianos) em 1 segundo.
														// A velocidade de mudanca de phi nao pode ultrapassar este valor
double GlobalState::max_phi_acceleration = 0.48 / (0.2 * 0.2); // Alcanca a velocidade maxima em 0.2 segundos (s = at²; a = s/t²).
														// A velocidade de phi pode aumentar no maximo deste valor por segundo
double GlobalState::time_to_change_gears = 1.0;

carmen_map_t GlobalState::cost_map;
Gradient_Cost_Map GlobalState::utility_map;
carmen_map_t GlobalState::lane_map = {{0, 0, 0, "", NULL, 0, 0}, NULL, NULL};
char *GlobalState::rddf_path = (char *)"../data/rndf/rddf-log_voltadaufes-20121003-01-novo-from-log.kml";
vector<carmen_point_t> GlobalState::lane_points;
vector<Pose> GlobalState::lane_points_on_map;

bool GlobalState::cost_map_initialized 	= false;

bool GlobalState::gradient_cost_map_old = false;

int    GlobalState::use_truepos 		= 0;
bool   GlobalState::following_path		= false;

double GlobalState::distance_interval	= 3.5;
double GlobalState::param_distance_interval	= 3.5;

double GlobalState::plan_time 			= 0.5;
double GlobalState::timeout				= 5.0;

double GlobalState::obstacle_threshold	= 0.5;

int GlobalState::current_algorithm = CARMEN_BEHAVIOR_SELECTOR_RRT;
int GlobalState::behavior_selector_task = BEHAVIOR_SELECTOR_FOLLOW_ROUTE;

int GlobalState::publish_tree = 1;
int GlobalState::publish_lane_map = 0;
int GlobalState::reuse_last_path = 0;

double GlobalState::obstacle_cost_distance = 1.5; // distancia para zero custo (os custos sao lineares com a distancia para obstaculos)

RRT_Node *GlobalState::goal_node = NULL;

int GlobalState::log_mode = 0;


void GlobalState::set_goal_pose(Pose goal_pose)
{
	if (!GlobalState::goal_pose)
		GlobalState::goal_pose = new Pose();

	*GlobalState::goal_pose = goal_pose;
}

void GlobalState::set_robot_pose(Pose robot_pose, double timestamp)
{
	if (!GlobalState::localize_pose)
		GlobalState::localize_pose = new Pose();

	*GlobalState::localize_pose = robot_pose;
	localizer_pose_timestamp = timestamp;
}


Robot_State
GlobalState::predict_initial_robot_state(carmen_point_t robot_current_position, double robot_current_position_timestamp)
{
	Robot_State initial_robot_pose;
	double delta_t;

	Pose pose = Util::convert_to_pose(robot_current_position, 0.0);

	GlobalState::rrt_planner_timestamp = carmen_get_time();// + GlobalState::plan_time;
	if (!GlobalState::log_mode)
		delta_t = GlobalState::rrt_planner_timestamp - robot_current_position_timestamp;
	else
		delta_t = 0.05;

	initial_robot_pose.v_and_phi = GlobalState::last_odometry;
	initial_robot_pose.pose = pose;
	initial_robot_pose = Ackerman::predict_next_pose_during_main_rrt_planning(initial_robot_pose, initial_robot_pose.v_and_phi, delta_t);

	GlobalState::set_robot_pose(initial_robot_pose.pose, GlobalState::rrt_planner_timestamp);

	return initial_robot_pose;
}


Robot_State
GlobalState::estimate_initial_robot_state()
{
	Robot_State initial_robot_pose;
	double delta_t;
	
	GlobalState::rrt_planner_timestamp = carmen_get_time();
	if (!GlobalState::log_mode)
		delta_t = GlobalState::rrt_planner_timestamp - GlobalState::localizer_pose_timestamp;
	else
		delta_t = 0.05;

	initial_robot_pose.pose = *GlobalState::localize_pose;
	initial_robot_pose.v_and_phi = GlobalState::last_odometry;

	initial_robot_pose = Ackerman::predict_next_pose_during_main_rrt_planning(initial_robot_pose, initial_robot_pose.v_and_phi, delta_t);

	return initial_robot_pose;
}
