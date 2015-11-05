/*
 * ackerman_util.h
 *
 *  Created on: 28/05/2012
 *      Author: romulo
 */

#ifndef ACKERMAN_H_
#define ACKERMAN_H_
using namespace std;

#include <vector>
#include <carmen/carmen.h>
#include "../model/command.h"
#include "../model/pose.h"
#include "../model/rrt_node.h"
#include "../model/global_state.h"
#include "../model/tree.h"
#include "cost_function.h"

#define OLD_STEERING_CONTROL

class Ackerman
{
public:
	/**
	 * Initialize the necessary command used by search_command
	 */
	static void init_search_parameters();
	static Pose predict_next_pose(const Pose &global_pose, const Command &command, double interval_time);
	static Pose predict_next_pose(const Pose &pose, const Command &command, double max_traveled_dist, double *time_spend);

	/*
	 * Predict next pose considering acceleration and steering rates
	 */
	static void compute_intermediate_times(double &t_fim_subida, double &t_fim_plato, double &t_fim_descida, double &max_phi_acceleration,
			double initial_phi, double requested_phi, const double full_time_interval, double &max_phi_velocity);

#ifdef OLD_STEERING_CONTROL
	static double predict_next_pose_during_main_rrt_planning_step(Robot_State &new_robot_state, const Command &requested_command, double delta_t,
		double &achieved_curvature, const double &desired_curvature, double &max_curvature_change);
#else
	static double predict_next_pose_during_main_rrt_planning_step(Robot_State &new_robot_state, const Command &requested_command, double &phi_velocity,
		const double t, const double delta_t,
		double t_fim_descida, double t_fim_plato, double t_fim_subida,
		double max_phi_velocity, double max_phi_acceleration);
#endif

	static Robot_State predict_next_pose_during_main_rrt_planning(const Robot_State &robot_pose, const Command &command, double interval_time, double *distance_traveled = NULL, double dt = 0.01);
	static Robot_State predict_next_pose_dist(const Robot_State &robot_pose, const Command &command, double distance_interval, double *time);
	static double get_turning_radius(double phi, double distance_between_front_and_rear_axles);

	/**
	 * Find command using reeds sheep
	 */
	static void find_command_rs(Pose &p1, Pose &p2, vector<Command> &commands, vector<double> &commands_time);
	static void find_command_rs(Robot_State &robot_state, Pose &p2, vector<Command> &commands, vector<double> &commands_time);

	/**
	 * Return time and distance traveled to stop the robot
	 */
	static void get_stop_info(Robot_State robot_state, double phi_command, double *traveled_distance, double *time_spend);

	/**
	 * Return true if the path was changed (first node was removed)
	 */
	static bool remove_front_node_of_the_path(Pose &robot_pose, list<RRT_Path_Edge> &path, double &path_distance, double &theta_diff, Robot_State *path_pose = NULL, double *time_to_reach_path_pose = NULL, double *distance_traveled = NULL, double *total_distance = NULL);

	/**
	 * Get the time spend to travel a given distance
	 */
	static double get_travel_time(Robot_State robot_state, double desired_vel, double traveled_distance, double *velocity_reached);
	static bool is_valid_phi(double current_v, double current_phi, double desired_v, double desired_phi, double time);
	static bool search_command_improved_parking(RRT_Node &near, Pose &rand, RRT_Node *new_node, Command *best_command,
			double *best_command_time, Cost_Function **cost_functions, const double *weights, const int num_cost_function,
			double *min_cost, const vector<double> &velocity_search_vector, Tree &tree);
	static bool search_for_x_new_alternative_and_associated_command_and_time(
			RRT_Node *x_new_alternative, Command *x_new_alternative_associated_command, double *x_new_alternative_associated_time,
			RRT_Node &x_near, Pose &x_rand,
			Cost_Function **cost_functions, const double *weights, const int num_cost_function,
			double *min_cost, Tree &tree);
	static double distance_from_robot_pose_to_curve_between_p1_and_p2(Pose robot_pose, Robot_State p1, Robot_State p2, Command command, double *traveled_dist, double *total_dist, Robot_State *closest_path_pose, double *time_spend = NULL);
	static double get_acceleration_magnitude(const double &v, const double &desired_v);

	static vector<double> velocity_search_vector;
	static vector<double> minimal_velocity_search_vector;

	static double get_env_cost(const Robot_State robot_state);
};

#endif /* ACKERMAN_H_ */
