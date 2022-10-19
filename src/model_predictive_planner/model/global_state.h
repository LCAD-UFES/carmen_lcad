/*
 * global_state.h
 *
 *  Created on: 07/03/2012
 *      Author: romulo
 */

#ifndef GLOBAL_STATE_H_
#define GLOBAL_STATE_H_

#include "../pose.h"
#include "../command.h"
#include <carmen/carmen.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/behavior_selector_messages.h>
#include <carmen/moving_objects_interface.h>
#include <list>
#include <vector>

using namespace std;

class GlobalState
{
public:
	static carmen_robot_ackerman_config_t robot_config;
	static carmen_semi_trailers_config_t semi_trailer_config;

	static double param_max_vel;

	static double robot_max_centripetal_acceleration;

	static double max_phi_velocity;		// Equivalente a rodar o volante todo para um lado em 1 segundo.
										// A velocidade de mudanca de phi nao pode ultrapassar este valor

	// Robot delay
	static int eliminate_path_follower;
	static double eliminate_path_follower_transition_v;
	static double robot_velocity_delay;
	static double robot_min_v_distance_ahead;
	static double robot_steering_delay;
	static double robot_min_s_distance_ahead;

	// Optimization weights
	static double w1; // end_of_path_to_goal_distance
	static double w2; // end_of_path_to_goal_angular_distance
	static double w3; // end_of_path_to_goal_delta_theta
	static double w4; // path_to_lane_distance
	static double w5; // proximity_to_obstacles
	static double w6; // traveled_distance
	static double w7; // look_ahead_based_on_stanley_method
	static double look_ahead_horizon; //fraction of the path (in meters it is about N*0.10m) based_on_stanley_method


	static double time_to_change_gears;

	static carmen_robot_and_trailers_pose_t *localizer_pose;
	static carmen_robot_and_trailers_pose_t *last_plan_pose;
	static double localizer_pose_timestamp;

	static Command last_odometry;

	static Pose *goal_pose;
	static bool last_goal;

	static bool last_path_received_is_empty;

	static carmen_map_t cost_map;
	static carmen_obstacle_distance_mapper_map_message *distance_map;
	static bool cost_map_initialized;

	static carmen_moving_objects_point_clouds_message *objects_message;
	static bool moving_objects_initialized;

	static std::vector<carmen_robot_and_trailers_traj_point_t*> moving_objects_trajectories;

	static bool following_path; // true if the path is being followed

	static int	use_truepos; // if true the algorithm will use the true pose, otherwise will use the localize pose

	static bool path_has_collision_or_phi_exceeded;

	static double obstacle_threshold;
	static bool ford_escape_online;
	static carmen_ford_escape_status_message ford_escape_status;
	static int current_algorithm;//which algorithm is running, define at carmen_navigator_ackerman_algorithm_t
	static int behavior_selector_task;
	static int behavior_selector_low_level_state;
	static int route_planner_state;
	static int offroad_planner_request;

	static bool use_obstacle_avoider;
	static bool use_mpc;

	static int publish_tree;
	static int show_debug_info;
	static int publish_lane_map;
	static int reuse_last_path;

	static int use_path_planner;
	static int use_tracker_goal_and_lane;

	static double obstacle_cost_distance;

	static int reverse_driving_flag;
	static int reverse_planning;
	static double distance_between_waypoints;

	static double param_max_vel_reverse;
	static double param_parking_speed_limit;

	static double max_square_distance_to_lane;

	static RRT_Node *goal_node; //the node that represents goal pose on the graph or NULL

	static void set_goal_pose(Pose goal_pose);

	static void set_robot_pose(Pose robot_pose, double timestamp);

	static void set_offline_map(carmen_map_t *new_map);

	static void set_map(carmen_map_t *new_map);

};

#endif /* GLOBAL_STATE_H_ */
