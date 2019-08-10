/*
 * global_state.h
 *
 *  Created on: 07/03/2012
 *      Author: romulo
 */

#ifndef GLOBAL_STATE_H_
#define GLOBAL_STATE_H_

#include "../model/pose.h"
#include "../model/command.h"
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
	static double param_max_vel;

	static double robot_max_centripetal_acceleration;

	static double max_phi_velocity;		// Equivalente a rodar o volante todo para um lado em 1 segundo.
										// A velocidade de mudanca de phi nao pode ultrapassar este valor
	static double time_to_change_gears;

	static Pose *localizer_pose;
	static Pose *last_plan_pose;
	static double localizer_pose_timestamp;

	static Command last_odometry;

	static Pose *goal_pose;
	static bool last_goal;

	static bool last_path_received_is_empty;

	static carmen_map_t cost_map;
	static carmen_obstacle_distance_mapper_map_message *distance_map;
	static carmen_obstacle_distance_mapper_map_message *distance_map_level1;
	static bool cost_map_initialized;

	static carmen_moving_objects_point_clouds_message *objects_message;
	static bool moving_objects_initialized;

	static std::vector<carmen_ackerman_traj_point_t*> moving_objects_trajectories;

	static bool	  following_path; // true if the path is being followed

	static int	  use_truepos; // if true the algorithm will use the true pose, otherwise will use the localize pose

	static double obstacle_threshold;
	static bool ford_escape_online;
	static carmen_ford_escape_status_message ford_escape_status;
	static int current_algorithm;//which algorithm is running, define at carmen_navigator_ackerman_algorithm_t
	static int behavior_selector_state;
	static int behavior_selector_low_level_state;

	static bool use_obstacle_avoider;
	static bool use_mpc;

	static int publish_tree;
	static int show_debug_info;
	static int publish_lane_map;
	static int reuse_last_path;

	static int use_path_planner;
	static int use_tracker_goal_and_lane;

	static double obstacle_cost_distance;

	static double max_square_distance_to_lane;

	static RRT_Node *goal_node; //the node that represents goal pose on the graph or NULL

	static void set_goal_pose(Pose goal_pose);

	static void set_robot_pose(Pose robot_pose, double timestamp);

	static void set_offline_map(carmen_map_t *new_map);

	static void set_map(carmen_map_t *new_map);

};

#endif /* GLOBAL_STATE_H_ */
