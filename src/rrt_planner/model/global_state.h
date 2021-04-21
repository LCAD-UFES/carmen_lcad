/*
 * global_state.h
 *
 *  Created on: 07/03/2012
 *      Author: romulo
 */

#ifndef GLOBAL_STATE_H_
#define GLOBAL_STATE_H_

#include "../model/pose.h"
#include "../model/robot_config.h"
#include "../model/cost_map.h"
#include "../model/command.h"
#include "../model/robot_state.h"
#include <carmen/carmen.h>
#include <carmen/behavior_selector_messages.h>
#include <list>

using namespace std;

class GlobalState
{
public:
	static Robot_Config robot_config;
	static double param_max_vel;

	static double max_phi_velocity;		// Equivalente a rodar o volante todo para um lado em 1 segundo.
										// A velocidade de mudanca de phi nao pode ultrapassar este valor
	static double max_phi_acceleration; // A velocidade de phi pode aumentar no maximo deste valor por segundo

	static double time_to_change_gears;

	static Pose *localize_pose;
	static double localizer_pose_timestamp;
	static double rrt_planner_timestamp;
	static double last_rrt_path_message_timestamp;

	static Command last_odometry;

	static Robot_State initial_robot_state; // initial state used to recontruct the tree or to follow the path

	static Pose *goal_pose;
	static bool goal_just_set;
	static bool last_goal;

	static bool last_path_received_is_empty;

	static vector<carmen_point_t> lane_points;
	static vector<Pose> lane_points_on_map;
	static carmen_map_t lane_map;
	static carmen_map_t cost_map;
	static Gradient_Cost_Map utility_map;
	static bool cost_map_initialized;
	static bool gradient_cost_map_old;

	static bool	  following_path; // true if the path is being followed

	static double distance_interval; // max distance between waypoints
	static double param_distance_interval; // max distance between waypoints

	static double timeout; // timeout in seconds
	static double plan_time; // time in seconds that the planner will respond if a path was found

	static int	  use_truepos; // if true the algorithm will use the true pose, otherwise will use the localize pose

	static double obstacle_threshold;

	static int current_algorithm;//which algorithm is running, define at carmen_navigator_ackerman_algorithm_t
	static int behavior_selector_task;

	static char *rddf_path;

	static int publish_tree;
	static int show_debug_info;
	static int publish_lane_map;
	static int reuse_last_path;

	static double obstacle_cost_distance;

	static RRT_Node *goal_node; //the node that represents goal pose on the graph or NULL

	static int log_mode;

	static void set_goal_pose(Pose goal_pose);

	static void set_robot_pose(Pose robot_pose, double timestamp);

	static Robot_State estimate_initial_robot_state();
	static Robot_State predict_initial_robot_state(carmen_point_t point, double pose_timestamp);

	static void set_offline_map(carmen_map_t *new_map);

	static void set_map(carmen_map_t *new_map);

};

#endif /* GLOBAL_STATE_H_ */
