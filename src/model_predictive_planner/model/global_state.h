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
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

#include "../kdtree/KDTree2D.hpp"

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
typedef bg::model::point<double, 2, bg::cs::cartesian> occupied_cell;

using namespace std;

class GlobalState
{
public:
	static carmen_robot_ackerman_config_t robot_config;
	static double param_max_vel;

	static double max_phi_velocity;		// Equivalente a rodar o volante todo para um lado em 1 segundo.
										// A velocidade de mudanca de phi nao pode ultrapassar este valor
	static double max_phi_acceleration; // A velocidade de phi pode aumentar no maximo deste valor por segundo

	static double time_to_change_gears;

	static Pose *localizer_pose;
	static Pose *last_plan_pose;
	static double localizer_pose_timestamp;
	static double rrt_planner_timestamp;
	static double last_rrt_path_message_timestamp;

	static Command last_odometry;

	static Robot_State initial_robot_state; // initial state used to recontruct the tree or to follow the path

	static Pose *goal_pose;
	static bool last_goal;

	static bool last_path_received_is_empty;

	static carmen_map_t cost_map;
	static carmen_grid_mapping_distance_map_message *localize_map;
	static bgi::rtree< occupied_cell, bgi::quadratic<16> > obstacles_rtree;
	static bool cost_map_initialized;
	static KDTree2D obstacles_kdtree;
	static vector<vector<cell_coords_t>> cell_mask;

	static bool	  following_path; // true if the path is being followed

	static int	  cheat; // if true the algorithm will use the true pose, otherwise will use the localize pose

	static double obstacle_threshold;

	static int current_algorithm;//which algorithm is running, define at carmen_navigator_ackerman_algorithm_t
	static int behavior_selector_state;

	static bool use_obstacle_avoider;

	static int publish_tree;
	static int show_debug_info;
	static int publish_lane_map;
	static int reuse_last_path;

	static double obstacle_cost_distance;

	static RRT_Node *goal_node; //the node that represents goal pose on the graph or NULL

	static void set_goal_pose(Pose goal_pose);

	static void set_robot_pose(Pose robot_pose, double timestamp);

	static Robot_State estimate_initial_robot_state();
	static Robot_State predict_initial_robot_state(carmen_point_t point, double pose_timestamp);

	static void set_offline_map(carmen_map_t *new_map);

	static void set_map(carmen_map_t *new_map);

};

#endif /* GLOBAL_STATE_H_ */
