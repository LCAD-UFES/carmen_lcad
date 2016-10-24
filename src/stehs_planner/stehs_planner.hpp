#ifndef STEHS_PLANNER_H
#define STEHS_PLANNER_H

#include <list>

#include <carmen/carmen.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/ford_escape_hybrid_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/collision_detection.h>

#include <cmath>
#include <queue>

#include "CircleNode.hpp"

class StehsPlanner
{
public:

	// the robot global state
	carmen_ackerman_traj_point_t start;

	//
	carmen_ackerman_traj_point_t goal;
	carmen_obstacle_distance_mapper_message *distance_map;

	carmen_behavior_selector_road_profile_message *goal_list_message;

	carmen_robot_ackerman_config_t robot_config;

	// the planner activation flag
	bool active;

	unsigned int show_debug_info;
	unsigned int cheat;
	bool ready;

	// circle path
	std::list<CircleNode> circle_path;

	// the trajectory found
	std::list<carmen_ackerman_motion_command_t> command_list;

	// TODO it needs to receive the start and goal node
	bool SpaceExploration(CircleNodePtr start_node, CircleNodePtr goal_node);

	void RDDFSpaceExploration();

	//
	//void SpaceTimeExploration();

	//
	//void HeuristicSearch();

	// the distance between two points
	double Distance(const carmen_ackerman_traj_point_t &a, const carmen_ackerman_traj_point_t &b);

	double Distance2(const carmen_ackerman_traj_point_t &a, const carmen_ackerman_traj_point_t &b);

	// the distance between two points
	double Distance(double ax, double ay, double bx, double by);

	// the nearest obstacle distance
	double ObstacleDistance(const carmen_ackerman_traj_point_t &point);

	// the nearest obstacle distance, overloaded version
	double ObstacleDistance(double x, double y);

	// constructor
	StehsPlanner();

	//
	std::list<carmen_ackerman_motion_command_t> BuildPath();

	std::vector<CircleNodePtr> Expand(CircleNodePtr current);

	void BuildCirclePath(CircleNodePtr goal_node);

	bool Exist(CircleNodePtr current, std::vector<CircleNodePtr> &closed_set);

	int FindClosestRDDFIndex();

	int FindNextRDDFIndex(double radius_2, int current_rddf_index);

};


#endif
