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
#include <prob_map.h>

#include "CircleNode.hpp"

class StehsPlanner
{

private:

	// the robot global state
	carmen_ackerman_traj_point_t start;

	//
	carmen_ackerman_traj_point_t goal;
	carmen_obstacle_distance_mapper_message *distance_map;

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

	//
	void SpaceExploration();

	//
	void SpaceTimeExploration();

	//
	void HeuristicSearch();

	// the distance between two points
	double Distance(const carmen_ackerman_traj_point_t &a, const carmen_ackerman_traj_point_t &b);

	// the nearest obstacle distance
	double ObstacleDistance(const carmen_ackerman_traj_point_t &p);

public:

	// constructor
	StehsPlanner();

	//
	std::list<carmen_ackerman_motion_command_t> BuildPath();

};


#endif
