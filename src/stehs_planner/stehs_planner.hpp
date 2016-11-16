#ifndef STEHS_PLANNER_H
#define STEHS_PLANNER_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

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
#include <list>
#include <vector>

#include "CircleNode.hpp"
#include "StateNode.hpp"

#define MIN_OVERLAP_FACTOR 0.5		// if two circles overlaps more than this factor then they are considered connected
#define MAX_OVERLAP_FACTOR 0.1		// if two circles overlaps more than this factor then they are considered the same
#define RGOAL 0.5

class StehsPlanner
{
public:

	// the robot global state
	State start;

	//
	State goal;
	carmen_obstacle_distance_mapper_message *distance_map;

	carmen_behavior_selector_road_profile_message *goal_list_message;

	carmen_robot_ackerman_config_t robot_config;

	// the planner activation flag
	bool active;

	unsigned int show_debug_info;
	unsigned int cheat;
	bool lane_ready, distance_map_ready, goal_ready;

	// circle path
	std::list<CircleNode> circle_path;

	// the trajectory found
	std::list<State> state_list;

	// the min step-rate threshold
	double kmin;

	// TODO it needs to receive the start and goal node

	// constructor
	StehsPlanner();

	// destructor
	~StehsPlanner();

	//
	std::list<CircleNode> SpaceExploration(CircleNodePtr start_node, CircleNodePtr goal_node);

	void RDDFSpaceExploration();

	//
	//void SpaceTimeExploration();

	//
	void HeuristicSearch();

	// the distance between two points
	double Distance(const State &a, const State &b);

	double Distance2(const State &a, const State &b);

	// the distance between two points
	double Distance(double ax, double ay, double bx, double by);

	// the nearest obstacle distance
	double ObstacleDistance(const State &point);

	// the nearest obstacle distance, overloaded version
	double ObstacleDistance(double x, double y);

	//
	std::list<State> BuildPath();

	void Expand(CircleNodePtr current, std::priority_queue<CircleNodePtr, std::vector<CircleNodePtr>, CircleNodePtrComparator> &open_set);

	std::list<CircleNode> BuildCirclePath(CircleNodePtr goal_node);

	bool Exist(CircleNodePtr current, std::vector<CircleNodePtr> &closed_set);

	int FindClosestRDDFIndex();

	void UpdateCircleGoalDistance();

	int FindNextRDDFIndex(double radius_2, int current_rddf_index);

	int FindNextRDDFFreeIndex(int current_rddf_index);

	void ConnectCirclePathGaps();

	unsigned char* GetCurrentMap();

	void ShowCirclePath();

	double TimeHeuristic(State s);

	void BuildStateList(StateNodePtr goal_node);

	bool Exist(StateNodePtr current, std::vector<StateNodePtr> &closed_set, double k);

	void Expand(StateNodePtr current, std::priority_queue<StateNodePtr, std::vector<StateNodePtr>, StateNodePtrComparator> &open_set, double k);

	void GoalExpand(StateNodePtr current, StateNodePtr goal_node);

	void SetSwap(std::priority_queue<StateNodePtr, std::vector<StateNodePtr>, StateNodePtrComparator> &open_set, std::vector<StateNodePtr> &closed_set);


};


#endif
