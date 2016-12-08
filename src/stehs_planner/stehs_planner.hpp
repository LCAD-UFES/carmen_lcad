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
#include <car_model.h>

#include <cmath>
#include <queue>
#include <list>
#include <vector>
#include <algorithm>

#include "CircleNode.hpp"
#include "StateNode.hpp"

#define MIN_OVERLAP_FACTOR 0.5		// if two circles overlaps more than this factor then they are considered connected
#define MAX_OVERLAP_FACTOR 0.1		// if two circles overlaps more than this factor then they are considered the same
#define RGOAL 3.5					// ???
#define DELTA_T 0.01                // Size of step for the ackerman Euler method
#define ALFA 1                		// Weight of nearest circle radius for step_size
#define BETA 1               		// Weight of nearest circle path distance to goal for step_size
#define MAX_STEP_SIZE 2.0			// max step size in seconds
#define KMIN 0.0125 				// Step rate multiplier
#define MIN_THETA_DIFF 0.24			// 15 degree

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
    std::list<carmen_ackerman_path_point_t> state_list;

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

    cv::Mat ShowCirclePath(std::vector<StateNodePtr> &state_node);

    void ShowCirclePath();

    cv::Mat ShowState(StateNodePtr &state_node, cv::Mat img);

    double TimeHeuristic(const State &state);

    double DistanceHeuristic(const State &state);

    void BuildStateList(StateNodePtr goal_node);

    bool Exist(StateNodePtr current, std::vector<StateNodePtr> &closed_set, double k);

    double compute_new_phi_with_ann(double steering_effort, double current_phi, double current_v);

    StateNodePtr GetNextState(StateNodePtr current_state, double a, double w, double step_size);

    std::list<CircleNode>::iterator FindNearestCircle(const State &state);

    double UpdateStep(StateNodePtr state_node);
    bool Collision(StateNodePtr state_node);

    void Expand(
                StateNodePtr current,
                std::priority_queue<StateNodePtr, std::vector<StateNodePtr>, StateNodePtrComparator> &open_set,
                std::vector<StateNodePtr> &closed_set,
                double k);

    void GoalExpand(StateNodePtr current, StateNodePtr &goal_node,
    		std::priority_queue<StateNodePtr, std::vector<StateNodePtr>, StateNodePtrComparator> &open_set);

    void SetSwap(std::priority_queue<StateNodePtr, std::vector<StateNodePtr>, StateNodePtrComparator> &open_set, std::vector<StateNodePtr> &closed_set);

    void GeneratePath();
};


#endif
