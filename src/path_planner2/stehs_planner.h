#ifndef STEHS_PLANNER_H_
#define STEHS_PLANNER_H_

#include <carmen/carmen.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/collision_detection.h>

#include <queue>
#include <list>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


#define MIN_OVERLAP_FACTOR 0.5	          // if two circles overlaps more than this factor then they are considered connected
#define MAX_OVERLAP_FACTOR 0.1	          // if two circles overlaps more than this factor then they are considered the same
#define RGOAL 3.5				          // ???
#define DELTA_T 0.01                      // Size of step for the ackerman Euler method
#define ALFA 1                	          // Weight of nearest circle radius for step_size
#define BETA 1               	          // Weight of nearest circle path distance to goal for step_size
#define MAX_STEP_SIZE 2.0		          // max step size in seconds
#define KMIN 0.0125 			          // Step rate multiplier
#define MIN_THETA_DIFF 0.24		          // 15 degree


typedef struct circle_node
{
	double x;
	double y;
	double radius;
	double g;                             // (distance) cost from start to current state
	double f;                             // total (distance) cost, g + h
	double h;                             // the (distance) cost from current state to goal
	circle_node *parent;
} circle_node;


typedef struct state_node
{
	carmen_ackerman_traj_point_t state;
	double g;                             // the (time) cost from start to current state
	double f;                             // the total (time) cost, g + h
	double h;                             // the (time) cost from current state to goal
	double step_size;
	state_node *parent;
} state_node;


class CircleNodeComparator {
public:
	bool operator() (circle_node *a, circle_node *b)
	{
		return (a->f > b->f);             // The default c++ stl is a max heap, so wee need to invert here
	}
};


void
compute_rddf_using_stehs_planner(carmen_point_t *robot_pose, carmen_point_t *goal, carmen_obstacle_distance_mapper_map_message *distance_map);

#endif
