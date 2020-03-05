#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <carmen/carmen.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/collision_detection.h>
#include <carmen/rddf_interface.h>
#include <car_model.h>

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
//#define MIN_THETA_DIFF 0.24		          // 15 degree from stehs_planner.h


typedef struct circle_node
{
	double x;
	double y;
	double radius;
	//double f;                             // Total distance g + h
	double g;                             // Distance from start to current state
	double h;                             // Distance from current state to goal state
	circle_node *parent;
} circle_node;


typedef struct state_node
{
	//carmen_ackerman_traj_point_t state;
	carmen_ackerman_traj_point_t state;
	//double f;                              // Total distance g + h
	double g;                                // Distance from start to current state
	double h;                                // Distance from current state to goal
	double angular_distance_to_goal;
	//double step_size;                      // TODO compute step size
	state_node *parent;
} state_node;


class CircleNodeComparator {
public:
	bool operator() (circle_node *a, circle_node *b)
	{
		float w1 = 1.0;
		float w2 = 2.0;
		return ((w1 * a->h - w2 * a->radius) > (w1 * b->h - w2 * b->radius)); // Uses radius as a choice criterion (the bigger the better)
		//return (a->h > b->h);             // The default c++ stl is a max heap, so wee need to invert here
	}
};


class StateNodePtrComparator {
public:
	bool operator() (state_node *a, state_node *b)
	{
	  	float w1 = 1.0;
	  	float w2 = 2.0;
//	  	return ((w1 * a->h + w2 * a->g) > (w1 * b->h + w2 * b->g)); // Uses radius as a choice criterion (the bigger the better)
		return (w1 * a->g + w2 * a->h > w1 * b->g + w2 * b->h);             // return (a->f > b->f);
	}
};


void
compute_path(carmen_point_t *robot_pose, carmen_point_t *goal_pose, carmen_robot_ackerman_config_t robot_config,
		carmen_obstacle_distance_mapper_map_message *distance_map);


typedef enum {
	RS_TURN_RIGHT, RS_TURN_LEFT, RS_STRAIGHT, RS_FWD, RS_BWD, RS_NONE
} RS_POSSIBLE_MOVES;

typedef struct
{
	int turn;
	int move;
} rs_move;

rs_move*
rs_get_moves(int numero);

int
fct_curve(int ty, int orientation, double val, carmen_ackerman_traj_point_p start, double delta, carmen_ackerman_traj_point_p points, int n);

void
rs_init_parameters(double max_phi, double distance_between_front_and_rear_axles);

double
reed_shepp(carmen_ackerman_traj_point_t start, carmen_ackerman_traj_point_t goal, int *numero, double *tr, double *ur, double *vr);

int
constRS(int num, double t, double u, double v, carmen_ackerman_traj_point_t start, carmen_ackerman_traj_point_p points);

#endif
