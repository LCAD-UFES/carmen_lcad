#include <carmen/carmen.h>
#include <carmen/rddf_interface.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/collision_detection.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/offroad_planner.h>
#include <carmen/route_planner_interface.h>
#include <carmen/global_graphics.h>
#include <carmen/mapper_interface.h>

#include <algorithm>
#include <car_model.h>
#include <float.h>
#include <math.h>
#include <queue>
#include <list>
#include <vector>
#include <string.h>
#include <stdlib.h>
#include <iostream>

#include <boost/heap/fibonacci_heap.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_math.h>

#include "planning.hpp"


#define OBSTACLE_DISTANCE_MIN 0.5
#define EXPAND_NODES_V 1.42

#define PENALTIES_W1 5.0
#define PENALTIES_W2 15.0

#define SEND_MESSAGE_IN_PARTS 0

#define USE_SMOOTH 1

#define SMOOTHNESS_WEIGHT 100.0
#define OBSTACLE_WEIGHT 10.0
#define CURVATURE_WEIGHT 1.0



#define DELTA_T 0.01                      // Size of step for the ackerman Euler method

#define LANE_WIDTH 	2.4
#define NUM_LANES	1

#define DELTA2D(x1,x2) ((carmen_ackerman_traj_point_t){x1.x - x2.x, x1.y - x2.y, 0.0, 0.0, 0.0})

enum possible_states {Not_visited, Open, Closed};
enum motion_direction {Forward, Backward};

typedef struct {
    double state_map_resolution;
    int state_map_theta_resolution;
    int precomputed_cost_size;
    int precomputed_cost_theta_size;
    double precomputed_cost_resolution;
    char *precomputed_cost_file_name;
    int use_matrix_cost_heuristic;

  } carmen_path_planner_astar_t;


typedef struct pose_node
{
	double x;
	double y;
	double theta;
	motion_direction r;
} pose_node, *pose_node_p;


typedef struct state_node
{
	pose_node pose;
	double g;
	double h;
	double f;
	state_node *parent;
} state_node, *state_node_p;


typedef struct grid_state
{
	possible_states state;
	double g;
} grid_state, *grid_state_p;


typedef struct nonholonomic_heuristic_cost
{
	double h;
} nonholonomic_heuristic_cost, *nonholonomic_heuristic_cost_p;


typedef struct param_otimization
{
	carmen_ackerman_traj_point_t *points;
	int *anchor_points;
	int path_size;
	int problem_size;
} param_t, *param_p;


class StateNodePtrComparator {
public:
	bool operator() (state_node *a, state_node *b) const
	{
		return (a->f > b->f);
	}
};


//Reed Shepp ////////////////////////////

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

//Reed Shepp ////////////////////////////
