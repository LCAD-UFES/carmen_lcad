#include <carmen/carmen.h>
#include <carmen/rddf_interface.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/collision_detection.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/offroad_planner.h>
#include <algorithm>
#include <car_model.h>
#include <carmen/global_graphics.h>
#include <carmen/mapper_interface.h>
#include <boost/heap/fibonacci_heap.hpp>

#include <queue>
#include <list>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_math.h>

#include "planning.hpp"


#define DELTA_T 0.01                      // Size of step for the ackerman Euler method

void
compute_astar_path(carmen_point_t *robot_pose, carmen_point_t *goal_pose, carmen_robot_ackerman_config_t robot_config,
		carmen_obstacle_distance_mapper_map_message *distance_map);


typedef struct {
    double state_map_resolution;
    int state_map_theta_resolution;
    int precomputed_cost_size;
    int precomputed_cost_theta_size;
    double precomputed_cost_resolution;
    char *precomputed_cost_file_name;
    int use_matrix_cost_heuristic;

  } carmen_path_planner_astar_t;


typedef struct state_node
{
	carmen_ackerman_traj_point_t state;
	double f;                              // Total distance g + h
	double g;                                // Distance from start to current state
	double distance_traveled_g;
//	double heuristic_g;
//	int heuristic_closed;
	double h;                                // Distance from current state to goal
//	double angular_distance_to_goal;
//	int is_open;
//	int is_closed;
//	double obstacle_distance;
//	int was_visited;
	state_node *parent;
} state_node, *state_node_p;


typedef struct discrete_pos_node
{
	int x;
	int y;
	int theta;
} discrete_pos_node;


typedef struct map_node
{
	int x;
	int y;
	int theta;
	double g;
	int is_open;
	int is_closed;
	double obstacle_distance;
} map_node, *map_node_p;


typedef struct cost_heuristic_node
{
	double h;
} cost_heuristic_node, *cost_heuristic_node_p;


class StateNodePtrComparator {
public:
	bool operator() (state_node *a, state_node *b) const
	{
		return (a->f > b->f);
	}
};

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
