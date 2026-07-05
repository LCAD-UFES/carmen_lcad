#ifndef NLP_MAT_PLANNER_PATH_PLANNER_H
#define NLP_MAT_PLANNER_PATH_PLANNER_H

#include <carmen/carmen.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/collision_detection.h>
#include <carmen/nlp_mat_planner.h>
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
#include <fstream>
#define _REMOVE_BOOST

#ifdef _REMOVE_BOOST
// #include <boost/heap/fibonacci_heap.hpp>
// #include <boost/heap/d_ary_heap.hpp>
#else
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/d_ary_heap.hpp>
#endif

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_math.h>
// #include <gsl/gsl_roots.h>
#include <gsl/gsl_multiroots.h>

#include "planning.hpp"
#include "offroad_planner_messages.h"
#include "trailer_analytical_expansion.h"

#include <carmen/util_io.h>

#define EXPAND_NODES_V (astar_config.state_map_resolution * M_SQRT2)

#define PENALTIES_W1 150.0 // custo de andar de reh
#define PENALTIES_W2 15.0  // custo de inverter a direcao

#define SEND_MESSAGE_IN_PARTS 0

#define SMOOTHNESS_WEIGHT 1.0
#define OBSTACLE_WEIGHT 20.0
#define CURVATURE_WEIGHT 10.0
#define LINE_FOLLOWING_WEIGHT 0.0
#define THETA_DISPLACEMENT_WEIGHT 0.0
#define USE_LANE_WEIGHT 0
#define LANE_WEIGHT 0.0

#define DELTA_T 0.01 // Size of step for the ackerman Euler method

#define LANE_WIDTH 2.4
#define NUM_LANES 1

#define DELTA2D(x1, x2) ((carmen_robot_and_trailer_traj_point_t){x1.x - x2.x, x1.y - x2.y, 0.0, 0.0, 0.0, 0.0})
#define THETA_DIFF(x1, x2) (carmen_normalize_theta((carmen_normalize_theta((x1).theta) - carmen_normalize_theta((x2).theta))))
#define THETA0_DIFF(x1, x2) (carmen_normalize_theta((carmen_normalize_theta((x1).trailer_theta[0]) - carmen_normalize_theta((x2).trailer_theta[0]))))

#define MAX_POLY_DISTANCE 0.1
#define MAX_POLY_BETA_DIFF carmen_degrees_to_radians(5.0)
#define MAX_POLY_THETA_DIFF carmen_degrees_to_radians(5.0)

// #define ENABLE_CACHING
// #define USE_VALID_CELLS_IN_NONHOLONOMIC_COST_TABLE_ONLY

#define USE_ONLY_ARDENTOV 0
// #define	TEST_RUSSO

#define USE_REED_SHEPP_AS_ANALYTICAL_EXPANSION

// #define USE_TRAILER_NONHOLONOMIC_COST_MAP 1 // Virou o parametro astar_config.use_matrix_cost_trailer_heuristic

enum possible_states
{
	Not_visited,
	Open,
	Closed
};
enum motion_direction
{
	Forward,
	Backward
};

typedef struct
{
	double state_map_resolution;
	int state_map_theta_resolution;
	int state_map_beta_resolution;
	double precomputed_cost_size;
	int precomputed_cost_theta_size;
	double precomputed_cost_resolution;
	char *precomputed_cost_file_name;
	int use_matrix_cost_heuristic;

	double precomputed_cost_trailer_size;
	int precomputed_cost_trailer_theta_size;
	int precomputed_cost_trailer_beta_size;
	double precomputed_cost_trailer_resolution;
	char *precomputed_cost_trailer_file_name;
	int use_matrix_cost_trailer_heuristic;

	int smooth_path;
	double min_dist_motion_change;
	double robot_fat_space;
	double path_smoothing_obstacles_safe_distance;
	double oa_obstacles_safe_distance;
	double penalties_w1;
	double penalties_w2;
	double penalties_w3;
	double penalties_w4;
	double penalties_w5;
	int use_rs_with_trailer;
	double min_distance_to_lane;

	goal_constraints_t goal_constraints;
	int max_change_direction;
	int max_reed_shepp_change_direction;
	double max_reed_shepp_distance_backwards;
	int use_reed_shepp;
	double radius_circle_to_ignore_obstacles_from_final_goal;

	int expansion_phi_resolution;
	double expansion_v_step;
} carmen_path_planner_astar_t;

//
// typedef struct pose_node
//{
//	double x;
//	double y;
//	double theta;
//	double beta;
//	double phi;
//	motion_direction r;
//} pose_node, *pose_node_p;

typedef struct state_node
{
	carmen_robot_and_trailers_traj_point_t pose;
	double g;
	double h;
	double f;
	int it_since_change_in_direction;
	carmen_robot_and_trailers_traj_point_t pose_of_direction_changed;

	int branches;
	int number_of_change_in_direction;

	state_node *parent;
} state_node, *state_node_p;

typedef struct index_node
{
	int x;
	int y;
	int theta;
	int trailer_theta[MAX_NUM_TRAILERS];
	int direction;
} index_node, *index_node_p;

typedef struct grid_state
{
	possible_states state;
	double g;
} grid_state, *grid_state_p;

typedef struct nonholonomic_heuristic_cost_trailer
{
	double h_forward;
	bool h_forward_valid;
	double h_backward;
	bool h_backward_valid;
	int indice;
	carmen_robot_and_trailers_traj_point_t *path;
} nonholonomic_heuristic_cost_trailer, *nonholonomic_heuristic_cost_trailer_p;

typedef struct nonholonomic_heuristic_cost
{
	double h;
} nonholonomic_heuristic_cost, *nonholonomic_heuristic_cost_p;

class StateNodePtrComparator
{
public:
	bool operator()(state_node *a, state_node *b) const
	{
		return (a->f > b->f);
	}
};

// Reed Shepp ////////////////////////////

typedef enum
{
	RS_TURN_RIGHT,
	RS_TURN_LEFT,
	RS_STRAIGHT,
	RS_FWD,
	RS_BWD,
	RS_NONE
} RS_POSSIBLE_MOVES;

typedef struct
{
	int turn;
	int move;
} rs_move;

// //////////

using namespace std;

class GlobalState
{
public:
	static carmen_robot_ackerman_config_t robot_config;
	static carmen_semi_trailers_config_t semi_trailer_config;

	static double param_max_vel;

	static double robot_max_centripetal_acceleration;

	static double max_phi_velocity; // Equivalente a rodar o volante todo para um lado em 1 segundo.
									// A velocidade de mudanca de phi nao pode ultrapassar este valor

	// Robot delay
	static int eliminate_path_follower;
	static double eliminate_path_follower_transition_v;
	static double robot_velocity_delay;
	static double robot_min_v_distance_ahead;
	static double robot_steering_delay;
	static double robot_min_s_distance_ahead;

	// Optimization weights
	static double w1;				  // end_of_path_to_goal_distance
	static double w2;				  // end_of_path_to_goal_angular_distance
	static double w3;				  // end_of_path_to_goal_delta_theta
	static double w4;				  // path_to_lane_distance
	static double w5;				  // proximity_to_obstacles
	static double w6;				  // traveled_distance
	static double w7;				  // look_ahead_based_on_stanley_method
	static double look_ahead_horizon; // fraction of the path (in meters it is about N*0.10m) based_on_stanley_method

	static double time_to_change_gears;

	static carmen_robot_and_trailers_pose_t *localizer_pose;
	static carmen_robot_and_trailers_pose_t *last_plan_pose;
	static double localizer_pose_timestamp;

	static bool last_goal;

	static bool last_path_received_is_empty;

	static carmen_map_t cost_map;
	static carmen_obstacle_distance_mapper_map_message *distance_map;
	static bool cost_map_initialized;

	static carmen_moving_objects_point_clouds_message *objects_message;
	static bool moving_objects_initialized;

	static std::vector<carmen_robot_and_trailers_traj_point_t *> moving_objects_trajectories;

	static bool following_path; // true if the path is being followed

	static int use_truepos; // if true the algorithm will use the true pose, otherwise will use the localize pose

	static bool path_has_collision_or_phi_exceeded;

	static double obstacle_threshold;
	static bool ford_escape_online;
	static carmen_ford_escape_status_message ford_escape_status;
	static int current_algorithm; // which algorithm is running, define at carmen_navigator_ackerman_algorithm_t
	static int behavior_selector_task;
	static int behavior_selector_low_level_state;
	static int route_planner_state;
	static int offroad_planner_request;

	static bool use_obstacle_avoider;
	static bool use_mpc;

	static int publish_tree;
	static int show_debug_info;
	static int publish_lane_map;
	static int reuse_last_path;

	static int use_path_planner;
	static int use_tracker_goal_and_lane;

	static double obstacle_cost_distance;

	static int reverse_driving_flag;
	static int reverse_planning;
	static double param_max_vel_reverse;
	static double param_parking_speed_limit;
	static double distance_between_waypoints;

	static double max_square_distance_to_lane;
};

rs_move *
rs_get_moves(int numero);

int fct_curve(int ty, int orientation, double val, carmen_robot_and_trailers_traj_point_t *start, double delta, carmen_robot_and_trailers_traj_point_t *points, int n);

void rs_init_parameters(double max_phi, double distance_between_front_and_rear_axles);

double
reed_shepp(carmen_robot_and_trailers_traj_point_t start, carmen_robot_and_trailers_traj_point_t goal, int *numero, double *tr, double *ur, double *vr);

int constRS(int num, double t, double u, double v, carmen_robot_and_trailers_traj_point_t start, carmen_robot_and_trailers_traj_point_t *points);

int get_index_of_nearest_pose_in_path(carmen_robot_and_trailers_traj_point_t *path, carmen_point_t globalpos, int path_length);

carmen_robot_and_trailers_traj_point_t *
get_poses_back(carmen_robot_and_trailers_traj_point_t *path, int nearest_pose_index);

void add_lanes(carmen_route_planner_road_network_message &route_planner_road_network_message,
			   carmen_robot_and_trailers_traj_point_t *path_copy);

void free_lanes(carmen_route_planner_road_network_message route_planner_road_network_message);

double *
get_goal_distance_map(double *goal_distance_map, carmen_point_t *goal_pose, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map);

std::vector<carmen_robot_and_trailers_traj_point_t>
carmen_path_planner_astar_search(carmen_offroad_planner_feedback_t &feedback,
								carmen_robot_and_trailers_traj_point_t *initial_pose, carmen_robot_and_trailers_traj_point_t *goal_pose,
								carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map, double *goal_distance_map,
								double *nonholonomic_heuristic_cost_map,
								carmen_route_planner_road_network_message *road_network_message);

int smooth_rddf_using_conjugate_gradient(std::vector<carmen_robot_and_trailers_traj_point_t> &astar_path);

carmen_map_t *
copy_grid_mapping_to_map(carmen_map_t *map, carmen_mapper_map_message *grid_map);

void alloc_cost_map();

void alloc_trailer_cost_map();

double
carmen_compute_abs_angular_distance(double theta_1, double theta_2);

int get_grid_state_map_x(double x, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map);

int get_grid_state_map_y(double y, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map);

bool update_goal_distance_map(carmen_robot_and_trailers_traj_point_t *requested_goal);

// vector<carmen_robot_and_trailers_traj_point_t>
// trailer_nlp_analytical_expansion(carmen_robot_and_trailers_traj_point_t current_pose, carmen_robot_and_trailers_traj_point_t goal_pose,
//		carmen_robot_ackerman_config_t robot_config, carmen_semi_trailers_config_t semi_trailer_config, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map,
//		double max_phi_multiplier, bool test_limits, bool make_test);

vector<carmen_robot_and_trailers_traj_point_t>
trailer_polynomial_analytical_expansion_new(carmen_robot_and_trailers_traj_point_t current_pose, carmen_robot_and_trailers_traj_point_t goal_pose,
											int direction, carmen_robot_ackerman_config_t robot_config, carmen_semi_trailers_config_t semi_trailer_config, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map,
											carmen_path_planner_astar_t astar_config, double max_phi_multiplier, bool test_limits);

bool near_enough(double distance, double theta_diff, double beta_diff);

std::vector<carmen_robot_and_trailers_traj_point_t>
increase_path_resolution(std::vector<carmen_robot_and_trailers_traj_point_t> path);

double get_offroad_min_resolution_path_param();

#endif
