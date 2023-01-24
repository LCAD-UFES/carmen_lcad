/*
 * model_predictive_planner_optimizer.h
 *
 *  Created on: Jun 23, 2016
 *      Author: lcad
 */

#ifndef MODEL_PREDICTIVE_PLANNER_OPTIMIZER_H_
#define MODEL_PREDICTIVE_PLANNER_OPTIMIZER_H_

#include <vector>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_multimin.h>
#include <car_model.h>

using namespace std;


#define N_DIST			21	//15 or 21	// Number of Distances traveled in polar coordinates
#define FIRST_DIST		2.3				// First Distance, or scale factor of its geometric progression (Wikipedia)
#define RATIO_DIST		1.18			// Ratio (Wikipedia) of the Distance geometric progression
#define ZERO_DIST_I		-1				// Index of zero Distance traveled

#define N_THETA			15 //19 //25 //15 				// Number of Angles in polar coordinates
#define COMMON_THETA	((8.0 * M_PI) / 180.0)	// Common difference in the arithmetic progression of Theta for each side
#define ZERO_THETA_I	7 //9 //12 //7				// Index of zero Angle

#define N_D_YAW			15 //21 			// Number of Displacements in yaw
#define FIRST_D_YAW		((10.0 * M_PI) / 180.0)	// First Displacement in yaw, or scale factor of its geometric progression
#define RATIO_D_YAW		1.3				// Ratio (Wikipedia) of the Displacements in yaw geometric progression
#define ZERO_D_YAW_I	7 //10 			// Index of zero yaw displacement

#define N_I_PHI			15				// Number of Initial steering wheel angles (phi)
#define FIRST_I_PHI		((3.0 * M_PI) / 180.0)	// Scale factor of phi geometric progression
#define RATIO_I_PHI		1.394			// Ratio (Wikipedia) of phi geometric progression
#define ZERO_I_PHI_I	7				// Index of zero initial phi

#define N_K2			15				// Number of k2 wheel angles
#define FIRST_K2		((3.0 * M_PI) / 180.0)	// Scale factor of k2 geometric progression
#define RATIO_K2		1.394			// Ratio (Wikipedia) of k2 geometric progression
#define ZERO_K2_I		7				// Index of zero k2

#define N_K3			15				// Number of k3 wheel angles
#define FIRST_K3		((3.0 * M_PI) / 180.0)	// Scale factor of k3 geometric progression
#define RATIO_K3		1.394			// Ratio (Wikipedia) of k3 geometric progression
#define ZERO_K3_I		7				// Index of zero k3

#define N_I_V			14				// Number of Initial velocities (it was 9 before reverse_driving)
#define FIRST_I_V		1.3				// First Initial velocity, or scale factor of its geometric progression
#define RATIO_I_V		1.381				// Ratio (Wikipedia) of Initial velocity geometric progression
#define ZERO_I_V_I		4				// Index of zero Initial velocity (it was 0 before reverse_driving)

#define N_D_V			8				// Number of Velocities displacements
#define FIRST_D_V		1.6				// First Velocity displacement, or scale factor of its geometric progression
#define RATIO_D_V		1.3				// Ratio (Wikipedia) of Velocity displacement geometric progression
#define ZERO_D_V_I		4				// Index of zero Velocity displacement

#define NUM_VELOCITY_PROFILES	4

#define LATENCY_CICLE_TIME		0.01
#define MAX_PLANNING_TIME		1.0

//To error layout
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define RESET "\033[0m"


typedef struct
{
	bool valid;
	vector<double> k;	// nos do spline de phi
	double s;	// comprimento da trajetoria ajustado no processo de otimizacao
	double tt;	// tempo total da trajetoria ajustado no processo de otimizacao (eh dirivado de s)
	double a;	// aceleracao ajustada durante a otimizacao (eh dirivado de s)
	double vf;	// velocidade final apos a otimizacao
	double sf;	// comprimento final apos a otimizacao
} TrajectoryControlParameters;

typedef struct
{
	double dist;		// Distance traveled in polar coordinates
	double theta;		// Angle in polar coordinates
	double d_yaw;		// Displacement in yaw
	double phi_i;		// Initial steering wheel angle
//	double beta_i;		// Initial semitrailer beta angle
	double trailer_theta_i[MAX_NUM_TRAILERS];
	double v_i;			// Initial velocity
	carmen_robot_and_trailers_pose_t goal_pose;	// Goal pose in car coordinates
	TrajectoryControlParameters control_parameters;
} TrajectoryDimensions;

typedef struct
{
	unsigned int dist;		// Distance traveled in polar coordinates
	unsigned int theta;	// Angle in polar coordinates
	unsigned int d_yaw;	// Displacement in yaw
	unsigned int phi_i;	// Initial steering wheel angle
	unsigned int v_i;		// Initial velocity
} TrajectoryDiscreteDimensions;

typedef struct
{
	double phi;
	double timestamp;
} steering_delay_t;


struct ObjectiveFunctionParams
{
	double target_v, suitable_acceleration, suitable_tt;
	double distance_by_index;
	double theta_by_index;
	double d_yaw_by_index;
	TrajectoryControlParameters *tcp_seed;
	TrajectoryDimensions *target_td;
	vector<carmen_robot_and_trailers_path_point_t> detailed_lane;
	vector<unsigned int> path_point_nearest_to_lane;
	unsigned int path_size;
	bool use_lane;
	int optimize_time;
	double plan_cost;
	double max_plan_cost;
	double (* my_f) (const gsl_vector  *x, void *params);
	double o_step_size;
	double o_tol;
	double o_epsabs;
	int max_iterations;
};


void save_trajectory_lookup_table();
TrajectoryDimensions convert_to_trajectory_dimensions(TrajectoryDiscreteDimensions tdd,
		TrajectoryControlParameters tcp);
TrajectoryDiscreteDimensions get_discrete_dimensions(TrajectoryDimensions td);
bool has_valid_discretization(TrajectoryDiscreteDimensions tdd);
TrajectoryControlParameters search_lookup_table(TrajectoryDiscreteDimensions tdd);

vector<carmen_robot_and_trailers_path_point_t> simulate_car_from_parameters(TrajectoryDimensions &td,
		TrajectoryControlParameters &tcp, double v0, double *i_trailer_theta,
		double delta_t = 0.15);
//vector<carmen_ackerman_path_point_t> simulate_car_from_parameters(TrajectoryLookupTable::TrajectoryDimensions &td,
//		TrajectoryLookupTable::TrajectoryControlParameters &tcp, double v0, double i_phi,
//		bool display_phi_profile, double delta_t = 0.1);

bool path_has_loop(double dist, double sf);
void move_path_to_current_robot_pose(vector<carmen_robot_and_trailers_path_point_t> &path, carmen_robot_and_trailers_pose_t *localizer_pose);

float get_d_yaw_by_index(int index);
float get_theta_by_index(int index);
float get_distance_by_index(int index);

gsl_spline *get_phi_spline(TrajectoryControlParameters tcp);


vector<carmen_robot_and_trailers_path_point_t> apply_robot_delays(vector<carmen_robot_and_trailers_path_point_t> &original_path);

void plot_state(vector<carmen_ackerman_path_point_t> &pOTCP, vector<carmen_ackerman_path_point_t> &pLane,
		  vector<carmen_ackerman_path_point_t> &pSeed, std::string titles[]);

void print_lane(vector<carmen_robot_and_trailers_path_point_t> path, char *file_name);

TrajectoryControlParameters get_complete_optimized_trajectory_control_parameters(TrajectoryControlParameters previous_good_tcp,
		TrajectoryDimensions target_td, double target_v, vector<carmen_robot_and_trailers_path_point_t> detailed_lane,
		bool use_lane);

TrajectoryControlParameters get_optimized_trajectory_control_parameters(TrajectoryControlParameters tcp_seed, ObjectiveFunctionParams &params);

void get_optimization_params(ObjectiveFunctionParams &params, double target_v,
		TrajectoryControlParameters *tcp_seed,
		TrajectoryDimensions *target_td,
		double max_plan_cost, int max_iterations,
		double (* my_f) (const gsl_vector  *x, void *params));

void compute_suitable_acceleration_and_tt(ObjectiveFunctionParams &params,
		TrajectoryControlParameters &tcp_seed,
		TrajectoryDimensions target_td, double target_v);

TrajectoryControlParameters fill_in_tcp(const gsl_vector *x, ObjectiveFunctionParams *params);

bool bad_tcp(TrajectoryControlParameters tcp);

TrajectoryControlParameters get_n_knots_tcp_from_detailed_lane(vector<carmen_robot_and_trailers_path_point_t> detailed_lane,
		int n, double v_i, double phi_i, double d_yaw, double a, double s, double tt);

void get_between_points(carmen_robot_and_trailers_path_point_t robot, carmen_robot_and_trailers_path_point_t point_before, carmen_robot_and_trailers_path_point_t center, carmen_robot_and_trailers_path_point_t point_next,
		int index_center, int &index_p1, int &index_p2, int &mais_proxima);

double get_distance_between_point_to_line2(carmen_robot_and_trailers_path_point_t p1,
		carmen_robot_and_trailers_path_point_t p2,
		carmen_robot_and_trailers_path_point_t robot);

double compute_path_to_lane_distance(ObjectiveFunctionParams *my_params, vector<carmen_robot_and_trailers_path_point_t> &path);

double compute_proximity_to_obstacles_using_distance_map(vector<carmen_robot_and_trailers_path_point_t> path);

void compute_path_points_nearest_to_lane(ObjectiveFunctionParams *param, vector<carmen_robot_and_trailers_path_point_t> &path);

double mpp_optimization_function_g(const gsl_vector *x, void *params);

double mpp_optimization_function_f(const gsl_vector *x, void *params);

void get_tcp_with_n_knots(TrajectoryControlParameters &tcp, int n);


#endif /* MODEL_PREDICTIVE_PLANNER_OPTIMIZER_H_ */
