#ifndef MPC_H
#define MPC_H


using namespace std;


#include <carmen/carmen.h>
#include <car_model.h>
#include <fann.h>
#include <fann_data.h>
#include <floatfann.h>
#include <fann_train.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_multimin.h>
#include <obstacle_avoider_interface.h>
#include <list>
#include <vector>


#define DELTA_T 0.025 					// 0.025 40 Htz
#define PREDICTION_HORIZON	0.4 		//Must be DELTA_T multiple
#define VELOCITY_PREDICTION_HORIZON	0.8 //Must be DELTA_T multiple
#define POSITION_PREDICTION_HORIZON	1.2 //Must be DELTA_T multiple
#define CAR_MODEL_GAIN 200.0
#define CONTROL_OUTPUT_GAIN 0.0
#define SMOOTH_OUTPUT_FACTOR 0.0


typedef struct {
	double k1;
	double k2;
	double k3;
	double k4;
} EFFORT_SPLINE_DESCRIPTOR;

typedef struct {
	vector<double> x;
	vector<double> y;
	vector<double> v;
	vector<double> phi;
	double total_time_of_commands;
} MOTION_COMMAND;


// Definition of type of pointer to function that will be passed to the optimizer
//typedef vector<double> (*get_vector_from_spline)(EFFORT_SPLINE_DESCRIPTOR *descriptors, void *params);

typedef double (*get_vector_from_spline)(EFFORT_SPLINE_DESCRIPTOR *descriptors, void *params);


typedef struct
{
	carmen_robot_and_trailers_motion_command_t *motion_commands_vector;
	unsigned int motion_commands_vector_size;

	MOTION_COMMAND path;
	MOTION_COMMAND optimized_path;

	struct fann *steering_ann;
	fann_type steering_ann_input[NUM_STEERING_ANN_INPUTS];
	struct fann *velocity_ann;
	fann_type velocity_ann_input[NUM_VELOCITY_ANN_INPUTS];

	double atan_current_curvature;
	double current_velocity;
	double understeer_coeficient;
	double distance_rear_axles;
	double dk;												// Disturbance error, to compensate for changes not modeled
	double previous_k1;

	double velocity_error_dk; 								// dk of velocity control
	double previous_velocity_k1; 							// previous velocity effort to compute velocity_error (dk)

	double time_elapsed_since_last_motion_command; 			// Time of velodyne message, the trajectory is planned at this time, the elapsed time must be discounted of the trajectory
	double max_phi;

	carmen_localize_ackerman_globalpos_message global_pos;
	carmen_robot_ackerman_config_t *robot_config;

	get_vector_from_spline get_vector_function;				// Pointer to function that will be used to extract the vector of the spline

} PARAMS;


vector<double>
get_effort_vector_from_spline_descriptors(EFFORT_SPLINE_DESCRIPTOR *descriptors, double prediction_horizon);


unsigned int
get_motion_timed_index_to_motion_command(PARAMS* params);

vector<double>
get_velocity_supersampling_motion_commands_vector(PARAMS *params, unsigned int size);


double
car_steering_model(double steering_effort, double atan_current_curvature, double v, fann_type *steering_ann_input, PARAMS *params);


vector<double>
get_phi_vector_from_spline_descriptors(EFFORT_SPLINE_DESCRIPTOR *descriptors, PARAMS *params);


MOTION_COMMAND
get_motion_commands_vector(carmen_robot_and_trailers_motion_command_t *current_motion_command_vector, int nun_motion_commands, double time_of_velodyne_message, double prediction_horizon);


void
plot_velocity(EFFORT_SPLINE_DESCRIPTOR *descriptors, double current_velocity, PARAMS *params, double prediction_horizon);


void
plot_phi(EFFORT_SPLINE_DESCRIPTOR *descriptors, double current_phi, PARAMS *params, double prediction_horizon);

void
plot_position(EFFORT_SPLINE_DESCRIPTOR *descriptors, double current_y, double current_x, PARAMS *params);


#endif // MPC_H
