#ifndef MPC_H
#define MPC_H


#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/carmen.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_multimin.h>
#include <car_model.h>
#include <obstacle_avoider_interface.h>


#define DELTA_T (1.0 / 40.0) // 0.025 40 Htz
#define PREDICTION_HORIZON	0.4 //Must be DELTA_T multiple
#define CAR_MODEL_GAIN 200.0
#define CONTROL_OUTPUT_GAIN 0.0
#define SMOOTH_OUTPUT_FACTOR 0.0


typedef struct {
	double k1;
	double k2;
	double k3;
	double k4;
} EFFORT_SPLINE_DESCRIPTOR;


typedef struct
{
	carmen_ackerman_motion_command_t *motion_commands_vector;
	unsigned int motion_commands_vector_size;
	struct fann *steering_ann;
	fann_type steering_ann_input[NUM_STEERING_ANN_INPUTS];
	double atan_current_curvature;
	double v;
	double understeer_coeficient;
	double distance_rear_axles;
	double dk;
	double previous_k1;
	double time_elapsed_since_last_motion_command;
	double max_phi;
	carmen_localize_ackerman_globalpos_message global_pos;
	carmen_robot_ackerman_config_t *robot_config;
} PARAMS;


double
carmen_libmpc_get_optimized_steering_effort_using_MPC(double atan_current_curvature,
		carmen_ackerman_motion_command_p current_motion_command_vector,
		int nun_motion_commands, double v, double yp, double time_of_last_motion_command,
		carmen_robot_ackerman_config_t *robot_config,
		int initialize_neural_networks);


double
carmen_libmpc_get_optimized_steering_effort_using_MPC_position_control(double atan_current_curvature,
		carmen_ackerman_motion_command_p current_motion_command_vector,
		int nun_motion_commands, double v, double yp, double time_of_last_motion_command,
		double understeer_coeficient, double distance_between_front_and_rear_axles, double max_phi, double maximum_steering_command_rate,
		carmen_localize_ackerman_globalpos_message global_pos, int initialize_neural_networks);

int
libmpc_stiction_simulation(double effort, double v);

double
libmpc_stiction_correction(double current_phi, double desired_phi, double effort, double v);


#ifdef __cplusplus
}
#endif

#endif // PID_H
