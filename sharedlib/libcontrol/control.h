#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif


//#define PLOT
//#define PRINT
//#define SAVE

#include <fann.h>
#include <fann_data.h>
#include <floatfann.h>
#include <fann_train.h>

double
carmen_libmpc_get_optimized_steering_effort_using_MPC(double atan_current_curvature,
		carmen_ackerman_motion_command_p current_motion_command_vector, int nun_motion_commands,
		double v, double yp, double time_of_last_motion_command,
		carmen_robot_ackerman_config_t *robot_config, int initialize_neural_networks);


double
carmen_libmpc_compute_efforts(double *throttle_effort, double *brake_effort, double *steering_effort, double atan_current_curvature,
		carmen_ackerman_motion_command_p current_motion_command_vector,	int nun_motion_commands,
		double v, double yp, double time_of_last_motion_command,
		carmen_robot_ackerman_config_t *robot_config,
		int initialize_neural_networks);

double
carmen_libmpc_compute_velocity_effort(carmen_ackerman_motion_command_p current_motion_command_vector, int nun_motion_commands,
		double current_velocity, double time_of_last_motion_command, carmen_robot_ackerman_config_t *robot_config);

double
carmen_libpid_steering_PID_controler(double atan_desired_curvature, double atan_current_curvature, double delta_t);


void
carmen_libpid_velocity_PID_controler(double *throttle_command, double *brakes_command, int *gear_command,
											double desired_velocity, double current_velocity, double delta_t);


void
carmen_libpid_read_PID_parameters(int argc, char *argv[]);


void
pid_plot_curvature(double current_phi, double desired_phi);


double
carmen_librlpid_compute_effort_signal(double current_phi, double desired_phi, double next_desired_phi, fann_type *steering_ann_input,
	struct fann *steering_ann, double v, double understeer_coeficient, double distance_between_front_and_rear_axles,
	double max_phi);


double
carmen_librlpid_compute_effort(double current_curvature, double desired_curvature, double delta_t);


#ifdef __cplusplus
}
#endif

#endif // CONTROL
