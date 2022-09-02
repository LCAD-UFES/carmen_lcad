#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif


//#define PLOT    		// Plot Phi
//#define PLOT_VELOCITY
//#define PRINT			// Print Debug
//#define SAVE			// Save Ploted Values

#include <fann.h>
#include <fann_data.h>
#include <floatfann.h>
#include <fann_train.h>


double
carmen_libmpc_get_optimized_steering_effort_using_MPC(double atan_current_curvature,
		carmen_robot_and_trailers_motion_command_t *current_motion_command_vector, int nun_motion_commands,
		double v, double yp, double time_of_last_motion_command,
		carmen_robot_ackerman_config_t *robot_config, int initialize_neural_networks);


void
carmen_libmpc_compute_velocity_effort(double *throttle_command, double *brake_command, int *gear_command,
		carmen_robot_and_trailers_motion_command_t *current_motion_command_vector, int nun_motion_commands,
		double current_velocity, double time_of_last_motion_command, carmen_robot_ackerman_config_t *robot_config);

double
carmen_libmpc_get_optimized_steering_effort_using_MPC_position_control(double atan_current_curvature,
		carmen_robot_and_trailers_motion_command_t *current_motion_command_vector,	int nun_motion_commands,
		double v, double yp, double time_of_last_motion_command,
		carmen_robot_ackerman_config_t *robot_config,
		carmen_localize_ackerman_globalpos_message global_pos, int initialize_neural_networks);

double
carmen_libpid_steering_PID_controler(double atan_desired_curvature, double atan_current_curvature, double plan_size, int manual_override);


double carmen_libpid_steering_PID_controler_FUZZY(double atan_desired_curvature, double atan_current_curvature, double delta_t_old __attribute__ ((unused)),
		int manual_override, double v);


void
carmen_libpid_velocity_PID_controler(double *throttle_command, double *brakes_command, int *gear_command,
											double desired_velocity, double current_velocity, double delta_t, int manual_override);


void
carmen_libpid_read_PID_parameters(int argc, char *argv[]);


void
pid_plot_phi(double current_phi, double desired_phi, double y_range, char *title);

void
pid_plot_velocity(double current_phi, double desired_phi, double y_range, char* title);


double
carmen_librlpid_compute_effort_signal(double current_phi, double desired_phi, double next_desired_phi, fann_type *steering_ann_input,
	struct fann *steering_ann, double v, double understeer_coeficient, double distance_between_front_and_rear_axles, double max_phi);


double
carmen_librlpid_compute_effort(double current_curvature, double desired_curvature, double delta_t);


double
carmen_librlpid_compute_effort_new(double current_curvature, double desired_curvature, double delta_t);


#ifdef __cplusplus
}
#endif

#endif // CONTROL
