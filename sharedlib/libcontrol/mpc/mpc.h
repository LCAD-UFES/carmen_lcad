#ifndef MPC_H
#define MPC_H

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
	double k1;
	double k2;
	double k3;
	double k4;
} EFFORT_SPLINE_DESCRIPTOR;


typedef struct
{
	carmen_ackerman_motion_command_p motion_commands_vector;
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
} PARAMS;


double
carmen_libmpc_get_optimized_steering_effort_using_MPC(double atan_current_curvature,
		carmen_ackerman_motion_command_p current_motion_command_vector,
		int nun_motion_commands, double v, double yp, double time_of_last_motion_command,
		double understeer_coeficient, double distance_between_front_and_rear_axles, double max_phi,
		int initialize_neural_networks);

int
libmpc_stiction_simulation(double effort, double v);

double
libmpc_stiction_correction(double current_phi, double desired_phi, double effort, double v);


#ifdef __cplusplus
}
#endif

#endif // PID_H
