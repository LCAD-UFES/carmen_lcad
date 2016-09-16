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
	double atan_desired_curvature;
	double v;
	double understeer_coeficient;
	double distance_rear_axles;
} PARAMS;


double
carmen_libmpc_get_optimized_steering_effort_using_MPC(double atan_current_curvature, double atan_desired_curvature,
											fann_type *steering_ann_input, struct fann *steering_ann,
											carmen_simulator_ackerman_config_t *simulator_config);

double
carmen_libmpc_get_optimized_steering_effort_using_MPCc(double atan_current_curvature, double atan_desired_curvature,
											fann_type *steering_ann_input, struct fann *steering_ann,
											ford_escape_hybrid_config_t *simulator_config);

#ifdef __cplusplus
}
#endif

#endif // PID_H
