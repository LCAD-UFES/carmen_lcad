#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif


#include "pid/pid.h"
#include "rlpid/rlpid.h"
#include <car_model.h>


double
carmen_libmpc_get_optimized_steering_effort_using_MPC(double atan_current_curvature,
		carmen_ackerman_motion_command_p current_motion_command_vector, int nun_motion_commands,
		double v, double yp, double time_of_last_motion_command,
		carmen_robot_ackerman_config_t *robot_config, int initialize_neural_networks);

#ifdef __cplusplus
}
#endif

#endif // CONTROL
