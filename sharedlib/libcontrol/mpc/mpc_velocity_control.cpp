#include <list>
#include <vector>
#include "mpc.h"


using namespace std;

#define DELTA_T (1.0 / 40.0) // 0.025 40 Htz
#define PREDICTION_HORIZON	0.4

enum
{
	STOP_CAR = 0,
	MOVE_CAR_FORWARD_ACCELERATING = 1,
	MOVE_CAR_FORWARD_DECCELERATING = 2,
	MOVE_CAR_BACKWARD_ACCELERATING = 3,
	MOVE_CAR_BACKWARD_DECCELERATING = 4
};


void
carmen_libmpc_get_optimized_velocity_effort_using_MPC(double *throttle_command, double *brake_command, int *gear_command,
		carmen_ackerman_motion_command_p current_motion_command_vector,	int nun_motion_commands, double desired_velocity, double current_velocity,
		double time_of_last_motion_command)
{
	static int mpc_state_controller = STOP_CAR;
	double g_brake_gap = 17.0;   // Q issu????

	if (fabs(desired_velocity) < 0.05)
	{
		desired_velocity = 0.0;
		mpc_state_controller = STOP_CAR;
	}

	if (mpc_state_controller == STOP_CAR)
	{
		*throttle_command = 0.0;
		*brake_command = 100.0;

		if ((desired_velocity > 0.0) && (current_velocity >= -0.05))
		{
			mpc_state_controller = MOVE_CAR_FORWARD_ACCELERATING;
			*gear_command = 1; // gear = Low // 2; //Drive gear (sharedlib/OpenJAUS/torc_docs/ByWire XGV User Manual v1.5.pdf page 67)
		}
		if ((desired_velocity < 0.0) && (current_velocity <= 0.05))
		{
			mpc_state_controller = MOVE_CAR_BACKWARD_ACCELERATING;
			*gear_command = 129; //Reverse gear (sharedlib/OpenJAUS/torc_docs/ByWire XGV User Manual v1.5.pdf page 67)
		}
	}

	if (mpc_state_controller == MOVE_CAR_FORWARD_ACCELERATING)
	{
		*throttle_command = u_t;
		*brake_command = g_brake_gap;

		if ((desired_velocity > 0.0) && (error_t < (0.0 - g_minimum_delta_velocity)) && (u_t <= 0.0))
		{
			error_t_1 = integral_t = integral_t_1 = 0.0;
			mpc_state_controller = MOVE_CAR_FORWARD_DECCELERATING;
		}
		if (desired_velocity <= 0.0)
			mpc_state_controller = STOP_CAR;
	}
	else if (mpc_state_controller == MOVE_CAR_FORWARD_DECCELERATING)
	{
		*throttle_command = 0.0;
		*brake_command = -u_t + g_brake_gap;

		if ((desired_velocity > 0.0) && (error_t > (0.0 + g_minimum_delta_velocity)) && u_t > 0.0)
		{
			error_t_1 = integral_t = integral_t_1 = 0.0;
			mpc_state_controller = MOVE_CAR_FORWARD_ACCELERATING;
		}
		if (desired_velocity <= 0.0)
			mpc_state_controller = STOP_CAR;
	}

	if (mpc_state_controller == MOVE_CAR_BACKWARD_ACCELERATING)
	{
//		*throttle_command = -u_t;
//		*brake_command = g_brake_gap;
//
//		if ((desired_velocity < 0.0) && (error_t > (0.0 + g_minimum_delta_velocity)))
//		{
//			error_t_1 = integral_t = integral_t_1 = 0.0;
//			mpc_state_controller = MOVE_CAR_BACKWARD_DECCELERATING;
//		}
//		if (desired_velocity >= 0.0)
//			mpc_state_controller = STOP_CAR;
	}
	else if (mpc_state_controller == MOVE_CAR_BACKWARD_DECCELERATING)
	{
//		*throttle_command = 0.0;
//		*brake_command = u_t + g_brake_gap;
//
//		if ((desired_velocity < 0.0) && (error_t < (0.0 - g_minimum_delta_velocity)))
//		{
//			error_t_1 = integral_t = integral_t_1 = 0.0;
//			mpc_state_controller = MOVE_CAR_BACKWARD_ACCELERATING;
//		}
//		if (desired_velocity >= 0.0)
//			mpc_state_controller = STOP_CAR;
	}

	*throttle_command = carmen_clamp(0.0, *throttle_command, 100.0);
	*brake_command = carmen_clamp(g_brake_gap, *brake_command, 100.0);
}
