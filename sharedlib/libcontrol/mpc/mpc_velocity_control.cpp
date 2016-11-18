#include <list>
#include <vector>
#include "mpc.h"


using namespace std;


enum
{
	STOP_CAR = 0,
	MOVE_CAR_FORWARD_ACCELERATING = 1,
	MOVE_CAR_FORWARD_DECCELERATING = 2,
	MOVE_CAR_BACKWARD_ACCELERATING = 3,
	MOVE_CAR_BACKWARD_DECCELERATING = 4
};


bool
init_velocity_mpc(PARAMS &params)
{
	static bool first_time = true;

	if (first_time)
	{
		params.velocity_ann = fann_create_from_file("velocity_ann.net");
		if (params.velocity_ann == NULL)
		{
			printf("Error: Could not create velocity_ann\n");
			exit(1);
		}
		carmen_libcarneuralmodel_init_velocity_ann_input(velocity_ann_input);

		params.velocity_descriptors.k1 = 0.0;
		params.velocity_descriptors.k2 = 0.0;
		params.velocity_descriptors.k3 = 0.0;
		params.velocity_descriptors.k4 = 0.0;

		params.velocity_error = 0.0;
		params.previous_velocity_k1 = 0.0;


//		if (save_and_plot)
//			open_file_to_save_plot(true);

		first_time = false;
		return (true);
	}

	return (false);
}



void
carmen_libmpc_get_optimized_velocity_effort_using_MPC(PARAMS &params)
{
	if (init_velocity_mpc(params))
		return;

	double effort = seed.k1;

	// Calcula o dk do proximo ciclo
	double Cxk = car_model(effort, atan_current_curvature, params.v, params.steering_ann_input, &params);
	params.velocity_error = params. - Cxk;
	params.previous_k1 = effort;

	seed = get_optimized_effort(&params, seed);
	static int mpc_state_controller = STOP_CAR;

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
