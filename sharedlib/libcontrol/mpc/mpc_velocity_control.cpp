#include "mpc.h"


using namespace std;

// Global variables
static double g_steering_Kp;
static double g_steering_Ki;
static double g_steering_Kd;

static int g_velocity_PID_controler_state = STOP_CAR;
static double g_minimum_delta_velocity;
static double g_velocity_forward_accelerating_Kp;
static double g_velocity_forward_accelerating_Ki;
static double g_velocity_forward_accelerating_Kd;
static double g_velocity_forward_deccelerating_Kp;
static double g_velocity_forward_deccelerating_Ki;
static double g_velocity_forward_deccelerating_Kd;
static double g_velocity_backward_accelerating_Kp;
static double g_velocity_backward_accelerating_Ki;
static double g_velocity_backward_accelerating_Kd;
static double g_velocity_backward_deccelerating_Kp;
static double g_velocity_backward_deccelerating_Ki;
static double g_velocity_backward_deccelerating_Kd;
static double g_brake_gap;

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

		first_time = false;
		return (true);
	}

	return (false);
}


void
carmen_libpid_velocity_PID_controler(double *throttle_command, double *brakes_command, int *gear_command,
										double desired_velocity, double current_velocity, double delta_t)
{
	// http://en.wikipedia.org/wiki/PID_controller -> Discrete implementation
	double 		error_t;		// error in time t
	static double 	error_t_1 = 0.0;	// error in time t-1
	static double 	integral_t = 0.0;
	static double 	integral_t_1 = 0.0;
	double		derivative_t;
	double 		u_t = 0.0;		// u(t)	-> actuation in time t

	if (delta_t == 0.0)
		return;

	if (fabs(desired_velocity) < 0.05)
	{
		desired_velocity = 0.0;
		g_velocity_PID_controler_state = STOP_CAR;
	}

	error_t = desired_velocity - current_velocity;
	integral_t = integral_t + error_t * delta_t;
	derivative_t = (error_t - error_t_1) / delta_t;

	if (g_velocity_PID_controler_state == STOP_CAR)
	{
		error_t_1 = integral_t = integral_t_1 = 0.0;

		*throttle_command = 0.0;
		*brakes_command = 100.0;

		if ((desired_velocity > 0.0) && (current_velocity >= -0.05))
		{
			g_velocity_PID_controler_state = MOVE_CAR_FORWARD_ACCELERATING;
			*gear_command = 1; // gear = Low // 2; //Drive gear (sharedlib/OpenJAUS/torc_docs/ByWire XGV User Manual v1.5.pdf page 67)
		}
		if ((desired_velocity < 0.0) && (current_velocity <= 0.05))
		{
			g_velocity_PID_controler_state = MOVE_CAR_BACKWARD_ACCELERATING;
			*gear_command = 129; //Reverse gear (sharedlib/OpenJAUS/torc_docs/ByWire XGV User Manual v1.5.pdf page 67)
		}
	}

	if (g_velocity_PID_controler_state == MOVE_CAR_FORWARD_ACCELERATING)
	{
		u_t = 	g_velocity_forward_accelerating_Kp * error_t +
			g_velocity_forward_accelerating_Ki * integral_t +
			g_velocity_forward_accelerating_Kd * derivative_t;

		*throttle_command = u_t;
		*brakes_command = g_brake_gap;

		if ((desired_velocity > 0.0) && (error_t < (0.0 - g_minimum_delta_velocity)) && (u_t <= 0.0))
		{
			error_t_1 = integral_t = integral_t_1 = 0.0;
			g_velocity_PID_controler_state = MOVE_CAR_FORWARD_DECCELERATING;
		}
		if (desired_velocity <= 0.0)
			g_velocity_PID_controler_state = STOP_CAR;
	}
	else if (g_velocity_PID_controler_state == MOVE_CAR_FORWARD_DECCELERATING)
	{
		u_t = 	g_velocity_forward_deccelerating_Kp * error_t +
			g_velocity_forward_deccelerating_Ki * integral_t +
			g_velocity_forward_deccelerating_Kd * derivative_t;

		*throttle_command = 0.0;
		*brakes_command = -u_t + g_brake_gap;

		if ((desired_velocity > 0.0) && (error_t > (0.0 + g_minimum_delta_velocity)) && u_t > 0.0)
		{
			error_t_1 = integral_t = integral_t_1 = 0.0;
			g_velocity_PID_controler_state = MOVE_CAR_FORWARD_ACCELERATING;
		}
		if (desired_velocity <= 0.0)
			g_velocity_PID_controler_state = STOP_CAR;
	}

	if (g_velocity_PID_controler_state == MOVE_CAR_BACKWARD_ACCELERATING)
	{
		u_t = 	g_velocity_backward_accelerating_Kp * error_t +
			g_velocity_backward_accelerating_Ki * integral_t +
			g_velocity_backward_accelerating_Kd * derivative_t;

		*throttle_command = -u_t;
		*brakes_command = g_brake_gap;

		if ((desired_velocity < 0.0) && (error_t > (0.0 + g_minimum_delta_velocity)))
		{
			error_t_1 = integral_t = integral_t_1 = 0.0;
			g_velocity_PID_controler_state = MOVE_CAR_BACKWARD_DECCELERATING;
		}
		if (desired_velocity >= 0.0)
			g_velocity_PID_controler_state = STOP_CAR;
	}
	else if (g_velocity_PID_controler_state == MOVE_CAR_BACKWARD_DECCELERATING)
	{
		u_t = 	g_velocity_backward_deccelerating_Kp * error_t +
			g_velocity_backward_deccelerating_Ki * integral_t +
			g_velocity_backward_deccelerating_Kd * derivative_t;

		*throttle_command = 0.0;
		*brakes_command = u_t + g_brake_gap;

		if ((desired_velocity < 0.0) && (error_t < (0.0 - g_minimum_delta_velocity)))
		{
			error_t_1 = integral_t = integral_t_1 = 0.0;
			g_velocity_PID_controler_state = MOVE_CAR_BACKWARD_ACCELERATING;
		}
		if (desired_velocity >= 0.0)
			g_velocity_PID_controler_state = STOP_CAR;
	}

	error_t_1 = error_t;
	// Anti windup
	if ((*throttle_command < 0.0) || (*throttle_command > 100.0) ||
	    (*brakes_command < g_brake_gap) || (*brakes_command > 100.0))
		integral_t = integral_t_1;
	integral_t_1 = integral_t;

	*throttle_command = carmen_clamp(0.0, *throttle_command, 100.0);
	*brakes_command = carmen_clamp(g_brake_gap, *brakes_command, 100.0);
//	fprintf(stdout, "VELOCITY (st, cv, dv, e, t, b, i, d, ts): %d, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
//		g_velocity_PID_controler_state, current_velocity, desired_velocity, error_t,
//		*throttle_command, *brakes_command,
//		integral_t, derivative_t, carmen_get_time());
//	fflush(stdout);
}



void
carmen_libmpc_get_optimized_velocity_effort_using_MPC(PARAMS &params)
{
	if (init_velocity_mpc(params))
		return;

	get_optimized_effort(&params, seed);

}
