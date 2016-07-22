/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <carmen/carmen.h>


enum
{
	STOP_CAR = 0,
	MOVE_CAR_FORWARD_ACCELERATING = 1,
	MOVE_CAR_FORWARD_DECCELERATING = 2,
	MOVE_CAR_BACKWARD_ACCELERATING = 3,
	MOVE_CAR_BACKWARD_DECCELERATING = 4
};


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


void
carmen_ford_escape_hybrid_steering_PID_controler(double *steering_command, 
	double atan_desired_curvature, double atan_current_curvature, double delta_t)
{
	// http://en.wikipedia.org/wiki/PID_controller -> Discrete implementation
	double 		error_t;		// error in time t
	static double 	error_t_1 = 0.0;	// error in time t-1
	static double 	integral_t = 0.0;	
	static double 	integral_t_1 = 0.0;	
	double		derivative_t;
	double 		u_t;			// u(t)	-> actuation in time t
	
	if (delta_t == 0.0)
		return;

	error_t = atan_desired_curvature - atan_current_curvature;
	integral_t = integral_t + error_t * delta_t;
	derivative_t = (error_t - error_t_1) / delta_t;
	
	u_t = 	g_steering_Kp * error_t +
		g_steering_Ki * integral_t +
		g_steering_Kd * derivative_t;

	error_t_1 = error_t;
	// Anti windup
	if ((u_t < -100.0) || (u_t > 100.0))
		integral_t = integral_t_1;
	integral_t_1 = integral_t;

	*steering_command = carmen_clamp(-100.0, u_t, 100.0);
//	fprintf(stdout, "STEERING (cc, dc, e, i, d, s, t): %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
//		atan_current_curvature, atan_desired_curvature, error_t, integral_t, derivative_t, *steering_command, carmen_get_time());
}


void
carmen_ford_escape_hybrid_velocity_PID_controler(double *throttle_command, double *brakes_command, int *gear_command, 
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
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//												//
// Inicializations										//
//												//
//////////////////////////////////////////////////////////////////////////////////////////////////


void 
carmen_ford_escape_hybrid_read_pid_parameters(int argc, char *argv[])
{
	int num_items;

	carmen_param_t param_list[]= 
	{
		{"robot", "PID_steering_kp", CARMEN_PARAM_DOUBLE, &g_steering_Kp, 0, NULL},
		{"robot", "PID_steering_ki", CARMEN_PARAM_DOUBLE, &g_steering_Ki, 0, NULL},
		{"robot", "PID_steering_kd", CARMEN_PARAM_DOUBLE, &g_steering_Kd, 0, NULL},
		{"robot", "PID_minimum_delta_velocity", CARMEN_PARAM_DOUBLE, &g_minimum_delta_velocity, 0, NULL},
		{"robot", "PID_velocity_forward_accelerating_Kp", CARMEN_PARAM_DOUBLE, &g_velocity_forward_accelerating_Kp, 0, NULL},
		{"robot", "PID_velocity_forward_accelerating_Ki", CARMEN_PARAM_DOUBLE, &g_velocity_forward_accelerating_Ki, 0, NULL},
		{"robot", "PID_velocity_forward_accelerating_Kd", CARMEN_PARAM_DOUBLE, &g_velocity_forward_accelerating_Kd, 0, NULL},
		{"robot", "PID_velocity_forward_deccelerating_Kp", CARMEN_PARAM_DOUBLE, &g_velocity_forward_deccelerating_Kp, 0, NULL},
		{"robot", "PID_velocity_forward_deccelerating_Ki", CARMEN_PARAM_DOUBLE, &g_velocity_forward_deccelerating_Ki, 0, NULL},
		{"robot", "PID_velocity_forward_deccelerating_Kd", CARMEN_PARAM_DOUBLE, &g_velocity_forward_deccelerating_Kd, 0, NULL},
		{"robot", "PID_velocity_backward_accelerating_Kp", CARMEN_PARAM_DOUBLE, &g_velocity_backward_accelerating_Kp, 0, NULL},
		{"robot", "PID_velocity_backward_accelerating_Ki", CARMEN_PARAM_DOUBLE, &g_velocity_backward_accelerating_Ki, 0, NULL},
		{"robot", "PID_velocity_backward_accelerating_Kd", CARMEN_PARAM_DOUBLE, &g_velocity_backward_accelerating_Kd, 0, NULL},
		{"robot", "PID_velocity_backward_deccelerating_Kp", CARMEN_PARAM_DOUBLE, &g_velocity_backward_deccelerating_Kp, 0, NULL},
		{"robot", "PID_velocity_backward_deccelerating_Ki", CARMEN_PARAM_DOUBLE, &g_velocity_backward_deccelerating_Ki, 0, NULL},
		{"robot", "PID_velocity_backward_deccelerating_Kd", CARMEN_PARAM_DOUBLE, &g_velocity_backward_deccelerating_Kd, 0, NULL},
		{"robot", "PID_velocity_brake_gap", CARMEN_PARAM_DOUBLE, &g_brake_gap, 0, NULL},
	};

	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}
