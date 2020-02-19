#include <carmen/carmen.h>
#include <carmen/simulator_ackerman.h>
#include "pid.h"
#include "../control.h"
#include <list>


using namespace std;


enum
{
	STOP_CAR = 0,
	MOVE_CAR_FORWARD_ACCELERATING = 1,
	MOVE_CAR_FORWARD_DECCELERATING = 2,
	MOVE_CAR_BACKWARD_ACCELERATING = 3,
	MOVE_CAR_BACKWARD_DECCELERATING = 4
};


#define min_fuzzy_v 4.17 // = 20km/h Parameters start variation from this velocity
#define max_fuzzy_v 9.72 // = 45km/h Parameters stop variation from twice this velocity


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
pid_plot_phi(double current_phi, double desired_phi, double y_range, char* title)
{
	#define PAST_SIZE 300
	static list<double> cphi;
	static list<double> dphi;
	static list<double> timestamp;
	static bool first_time = true;
	static double first_timestamp;
	static FILE *gnuplot_pipe;
	list<double>::iterator itc;
	list<double>::iterator itd;
	list<double>::iterator itt;

	double t = carmen_get_time();
	if (first_time)
	{
		first_timestamp = t;
		first_time = false;
		gnuplot_pipe = popen("gnuplot", "w"); //("gnuplot -persist", "w") to keep last plot after program closes
		fprintf(gnuplot_pipe, "set xrange [0:PAST_SIZE/30]\n set yrange [-%lf:%lf]\n", y_range, y_range);
	}

	cphi.push_front(current_phi);
	dphi.push_front(desired_phi);
	timestamp.push_front(t - first_timestamp);

	while(cphi.size() > PAST_SIZE)
	{
		cphi.pop_back();
		dphi.pop_back();
		timestamp.pop_back();
	}


	FILE *gnuplot_data_file = fopen("gnuplot_phi_data.txt", "w");

	for (itc = cphi.begin(), itd = dphi.begin(), itt = timestamp.begin(); itc != cphi.end(); itc++, itd++, itt++)
		fprintf(gnuplot_data_file, "%lf %lf %lf\n", *itt - timestamp.back(), *itc, *itd);

	fclose(gnuplot_data_file);


	fprintf(gnuplot_pipe, "plot "
			"'./gnuplot_phi_data.txt' using 1:2 with lines title 'C%s', './gnuplot_phi_data.txt' using 1:3 with lines title 'D%s'\n", title, title);

	fflush(gnuplot_pipe);
}


void
pid_plot_velocity(double current_vel, double desired_vel, double y_range, char* title)
{
	#define PAST_SIZE 300
	static list<double> cvel;
	static list<double> dvel;
	static list<double> timestamp;
	static bool first_time = true;
	static double first_timestamp;
	static FILE *gnuplot_pipe;
	list<double>::iterator itc;
	list<double>::iterator itd;
	list<double>::iterator itt;

	double t = carmen_get_time();
	if (first_time)
	{
		first_timestamp = t;
		first_time = false;
		gnuplot_pipe = popen("gnuplot", "w"); //("gnuplot -persist", "w") to keep last plot after program closes
		fprintf(gnuplot_pipe, "set xrange [0:PAST_SIZE/30]\n set yrange [-%lf:%lf]\n", y_range, y_range);
	}

	cvel.push_front(current_vel);
	dvel.push_front(desired_vel);
	timestamp.push_front(t - first_timestamp);

	while(cvel.size() > PAST_SIZE)
	{
		cvel.pop_back();
		dvel.pop_back();
		timestamp.pop_back();
	}


	FILE *gnuplot_data_file = fopen("gnuplot_velocity_data.txt", "w");

	for (itc = cvel.begin(), itd = dvel.begin(), itt = timestamp.begin(); itc != cvel.end(); itc++, itd++, itt++)
		fprintf(gnuplot_data_file, "%lf %lf %lf\n", *itt - timestamp.back(), *itc, *itd);

	fclose(gnuplot_data_file);


	fprintf(gnuplot_pipe, "plot "
			"'./gnuplot_velocity_data.txt' using 1:2 with lines title 'C%s', './gnuplot_velocity_data.txt' using 1:3 with lines title 'D%s'\n", title, title);

	fflush(gnuplot_pipe);
}


double
carmen_libpid_steering_PID_controler(double atan_desired_curvature, double atan_current_curvature, double delta_t_old __attribute__ ((unused)),
		int manual_override)
{
	// http://en.wikipedia.org/wiki/PID_controller -> Discrete implementation
	static double 	error_t_1 = 0.0;	// error in time t-1
	static double 	integral_t = 0.0;
	static double 	integral_t_1 = 0.0;
	static double 	u_t = 0.0;			// u(t)	-> actuation in time t
	static double	previous_t = 0.0;

	if (previous_t == 0.0)
	{
		previous_t = carmen_get_time();
		return (0.0);
	}
	double t = carmen_get_time();
	double delta_t = t - previous_t;

	if (delta_t < (0.7 * (1.0 / 40.0)))
		return (u_t);

	double error_t = atan_desired_curvature - atan_current_curvature;

	if (manual_override == 0)
		integral_t = integral_t + error_t * delta_t;
	else
		integral_t = integral_t_1 = 0.0;

//	double derivative_t = (error_t - error_t_1) / delta_t;
	double derivative_t = (error_t - error_t_1) / delta_t;

	u_t = g_steering_Kp * error_t +
		  g_steering_Ki * integral_t +
		  g_steering_Kd * derivative_t;

	error_t_1 = error_t;

	// Anti windup
	if ((u_t < -100.0) || (u_t > 100.0))
		integral_t = integral_t_1;
	integral_t_1 = integral_t;

	previous_t = t;

//	if (atan_desired_curvature != 0.0)
//		u_t = 13.0;

	u_t = carmen_clamp(-100.0, u_t, 100.0);

//	fprintf(stdout, "STEERING (cc, dc, e, i, d, s): %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
//		atan_current_curvature, atan_desired_curvature, error_t, integral_t, derivative_t, u_t, t);
//	fflush(stdout);

	return u_t;
}


double
carmen_libpid_steering_PID_controler_FUZZY(double atan_desired_curvature, double atan_current_curvature, double delta_t_old __attribute__ ((unused)),
		int manual_override, double v)
{
	static double 	error_t_1 = 0.0;	// error in time t-1
	static double 	integral_t = 0.0;
	static double 	integral_t_1 = 0.0;
	static double 	u_t = 0.0;			// u(t)	-> actuation in time t
	static double	previous_t = 0.0;
	double factor = 0.0, kp = 0.0, ki = 0.0, kd = 0.0;

	if (previous_t == 0.0)
	{
		previous_t = carmen_get_time();
		return (0.0);
	}
	double t = carmen_get_time();
	double delta_t = t - previous_t;

	if (delta_t < (0.7 * (1.0 / 40.0)))
		return (u_t);

	double error_t = atan_desired_curvature - atan_current_curvature;

	if (manual_override == 0)
		integral_t = integral_t + error_t * delta_t;
	else
		integral_t = integral_t_1 = 0.0;

	double derivative_t = (error_t - error_t_1) / delta_t;

	////////////////  FUZZY  ////////////////
	//1480.9  -  689.4   =  791.5
	//6985.4  -  2008.7  =  4976.7
	//81.45   -  30.8    =  50.65
	factor = carmen_clamp(0.0, (v - min_fuzzy_v) / (max_fuzzy_v - min_fuzzy_v), 1.0); // The PID parameters stabilize when the velocity is max_fuzzy_v

	kp = g_steering_Kp + factor * 791.5;
	ki = g_steering_Ki + factor * 4976.7;
	kd = g_steering_Kd + factor * 50.65;

	//printf("v %lf kp %lf ki %lf kd %lf\n", v, kp, ki, kd);

	u_t = (kp * error_t)  +  (ki * integral_t)  +  (kd * derivative_t);

	error_t_1 = error_t;

	// Anti windup
	if ((u_t < -100.0) || (u_t > 100.0))
		integral_t = integral_t_1;
	integral_t_1 = integral_t;

	previous_t = t;

	u_t = carmen_clamp(-100.0, u_t, 100.0);

//	fprintf(stdout, "STEERING (cc, dc, e, i, d, s): %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
//		atan_current_curvature, atan_desired_curvature, error_t, integral_t, derivative_t, u_t, t);
//	fflush(stdout);

	return u_t;
}


double
carmen_libpid_steering_PID_controler_new(double atan_desired_curvature, double atan_current_curvature, double delta_t_old __attribute__ ((unused)))
{
	// http://en.wikipedia.org/wiki/PID_controller -> Discrete implementation
	static double 	error_t_1 = 0.0;	// error in time t-1
	static double 	integral_t = 0.0;
	static double 	integral_t_1 = 0.0;
	static double 	u_t = 0.0;			// u(t)	-> actuation in time t
	static double	previous_t = 0.0;
#define DERIVATIVE_HISTORY_SIZE 3
	static double	previous_derivatives[DERIVATIVE_HISTORY_SIZE];
//	static double	initial_t = 0.0;

	if (previous_t == 0.0)
	{
//		initial_t = previous_t = carmen_get_time();
		for (int i = 0; i < DERIVATIVE_HISTORY_SIZE; i++)
			previous_derivatives[i] = 0.0;
		return (0.0);
	}
	double t = carmen_get_time();
	double delta_t = t - previous_t;

	if (delta_t < (0.7 * (1.0 / 40.0)))
		return (u_t);

	double error_t = atan_desired_curvature - atan_current_curvature;

	integral_t = integral_t + error_t * delta_t;

//	double derivative_t = (error_t - error_t_1) / delta_t;
	for (int i = DERIVATIVE_HISTORY_SIZE - 2; i >= 0; i--)
		previous_derivatives[i + 1] = previous_derivatives[i];
	previous_derivatives[0] = (error_t - error_t_1) / delta_t;
	double derivative_t = 0.0;
	for (int i = 0; i < DERIVATIVE_HISTORY_SIZE; i++)
		derivative_t += previous_derivatives[i];
	derivative_t /= (double) DERIVATIVE_HISTORY_SIZE;

	//
	double Kc = g_steering_Kp;
	double Ti = g_steering_Kp / g_steering_Ki;
	double Td = g_steering_Kp * g_steering_Kd;

//	u_t = atan_desired_curvature * 300.0;

	u_t = Kc * error_t + (Kc / Ti) * integral_t + Kc * Td * derivative_t;

	error_t_1 = error_t;

	// Anti windup
	if ((u_t < -100.0) || (u_t > 100.0))
		integral_t = integral_t_1;
	integral_t_1 = integral_t;

	previous_t = t;

	u_t = carmen_clamp(-100.0, u_t, 100.0);

//	fprintf(stdout, "STEERING (cc, dc, e, i, d, s): %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
//		atan_current_curvature, atan_desired_curvature, error_t, integral_t, derivative_t, u_t, t - initial_t);
//	fflush(stdout);

	return u_t;
}


void
carmen_libpid_velocity_PID_controler(double *throttle_command, double *brakes_command, int *gear_command,
										double desired_velocity, double current_velocity, double delta_t_old __attribute__ ((unused)),
										int manual_override)
{
	// http://en.wikipedia.org/wiki/PID_controller -> Discrete implementation
	static double 	error_t_1 = 0.0;	// error in time t-1
	static double 	integral_t = 0.0;
	static double 	integral_t_1 = 0.0;
	static double 	u_t = 0.0;			// u(t)	-> actuation in time t
	static double	previous_t = 0.0;

	if (previous_t == 0.0)
	{
		previous_t = carmen_get_time();
		return;
	}
	double t = carmen_get_time();
	double delta_t = t - previous_t;

	if (fabs(desired_velocity) < 0.05)
	{
		desired_velocity = 0.0;
		g_velocity_PID_controler_state = STOP_CAR;
	}

	double error_t = desired_velocity - current_velocity;
	if (manual_override == 0)
		integral_t = integral_t + error_t * delta_t;
	else
		integral_t = 0.0;

	double derivative_t = (error_t - error_t_1) / delta_t;

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

	previous_t = t;

	*throttle_command = carmen_clamp(0.0, *throttle_command, 100.0);
	*brakes_command = carmen_clamp(g_brake_gap, *brakes_command, 100.0);

//	fprintf(stdout, "VELOCITY (st, cv, dv, e, t, b, i, d, ts): %d, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
//		g_velocity_PID_controler_state, current_velocity, desired_velocity, error_t,
//		*throttle_command, *brakes_command,
//		integral_t, derivative_t, carmen_get_time());
//	fflush(stdout);
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//												//
// Inicializations								//
//												//
//////////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_libpid_read_PID_parameters(int argc, char *argv[])
{
	int num_items;

	carmen_param_t param_list[]=
	{
		{(char *)"robot", (char *)"PID_steering_kp", CARMEN_PARAM_DOUBLE, &g_steering_Kp, 0, NULL},
		{(char *)"robot", (char *)"PID_steering_ki", CARMEN_PARAM_DOUBLE, &g_steering_Ki, 0, NULL},
		{(char *)"robot", (char *)"PID_steering_kd", CARMEN_PARAM_DOUBLE, &g_steering_Kd, 0, NULL},
		{(char *)"robot", (char *)"PID_minimum_delta_velocity", CARMEN_PARAM_DOUBLE, &g_minimum_delta_velocity, 0, NULL},
		{(char *)"robot", (char *)"PID_velocity_forward_accelerating_Kp", CARMEN_PARAM_DOUBLE, &g_velocity_forward_accelerating_Kp, 0, NULL},
		{(char *)"robot", (char *)"PID_velocity_forward_accelerating_Ki", CARMEN_PARAM_DOUBLE, &g_velocity_forward_accelerating_Ki, 0, NULL},
		{(char *)"robot", (char *)"PID_velocity_forward_accelerating_Kd", CARMEN_PARAM_DOUBLE, &g_velocity_forward_accelerating_Kd, 0, NULL},
		{(char *)"robot", (char *)"PID_velocity_forward_deccelerating_Kp", CARMEN_PARAM_DOUBLE, &g_velocity_forward_deccelerating_Kp, 0, NULL},
		{(char *)"robot", (char *)"PID_velocity_forward_deccelerating_Ki", CARMEN_PARAM_DOUBLE, &g_velocity_forward_deccelerating_Ki, 0, NULL},
		{(char *)"robot", (char *)"PID_velocity_forward_deccelerating_Kd", CARMEN_PARAM_DOUBLE, &g_velocity_forward_deccelerating_Kd, 0, NULL},
		{(char *)"robot", (char *)"PID_velocity_backward_accelerating_Kp", CARMEN_PARAM_DOUBLE, &g_velocity_backward_accelerating_Kp, 0, NULL},
		{(char *)"robot", (char *)"PID_velocity_backward_accelerating_Ki", CARMEN_PARAM_DOUBLE, &g_velocity_backward_accelerating_Ki, 0, NULL},
		{(char *)"robot", (char *)"PID_velocity_backward_accelerating_Kd", CARMEN_PARAM_DOUBLE, &g_velocity_backward_accelerating_Kd, 0, NULL},
		{(char *)"robot", (char *)"PID_velocity_backward_deccelerating_Kp", CARMEN_PARAM_DOUBLE, &g_velocity_backward_deccelerating_Kp, 0, NULL},
		{(char *)"robot", (char *)"PID_velocity_backward_deccelerating_Ki", CARMEN_PARAM_DOUBLE, &g_velocity_backward_deccelerating_Ki, 0, NULL},
		{(char *)"robot", (char *)"PID_velocity_backward_deccelerating_Kd", CARMEN_PARAM_DOUBLE, &g_velocity_backward_deccelerating_Kd, 0, NULL},
		{(char *)"robot", (char *)"PID_velocity_brake_gap", CARMEN_PARAM_DOUBLE, &g_brake_gap, 0, NULL},
	};

	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}
