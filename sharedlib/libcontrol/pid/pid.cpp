#include <carmen/carmen.h>
#include <carmen/simulator_ackerman.h>
#include "pid.h"
#include "../control.h"
#include <list>
#include <vector>
#define PRINT			// Print Debug

#define VEHICLE_WITH_BRAKES 	0

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

#define NEAR_ZERO_V	0.01

#define ROBOT_ID_FORD_ESCAPE 	0
#define ROBOT_ID_ECOTECH4 		1
#define ROBOT_ID_MPW700 		2
#define ROBOT_ID_ASTRU 			3
#define ROBOT_ID_BUGGY 			4


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
static double g_max_brake_effort;
static double g_maximum_steering_command_rate;
static double g_throttle_gap;

static double g_fuzzy_factor = 0.5;

//static double g_v_error_multiplier = 0.0; // quando igual a 1.0, o erro de v eh multiplicado por 2 qundo v eh igual a g_target_velocity (error_multiplier = 1.0 + g_v_error_multiplier * fabs(current_velocity / g_target_velocity))
//static double g_target_velocity = 5.55;

static int robot_model_id = 0;
std::vector<double> current_steer_angle_vector, desired_steer_angle_vector, error_vector, timestamp_vector;

/*#ifdef PRINT
extern carmen_behavior_selector_low_level_state_t behavior_selector_low_level_state;
#endif*/

static double g_v = 0.0;


void
pid_plot_phi(double current_phi, double desired_phi, double y_range, char *title)
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
	return; // Para nao plotar

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
carmen_libpid_steering_PID_controler_publish_data(steering_pid_data_message * msg, double atan_desired_curvature, double atan_current_curvature, double plan_size,
		int manual_override, double kp, double kd, double ki)
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

//	if (delta_t < (0.7 * (1.0 / 40.0)))
//		return (u_t);

	double desired_curvature = tan(atan_desired_curvature);
	double current_curvature = tan(atan_current_curvature);
	double delta_curvature = fabs(desired_curvature - current_curvature);
	double command_curvature_signal = (current_curvature < desired_curvature) ? 1.0 : -1.0;
	double max_curvature_change = carmen_clamp(0.05, (0.6 * fabs(plan_size)), 1.2) * g_maximum_steering_command_rate * delta_t;

	double achieved_curvature = current_curvature + command_curvature_signal * fmin(delta_curvature, max_curvature_change);
	atan_desired_curvature = atan(achieved_curvature);

	double error_t = atan_desired_curvature - atan_current_curvature;

	if (manual_override == 0)
		integral_t = integral_t + error_t * delta_t;
	else
		integral_t = integral_t_1 = 0.0;

	double derivative_t = (error_t - error_t_1) / delta_t;

	u_t = g_steering_Kp * error_t +
		  g_steering_Ki * integral_t +
		  g_steering_Kd * derivative_t;

	
	u_t = kp * error_t +
		  ki * integral_t +
		  kd * derivative_t;

	error_t_1 = error_t;

	// Anti windup
	if ((u_t < -100.0) || (u_t > 100.0))
		integral_t = integral_t_1;
	integral_t_1 = integral_t;

	previous_t = t;

	u_t = carmen_clamp(-100.0, u_t, 100.0);

	msg->atan_current_curvature = atan_current_curvature;
	msg->atan_desired_curvature = atan_desired_curvature;
	msg->derivative_t = derivative_t;
	msg->error_t = error_t;
	msg->integral_t = integral_t;
	msg->effort = u_t;

#ifdef PRINT
	fprintf(stdout, "STEERING (cc, dc, e, i, d, s): %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
		atan_current_curvature, atan_desired_curvature, error_t, integral_t, derivative_t, u_t, t);
	fflush(stdout);
#endif

	return u_t;
}


double
carmen_libpid_steering_PID_controler(double atan_desired_curvature, double atan_current_curvature, double plan_size, int tune_pid_mode,
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

//	if (delta_t < (0.7 * (1.0 / 40.0)))
//		return (u_t);

	double desired_curvature = tan(atan_desired_curvature);
	double current_curvature = tan(atan_current_curvature);
	double delta_curvature = fabs(desired_curvature - current_curvature);
	double command_curvature_signal = (current_curvature < desired_curvature) ? 1.0 : -1.0;
	double max_curvature_change = carmen_clamp(0.15, plan_size, 1.0) * 1.2 * g_maximum_steering_command_rate * delta_t;

	double achieved_curvature;
	double fuzzy_factor = (fabs(g_v) * (g_fuzzy_factor / 8.33) + 1.0);
	if (tune_pid_mode)
	{
		achieved_curvature = current_curvature + command_curvature_signal * delta_curvature;
		fuzzy_factor = 1.0;
	}
	else
		achieved_curvature = current_curvature + command_curvature_signal * fmin(delta_curvature, max_curvature_change);

	atan_desired_curvature = atan(achieved_curvature);

//	printf("g_maximum_steering_command_rate %lf, v %lf, plan_size %lf, fuzzy_factor %lf\n", g_maximum_steering_command_rate, g_v, plan_size, fuzzy_factor);
	double error_t = (atan_desired_curvature - atan_current_curvature) * fuzzy_factor;

	if (manual_override == 0)
		integral_t = integral_t + error_t * delta_t;
	else
		integral_t = integral_t_1 = 0.0;

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

	u_t = carmen_clamp(-100.0, u_t, 100.0);

#ifdef PRINT
	fprintf(stdout, "STEERING (cc, dc, e, i, d, s): %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
		atan_current_curvature, atan_desired_curvature, error_t, integral_t, derivative_t, u_t, t);
	fflush(stdout);
#endif

	return u_t;
}

void
make_plot_pid_automatic(double steer_kp, double steer_kd, double steer_ki, double error_sum)
{

	FILE *gnuplot_pipe = NULL;
	double y_range = 0.55;
	static int iter = 0;
	if (gnuplot_pipe != NULL) {
	    fclose(gnuplot_pipe);
	}
	if(error_vector.empty())
	{
		return;
	}
	FILE *arquivo_teste = fopen("/home/lume/carmen_lcad/src/ford_escape_hybrid/steer_pid.txt", "w");
	for(long unsigned int i = 0 ; i < current_steer_angle_vector.size(); i++)
		fprintf(arquivo_teste, "%lf %lf %lf %ld\n",
			current_steer_angle_vector[i], desired_steer_angle_vector[i], error_vector[i], i);
	//fflush(arquivo_teste);
	fclose(arquivo_teste);
	printf("SAI DO PLOTTT\n");
	gnuplot_pipe = popen("gnuplot", "w"); // -persist to keep last plot after program closes
	fprintf(gnuplot_pipe, "set terminal png size 1000,1000\n");
	fprintf(gnuplot_pipe, "set output '/home/lume/carmen_lcad/src/ford_escape_hybrid/%d.png'\n", iter);
	fprintf(gnuplot_pipe, "set title 'KP %lf KI %lf KD %lf Err %lf'\n", steer_kp, steer_ki , steer_kd, error_sum);
	fprintf(gnuplot_pipe, "set yrange [-%lf:%lf]\n", y_range, y_range);
	fprintf(gnuplot_pipe, "plot "
			"'/home/lume/carmen_lcad/src/ford_escape_hybrid/steer_pid.txt' using 4:1 with lines title 'current steer angle',"
			"'/home/lume/carmen_lcad/src/ford_escape_hybrid/steer_pid.txt' using 4:2 with lines title 'desired steer angle', "
			"'/home/lume/carmen_lcad/src/ford_escape_hybrid/steer_pid.txt' using 4:3 with lines title 'error'\n");
	fflush(gnuplot_pipe);
	iter++;
    fclose(gnuplot_pipe);

}

double
carmen_libpid_steering_PID_controler_FUZZY_publish_data(steering_pid_data_message *msg, steering_pid_error_message *msg_error , double atan_desired_curvature, double atan_current_curvature, double delta_t_old __attribute__ ((unused)),
		int manual_override, double v, double steer_kp, double steer_kd, double steer_ki)
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

//	if (delta_t < (0.7 * (1.0 / 40.0)))
//		return (u_t);

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
	
	/*printf("Chegou o outro kd %lf\n", steer_kp);
	
	kp = g_steering_Kp + factor * 791.5;
	ki = g_steering_Ki + factor * 4976.7;
	kd = g_steering_Kd + factor * 50.65;*/

	kp = steer_kp + factor * 791.5;
	ki = steer_ki + factor * 4976.7;
	kd = steer_kd + factor * 50.65;

	//printf("v %lf kp %lf ki %lf kd %lf\n", v, kp, ki, kd);

	u_t = (kp * error_t)  +  (ki * integral_t)  +  (kd * derivative_t);

	error_t_1 = error_t;

	// Anti windup
	if ((u_t < -100.0) || (u_t > 100.0))
		integral_t = integral_t_1;
	integral_t_1 = integral_t;

	previous_t = t;

	u_t = carmen_clamp(-100.0, u_t, 100.0);
	static double error_sum = 0.0;
	static bool already_cleaned_vectors = false;

	if(atan_desired_curvature == 0.0)
	{
		if(!already_cleaned_vectors)
		{
			make_plot_pid_automatic(steer_kp, steer_kd, steer_ki, error_sum);
			error_sum = 0.0;
			desired_steer_angle_vector.clear();
			current_steer_angle_vector.clear();
			error_vector.clear();
			already_cleaned_vectors = true;
		}
	}else
	{
		already_cleaned_vectors = false;
		desired_steer_angle_vector.push_back(atan_desired_curvature);
		current_steer_angle_vector.push_back(atan_current_curvature);
		error_vector.push_back(error_t);
	}
	error_sum += error_t;
	msg->atan_current_curvature = atan_current_curvature;
	msg->atan_desired_curvature = atan_desired_curvature;
	msg->derivative_t = derivative_t;
	msg->error_t = error_t;
	msg->integral_t = integral_t;
	msg->effort = u_t;
	msg_error->errror_sum = error_sum;

#ifdef PRINT
	fprintf(stdout, "STEERING (cc, dc, e, i, d, s): %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
		atan_current_curvature, atan_desired_curvature, error_t, integral_t, derivative_t, u_t, t);
	fflush(stdout);
#endif

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

//	if (delta_t < (0.7 * (1.0 / 40.0)))
//		return (u_t);

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

#ifdef PRINT
	fprintf(stdout, "STEERING (cc, dc, e, i, d, s): %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
		atan_current_curvature, atan_desired_curvature, error_t, integral_t, derivative_t, u_t, t);
	fflush(stdout);
#endif

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

#ifdef PRINT
	static double	initial_t = 0.0;
#endif

	if (previous_t == 0.0)
	{
//		initial_t = previous_t = carmen_get_time();
		for (int i = 0; i < DERIVATIVE_HISTORY_SIZE; i++)
			previous_derivatives[i] = 0.0;
		return (0.0);
	}
	double t = carmen_get_time();
	double delta_t = t - previous_t;

//	if (delta_t < (0.7 * (1.0 / 40.0)))
//		return (u_t);

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

#ifdef PRINT
	fprintf(stdout, "STEERING (cc, dc, e, i, d, s): %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
		atan_current_curvature, atan_desired_curvature, error_t, integral_t, derivative_t, u_t, t - initial_t);
	fflush(stdout);
#endif

	return u_t;
}

void
carmen_libpid_velocity_PID_controler_publish_data(velocity_pid_data_message *msg, double *throttle_command, double *brakes_command, int *gear_command,
										double desired_velocity, double current_velocity, double delta_t_old __attribute__ ((unused)),
										int manual_override, double kp, double kd, double ki)
{
	// http://en.wikipedia.org/wiki/PID_controller -> Discrete implementation
	static double 	error_t_1 = 0.0;	// error in time t-1
	static double 	integral_t = 0.0;
	static double 	integral_t_1 = 0.0;
	static double 	u_t = 0.0;			// u(t)	-> actuation in time t
	static double	previous_t = 0.0;
	static double	current_max_break_effort = 0.0;

	if (previous_t == 0.0)
	{
		previous_t = carmen_get_time();
		return;
	}
	double t = carmen_get_time();
	double delta_t = t - previous_t;

//	double g_maximum_acceleration = 0.5;
//	double delta_velocity = fabs(desired_velocity - current_velocity);
//	double command_curvature_signal = (current_velocity < desired_velocity) ? 1.0 : -1.0;
//	double max_velocity_change = g_maximum_acceleration * delta_t;
//
//	desired_velocity = current_velocity + command_curvature_signal * fmin(delta_velocity, max_velocity_change);

//	double delta_velocity = fabs(desired_velocity - current_velocity);
//	double command_velocity_signal = (current_velocity < desired_velocity) ? 1.0 : -1.0;
//	double max_velocity_change = 5.4 * delta_t;

//	desired_velocity = current_velocity + command_velocity_signal * fmin(delta_velocity, max_velocity_change);

	if (fabs(desired_velocity) < NEAR_ZERO_V) //(fabs(desired_velocity) < 0.01)	// Estudar esta linha para reduzir parada brusca
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
		if (current_max_break_effort < g_max_brake_effort)
			current_max_break_effort = current_max_break_effort + g_max_brake_effort / 80.0;
//		current_max_break_effort = current_max_break_effort + 0.05 * (g_max_brake_effort - current_max_break_effort);
		*brakes_command = current_max_break_effort;
//		*brakes_command = *brakes_command + NEAR_ZERO_V * (g_max_brake_effort - *brakes_command); // Estudar esta linha para reduzir parada brusca

		if ((desired_velocity > 0.0) && (current_velocity >= -NEAR_ZERO_V))
		{
			g_velocity_PID_controler_state = MOVE_CAR_FORWARD_ACCELERATING;
			*gear_command = 1; 		// 1 = Low; 2 = Drive (sharedlib/OpenJAUS/torc_docs/ByWire XGV User Manual v1.5.pdf page 67)
		}
		if ((desired_velocity < 0.0) && (current_velocity <= NEAR_ZERO_V))
		{
			g_velocity_PID_controler_state = MOVE_CAR_BACKWARD_ACCELERATING;
			*gear_command = 129; 	// 129 = Reverse gear (sharedlib/OpenJAUS/torc_docs/ByWire XGV User Manual v1.5.pdf page 67)
		}
	}

	if (g_velocity_PID_controler_state == MOVE_CAR_FORWARD_ACCELERATING)
	{
		u_t = 	g_velocity_forward_accelerating_Kp * error_t +
			g_velocity_forward_accelerating_Ki * integral_t +
			g_velocity_forward_accelerating_Kd * derivative_t;

		u_t = 	kp * error_t +
		ki * integral_t +
		kd * derivative_t;

		*throttle_command = u_t + g_throttle_gap;
		if (robot_model_id == ROBOT_ID_BUGGY)
			*brakes_command = 0.0;
		else
			*brakes_command = g_brake_gap;
		current_max_break_effort = *brakes_command;

#if (VEHICLE_WITH_BRAKES == 1)
		if ((desired_velocity > 0.0) && (error_t < (0.0 - g_minimum_delta_velocity)) && (u_t <= 0.0))
		{
			error_t_1 = integral_t = integral_t_1 = 0.0;
			g_velocity_PID_controler_state = MOVE_CAR_FORWARD_DECCELERATING;
		}
#endif
		if (desired_velocity <= 0.0)
			g_velocity_PID_controler_state = STOP_CAR;
	}
#if (VEHICLE_WITH_BRAKES == 1)
	else if (g_velocity_PID_controler_state == MOVE_CAR_FORWARD_DECCELERATING)
	{
		u_t = 	g_velocity_forward_deccelerating_Kp * error_t +
			g_velocity_forward_deccelerating_Ki * integral_t +
			g_velocity_forward_deccelerating_Kd * derivative_t;

		u_t = 	kp * error_t +
		ki * integral_t +
		kd * derivative_t;

		*throttle_command = 0.0;
		*brakes_command = -u_t + g_brake_gap;
		current_max_break_effort = *brakes_command;

		if ((desired_velocity > 0.0) && (error_t > (0.0 + g_minimum_delta_velocity)) && (u_t > 0.0))
		{
			error_t_1 = integral_t = integral_t_1 = 0.0;
			g_velocity_PID_controler_state = MOVE_CAR_FORWARD_ACCELERATING;
		}
		if (desired_velocity <= 0.0)
			g_velocity_PID_controler_state = STOP_CAR;
	}
#endif
	if (g_velocity_PID_controler_state == MOVE_CAR_BACKWARD_ACCELERATING)
	{
		u_t = 	g_velocity_backward_accelerating_Kp * error_t +
			g_velocity_backward_accelerating_Ki * integral_t +
			g_velocity_backward_accelerating_Kd * derivative_t;
		
		u_t = 	kp * error_t +
		ki * integral_t +
		kd * derivative_t;

		*throttle_command = -u_t + g_throttle_gap;
		if (robot_model_id == ROBOT_ID_BUGGY)
			*brakes_command = 0.0;
		else
			*brakes_command = g_brake_gap;
//		current_max_break_effort = *brakes_command;
		current_max_break_effort = 0.0;

#if (VEHICLE_WITH_BRAKES == 1)
		if ((desired_velocity < 0.0) && (error_t > (0.0 + g_minimum_delta_velocity)) && (u_t >= 0.0))
		{
			error_t_1 = integral_t = integral_t_1 = 0.0;
			g_velocity_PID_controler_state = MOVE_CAR_BACKWARD_DECCELERATING;
		}
#endif
		if (desired_velocity >= 0.0)
			g_velocity_PID_controler_state = STOP_CAR;
	}
#if (VEHICLE_WITH_BRAKES == 1)
	else if (g_velocity_PID_controler_state == MOVE_CAR_BACKWARD_DECCELERATING)
	{
		u_t = 	g_velocity_backward_deccelerating_Kp * error_t +
			g_velocity_backward_deccelerating_Ki * integral_t +
			g_velocity_backward_deccelerating_Kd * derivative_t;

		u_t = 	kp * error_t +
		ki * integral_t +
		kd * derivative_t;

		*throttle_command = 0.0;
		*brakes_command = u_t + g_brake_gap;
//		current_max_break_effort = *brakes_command;
		current_max_break_effort = 0.0;

		if ((desired_velocity < 0.0) && (error_t < (0.0 - g_minimum_delta_velocity)) && (u_t < 0.0))
		{
			error_t_1 = integral_t = integral_t_1 = 0.0;
			g_velocity_PID_controler_state = MOVE_CAR_BACKWARD_ACCELERATING;
		}
		if (desired_velocity >= 0.0)
			g_velocity_PID_controler_state = STOP_CAR;
	}
#endif
	error_t_1 = error_t;

	// Anti windup
	if (robot_model_id == ROBOT_ID_BUGGY)
	{
		if ((g_velocity_PID_controler_state == MOVE_CAR_BACKWARD_ACCELERATING) || (g_velocity_PID_controler_state == MOVE_CAR_FORWARD_ACCELERATING))
		{
			if ((*throttle_command < 0.0) || (*throttle_command > 100.0) ||
				(*brakes_command < 0.0) || (*brakes_command > 100.0))
				integral_t = integral_t_1;
		}
		else
		{
			if ((*throttle_command < 0.0) || (*throttle_command > 100.0) ||
				(*brakes_command < g_brake_gap) || (*brakes_command > 100.0))
				integral_t = integral_t_1;
		}
	}
	else
	{
		if ((*throttle_command < 0.0) || (*throttle_command > 100.0) ||
		    (*brakes_command < g_brake_gap) || (*brakes_command > 100.0))
			integral_t = integral_t_1;
	}

	integral_t_1 = integral_t;

	previous_t = t;

	*throttle_command = carmen_clamp(0.0, *throttle_command, 100.0);
	if (robot_model_id == ROBOT_ID_BUGGY)
	{
		if ((g_velocity_PID_controler_state == MOVE_CAR_BACKWARD_ACCELERATING) || (g_velocity_PID_controler_state == MOVE_CAR_FORWARD_ACCELERATING))
			*brakes_command = carmen_clamp(0.0, *brakes_command, 100.0);
		else
			*brakes_command = carmen_clamp(g_brake_gap, *brakes_command, 100.0);
	}
	else
		*brakes_command = carmen_clamp(g_brake_gap, *brakes_command, 100.0);

	
	msg->brakes_command = *brakes_command;
	msg->current_velocity = current_velocity;
	msg->desired_velocity = desired_velocity;
	msg->derivative_t = derivative_t;
	msg->error_t = error_t;
	msg->integral_t = integral_t;
	msg->PID_controler_state = g_velocity_PID_controler_state;
	msg->throttle_command = *throttle_command;
	
#ifdef PRINT
	fprintf(stdout, "VELOCITY (st, cv, dv, e, t, b, i, d, bs, ts): %d, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
		g_velocity_PID_controler_state, current_velocity, desired_velocity, error_t,
		*throttle_command, *brakes_command,
		integral_t, derivative_t, carmen_get_time());
	fflush(stdout);
#endif

}

void
carmen_libpid_velocity_PID_controler(double *throttle_command, double *brakes_command, int *gear_command,
										double desired_velocity, double current_velocity, double delta_t_old __attribute__ ((unused)),
										int manual_override, double gear_ratio __attribute__ ((unused)))
{
	// http://en.wikipedia.org/wiki/PID_controller -> Discrete implementation
	static double 	error_t_1 = 0.0;	// error in time t-1
	static double 	integral_t = 0.0;
	static double 	integral_t_1 = 0.0;
	static double 	u_t = 0.0;			// u(t)	-> actuation in time t
	static double	previous_t = 0.0;
	static double	current_max_break_effort = 0.0;

	if (previous_t == 0.0)
	{
		previous_t = carmen_get_time();
		return;
	}
	double t = carmen_get_time();
	double delta_t = t - previous_t;

//	double g_maximum_acceleration = 0.5;
//	double delta_velocity = fabs(desired_velocity - current_velocity);
//	double command_curvature_signal = (current_velocity < desired_velocity) ? 1.0 : -1.0;
//	double max_velocity_change = g_maximum_acceleration * delta_t;
//
//	desired_velocity = current_velocity + command_curvature_signal * fmin(delta_velocity, max_velocity_change);

//	double delta_velocity = fabs(desired_velocity - current_velocity);
//	double command_velocity_signal = (current_velocity < desired_velocity) ? 1.0 : -1.0;
//	double max_velocity_change = 5.4 * delta_t;

//	desired_velocity = current_velocity + command_velocity_signal * fmin(delta_velocity, max_velocity_change);

	if (fabs(desired_velocity) < NEAR_ZERO_V) //(fabs(desired_velocity) < 0.01)	// Estudar esta linha para reduzir parada brusca
	{
		desired_velocity = 0.0;
		g_velocity_PID_controler_state = STOP_CAR;
	}

	double a = (current_velocity - g_v) / delta_t;
	g_v = current_velocity;

//	double error_multiplier = 1.0 + g_v_error_multiplier * fabs(current_velocity / g_target_velocity);
//	if (error_multiplier > (1.0 + g_v_error_multiplier))
//		error_multiplier = 1.0 + g_v_error_multiplier;
//	double error_t = (desired_velocity - current_velocity) * error_multiplier;
	double error_t = (desired_velocity - current_velocity) * 1.0;

	if (manual_override == 0)
		integral_t = integral_t + error_t * delta_t;
	else
		integral_t = 0.0;

	double derivative_t = (error_t - error_t_1) / delta_t;

	if (g_velocity_PID_controler_state == STOP_CAR)
	{
		error_t_1 = integral_t = integral_t_1 = 0.0;

		*throttle_command = 0.0;
		if (current_max_break_effort < g_max_brake_effort)
			current_max_break_effort = current_max_break_effort + g_max_brake_effort / 80.0;
//		current_max_break_effort = current_max_break_effort + 0.05 * (g_max_brake_effort - current_max_break_effort);
		*brakes_command = current_max_break_effort;
//		*brakes_command = *brakes_command + NEAR_ZERO_V * (g_max_brake_effort - *brakes_command); // Estudar esta linha para reduzir parada brusca

		if ((desired_velocity > 0.0) && (current_velocity >= -NEAR_ZERO_V))
		{
			g_velocity_PID_controler_state = MOVE_CAR_FORWARD_ACCELERATING;
			*gear_command = 1; 		// 1 = Low; 2 = Drive (sharedlib/OpenJAUS/torc_docs/ByWire XGV User Manual v1.5.pdf page 67)
		}
		if ((desired_velocity < 0.0) && (current_velocity <= NEAR_ZERO_V))
		{
			g_velocity_PID_controler_state = MOVE_CAR_BACKWARD_ACCELERATING;
			*gear_command = 129; 	// 129 = Reverse gear (sharedlib/OpenJAUS/torc_docs/ByWire XGV User Manual v1.5.pdf page 67)
		}
	}

	if (g_velocity_PID_controler_state == MOVE_CAR_FORWARD_ACCELERATING)
	{
		u_t = 	g_velocity_forward_accelerating_Kp * error_t +
			g_velocity_forward_accelerating_Ki * integral_t +
			g_velocity_forward_accelerating_Kd * derivative_t;

		*throttle_command = u_t + g_throttle_gap;
		if (robot_model_id == ROBOT_ID_BUGGY)
			*brakes_command = 0.0;
		else
			*brakes_command = g_brake_gap;
		current_max_break_effort = *brakes_command;

#if (VEHICLE_WITH_BRAKES == 1)
		if ((desired_velocity > 0.0) && (error_t < (0.0 - g_minimum_delta_velocity)) && (u_t <= 0.0))
		{
			error_t_1 = integral_t = integral_t_1 = 0.0;
			g_velocity_PID_controler_state = MOVE_CAR_FORWARD_DECCELERATING;
		}
#endif
		if (desired_velocity <= 0.0)
			g_velocity_PID_controler_state = STOP_CAR;
	}
#if (VEHICLE_WITH_BRAKES == 1)
	else if (g_velocity_PID_controler_state == MOVE_CAR_FORWARD_DECCELERATING)
	{
		u_t = 	g_velocity_forward_deccelerating_Kp * error_t +
			g_velocity_forward_deccelerating_Ki * integral_t +
			g_velocity_forward_deccelerating_Kd * derivative_t;

		*throttle_command = 0.0;
		*brakes_command = -u_t + g_brake_gap;
		current_max_break_effort = *brakes_command;

		if ((desired_velocity > 0.0) && (error_t > (0.0 + g_minimum_delta_velocity)) && (u_t > 0.0))
		{
			error_t_1 = integral_t = integral_t_1 = 0.0;
			g_velocity_PID_controler_state = MOVE_CAR_FORWARD_ACCELERATING;
		}
		if (desired_velocity <= 0.0)
			g_velocity_PID_controler_state = STOP_CAR;
	}
#endif
	if (g_velocity_PID_controler_state == MOVE_CAR_BACKWARD_ACCELERATING)
	{
		u_t = 	g_velocity_backward_accelerating_Kp * error_t +
			g_velocity_backward_accelerating_Ki * integral_t +
			g_velocity_backward_accelerating_Kd * derivative_t;

		*throttle_command = -u_t + g_throttle_gap;
		if (robot_model_id == ROBOT_ID_BUGGY)
			*brakes_command = 0.0;
		else
			*brakes_command = g_brake_gap;
//		current_max_break_effort = *brakes_command;
		current_max_break_effort = 0.0;

#if (VEHICLE_WITH_BRAKES == 1)
		if ((desired_velocity < 0.0) && (error_t > (0.0 + g_minimum_delta_velocity)) && (u_t >= 0.0))
		{
			error_t_1 = integral_t = integral_t_1 = 0.0;
			g_velocity_PID_controler_state = MOVE_CAR_BACKWARD_DECCELERATING;
		}
#endif
		if (desired_velocity >= 0.0)
			g_velocity_PID_controler_state = STOP_CAR;
	}
#if (VEHICLE_WITH_BRAKES == 1)
	else if (g_velocity_PID_controler_state == MOVE_CAR_BACKWARD_DECCELERATING)
	{
		u_t = 	g_velocity_backward_deccelerating_Kp * error_t +
			g_velocity_backward_deccelerating_Ki * integral_t +
			g_velocity_backward_deccelerating_Kd * derivative_t;

		*throttle_command = 0.0;
		*brakes_command = u_t + g_brake_gap;
//		current_max_break_effort = *brakes_command;
		current_max_break_effort = 0.0;

		if ((desired_velocity < 0.0) && (error_t < (0.0 - g_minimum_delta_velocity)) && (u_t < 0.0))
		{
			error_t_1 = integral_t = integral_t_1 = 0.0;
			g_velocity_PID_controler_state = MOVE_CAR_BACKWARD_ACCELERATING;
		}
		if (desired_velocity >= 0.0)
			g_velocity_PID_controler_state = STOP_CAR;
	}
#endif
	error_t_1 = error_t;

	// Anti windup
	if (robot_model_id == ROBOT_ID_BUGGY)
	{
		if ((g_velocity_PID_controler_state == MOVE_CAR_BACKWARD_ACCELERATING) || (g_velocity_PID_controler_state == MOVE_CAR_FORWARD_ACCELERATING))
		{
			if ((*throttle_command < 0.0) || (*throttle_command > 100.0) ||
				(*brakes_command < 0.0) || (*brakes_command > 100.0))
				integral_t = integral_t_1;
		}
		else
		{
			if ((*throttle_command < 0.0) || (*throttle_command > 100.0) ||
				(*brakes_command < g_brake_gap) || (*brakes_command > 100.0))
				integral_t = integral_t_1;
		}
	}
	else
	{
		if ((*throttle_command < 0.0) || (*throttle_command > 100.0) ||
		    (*brakes_command < g_brake_gap) || (*brakes_command > 100.0))
			integral_t = integral_t_1;
	}

	integral_t_1 = integral_t;

	previous_t = t;

	*throttle_command = carmen_clamp(0.0, *throttle_command, 100.0);
	if (robot_model_id == ROBOT_ID_BUGGY)
	{
		if ((g_velocity_PID_controler_state == MOVE_CAR_BACKWARD_ACCELERATING) || (g_velocity_PID_controler_state == MOVE_CAR_FORWARD_ACCELERATING))
			*brakes_command = carmen_clamp(0.0, *brakes_command, 100.0);
		else
			*brakes_command = carmen_clamp(g_brake_gap, *brakes_command, 100.0);
	}
	else
		*brakes_command = carmen_clamp(g_brake_gap, *brakes_command, 100.0);

#ifdef PRINT
	fprintf(stdout, "VELOCITY (st, cv, dv, e, t, b, i, d, bs, ts): %d, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
		g_velocity_PID_controler_state, current_velocity, desired_velocity, error_t,
		*throttle_command, *brakes_command,
		integral_t, derivative_t, carmen_get_time(), a, delta_t);
	fflush(stdout);
#endif

}


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Inicializations																				//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_libpid_read_PID_parameters(int argc, char *argv[])
{
	int num_items;

	carmen_param_allow_unfound_variables(0);

	carmen_param_t param_list[]=
	{
		{(char *)"robot", (char *)"PID_steering_kp", CARMEN_PARAM_DOUBLE, &g_steering_Kp, 1, NULL},
		{(char *)"robot", (char *)"PID_steering_ki", CARMEN_PARAM_DOUBLE, &g_steering_Ki, 1, NULL},
		{(char *)"robot", (char *)"PID_steering_kd", CARMEN_PARAM_DOUBLE, &g_steering_Kd, 1, NULL},
		{(char *)"robot", (char *)"PID_minimum_delta_velocity", CARMEN_PARAM_DOUBLE, &g_minimum_delta_velocity, 1, NULL},
		{(char *)"robot", (char *)"PID_velocity_forward_accelerating_Kp", CARMEN_PARAM_DOUBLE, &g_velocity_forward_accelerating_Kp, 1, NULL},
		{(char *)"robot", (char *)"PID_velocity_forward_accelerating_Ki", CARMEN_PARAM_DOUBLE, &g_velocity_forward_accelerating_Ki, 1, NULL},
		{(char *)"robot", (char *)"PID_velocity_forward_accelerating_Kd", CARMEN_PARAM_DOUBLE, &g_velocity_forward_accelerating_Kd, 1, NULL},
		{(char *)"robot", (char *)"PID_velocity_forward_deccelerating_Kp", CARMEN_PARAM_DOUBLE, &g_velocity_forward_deccelerating_Kp, 1, NULL},
		{(char *)"robot", (char *)"PID_velocity_forward_deccelerating_Ki", CARMEN_PARAM_DOUBLE, &g_velocity_forward_deccelerating_Ki, 1, NULL},
		{(char *)"robot", (char *)"PID_velocity_forward_deccelerating_Kd", CARMEN_PARAM_DOUBLE, &g_velocity_forward_deccelerating_Kd, 1, NULL},
		{(char *)"robot", (char *)"PID_velocity_backward_accelerating_Kp", CARMEN_PARAM_DOUBLE, &g_velocity_backward_accelerating_Kp, 1, NULL},
		{(char *)"robot", (char *)"PID_velocity_backward_accelerating_Ki", CARMEN_PARAM_DOUBLE, &g_velocity_backward_accelerating_Ki, 1, NULL},
		{(char *)"robot", (char *)"PID_velocity_backward_accelerating_Kd", CARMEN_PARAM_DOUBLE, &g_velocity_backward_accelerating_Kd, 1, NULL},
		{(char *)"robot", (char *)"PID_velocity_backward_deccelerating_Kp", CARMEN_PARAM_DOUBLE, &g_velocity_backward_deccelerating_Kp, 1, NULL},
		{(char *)"robot", (char *)"PID_velocity_backward_deccelerating_Ki", CARMEN_PARAM_DOUBLE, &g_velocity_backward_deccelerating_Ki, 1, NULL},
		{(char *)"robot", (char *)"PID_velocity_backward_deccelerating_Kd", CARMEN_PARAM_DOUBLE, &g_velocity_backward_deccelerating_Kd, 1, NULL},
		{(char *)"robot", (char *)"PID_velocity_brake_gap", CARMEN_PARAM_DOUBLE, &g_brake_gap, 1, NULL},
		{(char *)"robot", (char *)"PID_velocity_max_brake_effort", CARMEN_PARAM_DOUBLE, &g_max_brake_effort, 1, NULL},
		{(char *)"robot", (char *)"maximum_steering_command_rate", CARMEN_PARAM_DOUBLE, &g_maximum_steering_command_rate, 1, NULL},
		{(char *)"robot", (char *)"PID_velocity_throttle_gap", CARMEN_PARAM_DOUBLE, &g_throttle_gap, 1, NULL},
	};

	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	carmen_param_t param_optional_list[] =
	{
		{(char *) "robot", (char *) "model_id",	CARMEN_PARAM_INT, &(robot_model_id),0, NULL},
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, param_optional_list, sizeof(param_optional_list) / sizeof(param_optional_list[0]));
}
