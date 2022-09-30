#include <carmen/carmen.h>
#include <carmen/motion_planner.h>
#include "ford_escape_hybrid_tune_pid_automatic.h"
#include <carmen/car_model.h>

#include "ford_escape_hybrid_interface.h"
#include "ford_escape_hybrid_messages.h"


using namespace std;

typedef double (*MotionControlFunction)(double v, double w, double t);

static carmen_robot_ackerman_config_t motion_planner_config;

static carmen_robot_and_trailers_motion_command_t motion_commands_vector[NUM_MOTION_COMMANDS_PER_VECTOR];

static double max_v = 0.0;
static double max_phi = 0.0;
static double frequency = 0.0;
static double timer_period = 1.0;

static double t1, t2, t3;

static MotionControlFunction v_function;
static MotionControlFunction phi_function;

static int wave_form = 0;


carmen_behavior_selector_low_level_state_t behavior_selector_low_level_state;


static void
signal_handler(int signo __attribute__ ((unused)) )
{
	carmen_ipc_disconnect();
	
	exit(0);
}


static void
send_trajectory_to_robot()
{
	carmen_base_ackerman_publish_motion_command(motion_commands_vector, NUM_MOTION_COMMANDS_PER_VECTOR, carmen_get_time());
}


double
get_effort_in_time_from_spline(EFFORT_SPLINE_DESCRIPTOR *descriptors, double time)
{
	double x[4] = { 0.0, PREDICTION_HORIZON / 3.0, 2.0 * PREDICTION_HORIZON / 3.0, PREDICTION_HORIZON };
	double y[4] = { descriptors->k1, descriptors->k2, descriptors->k3, descriptors->k4 };

	gsl_interp_accel *acc = gsl_interp_accel_alloc();
	const gsl_interp_type *type = gsl_interp_cspline;
	gsl_spline *phi_effort_spline = gsl_spline_alloc(type, 4);

	gsl_spline_init(phi_effort_spline, x, y, 4);

	double effort = gsl_spline_eval(phi_effort_spline, time, acc);

	gsl_spline_free(phi_effort_spline);
	gsl_interp_accel_free(acc);

	return (effort);
}


vector<double>
get_effort_vector_from_spline_descriptors(EFFORT_SPLINE_DESCRIPTOR *descriptors, double prediction_horizon)
{
	double x[4] = { 0.0, prediction_horizon / 3.0, 2.0 * prediction_horizon / 3.0, prediction_horizon };
	double y[4] = { descriptors->k1, descriptors->k2, descriptors->k3, descriptors->k4 };

	gsl_interp_accel *acc = gsl_interp_accel_alloc();
	const gsl_interp_type *type = gsl_interp_cspline;
	gsl_spline *phi_effort_spline = gsl_spline_alloc(type, 4);

	gsl_spline_init(phi_effort_spline, x, y, 4);

	vector<double> effort_vector;
	for (double t = 0.0; t < prediction_horizon; t += DELTA_T)
		effort_vector.push_back(gsl_spline_eval(phi_effort_spline, t, acc));
		//effort_vector.push_back(carmen_clamp(-100.0, gsl_spline_eval(phi_effort_spline, t, acc), 100.0));

	gsl_spline_free(phi_effort_spline);
	gsl_interp_accel_free(acc);

	return (effort_vector);
}


unsigned int
get_motion_timed_index_to_motion_command(PARAMS* params) // TODO nao devia retornar j--   ????
{
	double motion_commands_vector_time = params->motion_commands_vector[0].time;
	unsigned int j = 0;
	while ((motion_commands_vector_time	< params->time_elapsed_since_last_motion_command) && (j < params->motion_commands_vector_size))
	{
		j++;
		motion_commands_vector_time += params->motion_commands_vector[j].time;
	}

	return j;
}


vector<double>
get_velocity_supersampling_motion_commands_vector(PARAMS *params, unsigned int size)
{
	vector<double> velocity_vector;
	double phi_vector_time = 0.0;

	unsigned int timed_index_to_motion_command = get_motion_timed_index_to_motion_command(params);
	double motion_commands_vector_time = params->motion_commands_vector[timed_index_to_motion_command].time;

	for (unsigned int i = 0; i < size; i++)
	{
		velocity_vector.push_back(params->motion_commands_vector[timed_index_to_motion_command].v);

		phi_vector_time += DELTA_T;
		if (phi_vector_time > motion_commands_vector_time)
		{
			timed_index_to_motion_command++;
			if (timed_index_to_motion_command >= params->motion_commands_vector_size)
				break;

			motion_commands_vector_time += params->motion_commands_vector[timed_index_to_motion_command].time;
		}
	}

	return (velocity_vector);
}


double
car_steering_model(double steering_effort, double atan_current_curvature, double v, fann_type *steering_ann_input, PARAMS *params)
{
	//steering_effort *= (1.0 / (1.0 + (params->current_velocity * params->current_velocity) / CAR_MODEL_GAIN)); // boa
	steering_effort = carmen_clamp(-100.0, steering_effort, 100.0);

	// TODO fazer funcao aterar a proxima curvatura tambem
	double phi = carmen_libcarneuralmodel_compute_new_phi_from_effort(steering_effort, atan_current_curvature, steering_ann_input,
			params->steering_ann, v, params->understeer_coeficient, params->distance_rear_axles, 2.0 * params->max_phi);
//	phi = 1.0 * phi;// - 0.01;
//	phi *= (1.0 / (1.0 + v / 10.0));
	
	return (phi);
}


vector<double>
get_phi_vector_from_spline_descriptors(EFFORT_SPLINE_DESCRIPTOR *descriptors, PARAMS *params)
{
	fann_type steering_ann_input[NUM_STEERING_ANN_INPUTS];
	memcpy(steering_ann_input, params->steering_ann_input, NUM_STEERING_ANN_INPUTS * sizeof(fann_type));

	vector<double> effort_vector = get_effort_vector_from_spline_descriptors(descriptors, PREDICTION_HORIZON);
	vector<double> velocity_vector = get_velocity_supersampling_motion_commands_vector(params, effort_vector.size());
	vector<double> phi_vector;
	double phi, atan_current_curvature = params->atan_current_curvature;

	for (unsigned int i = 0; i < effort_vector.size(); i++)
	{
		phi = car_steering_model(effort_vector[i], atan_current_curvature, velocity_vector[i], steering_ann_input, params);

		phi_vector.push_back(phi + params->dk);

		atan_current_curvature = carmen_get_curvature_from_phi(phi, params->current_velocity, params->understeer_coeficient, params->distance_rear_axles);
	}
	return (phi_vector);
}


double
my_f_new(const gsl_vector *v, void *params_ptr)
{
	EFFORT_SPLINE_DESCRIPTOR d;
	PARAMS *params = (PARAMS *) params_ptr;

	double delta_t = DELTA_T;
	double phi_vector_time = 0.0;
	double error = 0.0;
	double error_sum = 0.0;

	d.k1 = gsl_vector_get(v, 0);
	d.k2 = gsl_vector_get(v, 1);
	d.k3 = gsl_vector_get(v, 2);
	d.k4 = gsl_vector_get(v, 3);

	vector<double> phi_vector = get_phi_vector_from_spline_descriptors(&d, params);

	unsigned int j = get_motion_timed_index_to_motion_command(params);
	double motion_commands_vector_time = params->motion_commands_vector[j].time;

//	while ((motion_commands_vector_time < params->time_elapsed_since_last_motion_command) && (j < params->motion_commands_vector_size))
//	{
//		j++;
//		motion_commands_vector_time += params->motion_commands_vector[j].time;
//	}

	for (unsigned int i = 0; i < phi_vector.size(); i++)
	{
		error = phi_vector[i] - params->motion_commands_vector[j].phi;
		error_sum += fabs(error);

		phi_vector_time += delta_t;
		if (phi_vector_time > motion_commands_vector_time)
		{
			j++;
			if (j >= params->motion_commands_vector_size)
				break;
			motion_commands_vector_time += params->motion_commands_vector[j].time;
		}
	}

	double cost = error_sum;// + 0.00011 * sqrt((params->previous_k1 - d.k1) * (params->previous_k1 - d.k1));
	//printf("%lf  %lf  %lf  %lf\n", cost, params->previous_k1, d.k1, params->previous_k1 - d.k1);

	return (cost);
}


void
my_df_new(const gsl_vector *v, void *params, gsl_vector *df)
{
	EFFORT_SPLINE_DESCRIPTOR d;
	double h = 0.1;
	double f_x = my_f_new(v, params);
	gsl_vector *x_h;

	x_h = gsl_vector_alloc(4);

	d.k1 = gsl_vector_get(v, 0);
	d.k2 = gsl_vector_get(v, 1);
	d.k3 = gsl_vector_get(v, 2);
	d.k4 = gsl_vector_get(v, 3);

	gsl_vector_set(x_h, 0, d.k1 + h);
	gsl_vector_set(x_h, 1, d.k2);
	gsl_vector_set(x_h, 2, d.k3);
	gsl_vector_set(x_h, 3, d.k4);
	double f_k1_h = my_f_new(x_h, params);
	double df_k1_h = (f_k1_h - f_x) / h;

	gsl_vector_set(x_h, 0, d.k1);
	gsl_vector_set(x_h, 1, d.k2 + h);
	gsl_vector_set(x_h, 2, d.k3);
	gsl_vector_set(x_h, 3, d.k4);
	double f_k2_h = my_f_new(x_h, params);
	double df_k2_h = (f_k2_h - f_x) / h;

	gsl_vector_set(x_h, 0, d.k1);
	gsl_vector_set(x_h, 1, d.k2);
	gsl_vector_set(x_h, 2, d.k3 + h);
	gsl_vector_set(x_h, 3, d.k4);
	double f_k3_h = my_f_new(x_h, params);
	double df_k3_h = (f_k3_h - f_x) / h;

	gsl_vector_set(x_h, 0, d.k1);
	gsl_vector_set(x_h, 1, d.k2);
	gsl_vector_set(x_h, 2, d.k3);
	gsl_vector_set(x_h, 3, d.k4 + h);
	double f_k4_h = my_f_new(x_h, params);
	double df_k4_h = (f_k4_h - f_x) / h;

	gsl_vector_set(df, 0, df_k1_h);
	gsl_vector_set(df, 1, df_k2_h);
	gsl_vector_set(df, 2, df_k3_h);
	gsl_vector_set(df, 3, df_k4_h);

	gsl_vector_free(x_h);
}


void
my_fdf_new(const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
	*f = my_f_new (x, params);
	my_df_new (x, params, df);
}


EFFORT_SPLINE_DESCRIPTOR
get_optimized_effort_new(PARAMS *params, EFFORT_SPLINE_DESCRIPTOR seed)
{
	gsl_vector *x;
	gsl_multimin_function_fdf my_func;
	size_t iter = 0;
	int status;

	my_func.n = 4;
	my_func.f = my_f_new;
	my_func.df = my_df_new;
	my_func.fdf = my_fdf_new;
	my_func.params = params;

	x = gsl_vector_alloc (4);  // Num of parameters to minimize
	gsl_vector_set(x, 0, seed.k1);
	gsl_vector_set(x, 1, seed.k2);
	gsl_vector_set(x, 2, seed.k3);
	gsl_vector_set(x, 3, seed.k4);

	const gsl_multimin_fdfminimizer_type *T = gsl_multimin_fdfminimizer_conjugate_fr;
	gsl_multimin_fdfminimizer *s = gsl_multimin_fdfminimizer_alloc(T, 4);

	gsl_multimin_fdfminimizer_set(s, &my_func, x, 0.01, 0.0001);

	do
	{
		iter++;
		status = gsl_multimin_fdfminimizer_iterate(s);

		if (status)
			break;

		status = gsl_multimin_test_gradient(s->gradient, 1e-3);

	} while ((status == GSL_CONTINUE) && (iter < 15));

	//printf("iter = %ld\n", iter);

	seed.k1 = carmen_clamp(-100.0, gsl_vector_get(s->x, 0), 100.0);
	seed.k2 = carmen_clamp(-100.0, gsl_vector_get(s->x, 1), 100.0);
	seed.k3 = carmen_clamp(-100.0, gsl_vector_get(s->x, 2), 100.0);
	seed.k4 = carmen_clamp(-100.0, gsl_vector_get(s->x, 3), 100.0);

	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

	return (seed);
}


static void
build_trajectory(double t)
{
	int i;
	double v, phi;
	
	if (v_function)
		v = (*v_function)(max_v, frequency, t);
	else
		v = 0.0;

	if (phi_function)
		phi = (*phi_function)(max_phi, frequency, t);
	else
		phi = 0.0;

	for (i = 0; i < NUM_MOTION_COMMANDS_PER_VECTOR; i++)
	{
		motion_commands_vector[i].v = v;
		motion_commands_vector[i].phi = phi;
		motion_commands_vector[i].time = 1.0 * timer_period;
	}

	carmen_warn("t = %lf, v = %lf, phi = %lf, time = %lf\n", t,
			motion_commands_vector[0].v, 
			motion_commands_vector[0].phi,
			motion_commands_vector[0].time);

	send_trajectory_to_robot();
}



void
build_trajectory_trapezoidal_v_phi()
{
	double delta_t, t;
	int i;

	double t0 = 2.0;
	double t4 = 2.0;
	delta_t = (t0 + t1 + t2 + t3 + t4) / (double) (NUM_MOTION_COMMANDS_PER_VECTOR - 2);

	printf("max_v %lf\n", max_v);
	for (i = 0, t = 0.0; t < t0; t += delta_t, i++)
	{
		motion_commands_vector[i].v = 0.0;
		motion_commands_vector[i].phi = 0.0;
		motion_commands_vector[i].time = delta_t;
		printf("av %lf\n", motion_commands_vector[i].v);
	}

	for (t = 0.0; t < t1; t += delta_t, i++)
	{
		motion_commands_vector[i].v = t * (max_v / t1);
		motion_commands_vector[i].phi = t * (max_phi / t1);
		motion_commands_vector[i].time = delta_t;
		printf("bv %lf\n", motion_commands_vector[i].v);
	}

	for (t = 0.0; t < t2; t += delta_t, i++)
	{
		motion_commands_vector[i].v = max_v;
		motion_commands_vector[i].phi = max_phi;
		motion_commands_vector[i].time = delta_t;
		printf("cv %lf\n", motion_commands_vector[i].v);
	}

	for (t = 0.0; t <= t3; t += delta_t, i++)
	{
		motion_commands_vector[i].v = max_v - t * (max_v / t3);
		motion_commands_vector[i].phi = max_phi - t * (max_phi / t3);;
		motion_commands_vector[i].time = delta_t;
		printf("dv %lf\n", motion_commands_vector[i].v);
	}

	for (t = 0.0; t <= (t4 + delta_t); t += delta_t, i++)
	{
		motion_commands_vector[i].v = 0.0;
		motion_commands_vector[i].phi = 0.0;
		motion_commands_vector[i].time = delta_t;
		printf("ev %lf\n", motion_commands_vector[i].v);
	}
	printf("max_phi = %lf, i = %d, NUM_MOTION_COMMANDS_PER_VECTOR = %d\n", max_phi, i, NUM_MOTION_COMMANDS_PER_VECTOR);
	send_trajectory_to_robot();
}


void
build_trajectory_trapezoidal_phi()
{
	double delta_t, t;
	int i;
	
	double t0 = 2.0;
	double t4 = 2.0;
	delta_t = (t0 + t1 + t2 + t3 + t4) / (double) (NUM_MOTION_COMMANDS_PER_VECTOR - 2);

	for (i = 0, t = 0.0; t < t0; t += delta_t, i++)
	{
		motion_commands_vector[i].v = max_v;
		motion_commands_vector[i].phi = 0.0;
		motion_commands_vector[i].time = delta_t;
	}

	for (t = 0.0; t < t1; t += delta_t, i++)
	{
		motion_commands_vector[i].v = max_v;
		motion_commands_vector[i].phi = t * (max_phi / t1);
		motion_commands_vector[i].time = delta_t;
	}
	
	for (t = 0.0; t < t2; t += delta_t, i++)
	{
		motion_commands_vector[i].v = max_v;
		motion_commands_vector[i].phi = max_phi;
		motion_commands_vector[i].time = delta_t;
	}	
	
	for (t = 0.0; t <= t3; t += delta_t, i++)
	{
		motion_commands_vector[i].v = max_v;
		motion_commands_vector[i].phi = max_phi - t * (max_phi / t3);
		motion_commands_vector[i].time = delta_t;
	}	
	
	for (t = 0.0; t <= (t4 + delta_t); t += delta_t, i++)
	{
		//3 * exp(-((10 - x) * (10 - x)) / (4 * 4))
		motion_commands_vector[i].v = max_v;
		motion_commands_vector[i].phi = 0.0;
		motion_commands_vector[i].time = delta_t;
	}
//	printf("i = %d, NUM_MOTION_COMMANDS_PER_VECTOR = %d\n", i, NUM_MOTION_COMMANDS_PER_VECTOR);
	send_trajectory_to_robot();
}

void
build_trajectory_sinusoidal_phi()
{
	double delta_t, t;
	int i;
	double t0 = 2.0;

	delta_t = (t0 + t1) / (double) (NUM_MOTION_COMMANDS_PER_VECTOR - 2);

	for (i = 0, t = 0.0; t < t0; t += delta_t, i++)
	{
		motion_commands_vector[i].v = 0.0;
		motion_commands_vector[i].phi = 0.0;
		motion_commands_vector[i].time = delta_t;
		printf("av %lf\n", motion_commands_vector[i].v);
	}

	for (t = 0.0; t < t1; t += delta_t, i++)
	{
		motion_commands_vector[i].v = max_v;

		motion_commands_vector[i].phi = max_phi * sin(2.0 * M_PI * frequency * t);
		motion_commands_vector[i].time = delta_t;

		printf("%lf   %lf\n", frequency * t, motion_commands_vector[i].phi);
	}

	printf("i = %d, NUM_MOTION_COMMANDS_PER_VECTOR = %d\n", i, NUM_MOTION_COMMANDS_PER_VECTOR);
	send_trajectory_to_robot();
}


tune_pid_gain_velocity_parameters_message* 
fill_pid_gain_velocity_parameters_message()
{
	tune_pid_gain_velocity_parameters_message *pid_msg = (tune_pid_gain_velocity_parameters_message*) malloc (sizeof (tune_pid_gain_velocity_parameters_message));
	pid_msg->kd = -0.2;
	pid_msg->ki = 6.0;
	pid_msg->kp = 25.0;
	pid_msg->host = carmen_get_host();
	pid_msg->timestamp = carmen_get_time();
	return pid_msg;
}

tune_pid_gain_steering_parameters_message* 
fill_pid_gain_steering_parameters_message()
{
	tune_pid_gain_steering_parameters_message *pid_msg = (tune_pid_gain_steering_parameters_message*) malloc (sizeof (tune_pid_gain_steering_parameters_message));
	pid_msg->kd = 30.8;
	pid_msg->ki = 2008.7;
	pid_msg->kp = 689.4;
	pid_msg->host = carmen_get_host();
	pid_msg->timestamp = carmen_get_time();
	return pid_msg;
}

void
publish_current_state(carmen_behavior_selector_state_message *msg)
{
	IPC_RETURN_TYPE err;

	msg->timestamp = carmen_get_time();
	msg->host = carmen_get_host();

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME, msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME);
}


static void
timer_handler()
{
	static int first_time = 1;

	// Publicar ganhos de steering 
	// Publicar ganhos de velocity
	
	if (first_time)
	{
		if (frequency != 0.0)
			build_trajectory_sinusoidal_phi();
		else
			build_trajectory_trapezoidal_v_phi();
		first_time = 0;

		carmen_behavior_selector_state_message msg_behavior;
		msg_behavior.low_level_state = Free_Running;
		msg_behavior.low_level_state_flags = 0;
		msg_behavior.host = carmen_get_host();
		msg_behavior.timestamp = carmen_get_time();
		publish_current_state(&msg_behavior);

		tune_pid_gain_velocity_parameters_message *pid_vel_msg = fill_pid_gain_velocity_parameters_message();
		carmen_ford_escape_publish_tune_pid_gain_velocity_parameters_message(pid_vel_msg , carmen_get_time());
		tune_pid_gain_steering_parameters_message *pid_steer_msg = fill_pid_gain_steering_parameters_message();
		carmen_ford_escape_publish_tune_pid_gain_steering_parameters_message(pid_steer_msg , carmen_get_time());
	}
}

static void
read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
			{"robot", "max_velocity", CARMEN_PARAM_DOUBLE, &motion_planner_config.max_v, 1, NULL},
			{"robot", "maximum_acceleration_forward", CARMEN_PARAM_DOUBLE, &motion_planner_config.maximum_acceleration_forward, 1, NULL},
			{"robot", "understeer_coeficient", CARMEN_PARAM_DOUBLE, &motion_planner_config.understeer_coeficient, 1, NULL},
			{"robot", "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &motion_planner_config.distance_between_front_and_rear_axles, 1, NULL},
			{"commandline", "max_v", CARMEN_PARAM_DOUBLE, &max_v, 0, NULL},
			{"commandline", "max_phi", CARMEN_PARAM_DOUBLE, &max_phi, 0, NULL},
			{"commandline", "frequency", CARMEN_PARAM_DOUBLE, &frequency, 0, NULL},
			{"commandline", "timer_period", CARMEN_PARAM_DOUBLE, &timer_period, 0, NULL},
			{"commandline", "wave_form", CARMEN_PARAM_INT, &wave_form, 0, NULL},

			{"commandline", "t1", CARMEN_PARAM_DOUBLE, &t1, 0, NULL},
			{"commandline", "t2", CARMEN_PARAM_DOUBLE, &t2, 0, NULL},
			{"commandline", "t3", CARMEN_PARAM_DOUBLE, &t3, 0, NULL},
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);

	carmen_param_allow_unfound_variables(1);

	carmen_param_install_params(argc, argv, param_list, num_items);

	max_phi = carmen_degrees_to_radians(max_phi);
}


static void
define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, IPC_VARIABLE_LENGTH, CARMEN_BASE_ACKERMAN_MOTION_COMMAND_FMT);

	carmen_test_ipc_exit(err, "Could not define message", CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME);
}


void
base_ackerman_odometry_handler(carmen_base_ackerman_odometry_message *msg)
{
	int base_ackerman_odometry_index = (base_ackerman_odometry_index + 1) % BASE_ACKERMAN_ODOMETRY_VECTOR_SIZE;
	//base_ackerman_odometry_vector[base_ackerman_odometry_index] = *msg;
}



EFFORT_SPLINE_DESCRIPTOR
calibrate_PID_parameters(PARAMS *params, EFFORT_SPLINE_DESCRIPTOR seed)
{
	gsl_vector *x;
	gsl_multimin_function_fdf my_func;
	size_t iter = 0;
	int status;

	my_func.n = 3;
	my_func.f = my_f_new;
	my_func.df = my_df_new;
	my_func.fdf = my_fdf_new;
	my_func.params = params;

	x = gsl_vector_alloc (3);  // Num of parameters to minimize
	gsl_vector_set(x, 0, seed.k1);
	gsl_vector_set(x, 1, seed.k2);
	gsl_vector_set(x, 2, seed.k3);

	const gsl_multimin_fdfminimizer_type *T = gsl_multimin_fdfminimizer_conjugate_fr;
	gsl_multimin_fdfminimizer *s = gsl_multimin_fdfminimizer_alloc(T, 4);

	gsl_multimin_fdfminimizer_set(s, &my_func, x, 0.01, 0.0001);

	do
	{
		iter++;
		status = gsl_multimin_fdfminimizer_iterate(s);

		if (status)
			break;

		status = gsl_multimin_test_gradient(s->gradient, 1e-3);

	} while ((status == GSL_CONTINUE) && (iter < 15));

	//printf("iter = %ld\n", iter);

	seed.k1 = carmen_clamp(-100.0, gsl_vector_get(s->x, 0), 100.0);
	seed.k2 = carmen_clamp(-100.0, gsl_vector_get(s->x, 1), 100.0);
	seed.k3 = carmen_clamp(-100.0, gsl_vector_get(s->x, 2), 100.0);
	seed.k4 = carmen_clamp(-100.0, gsl_vector_get(s->x, 3), 100.0);

	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

	return (seed);
}


int
main(int argc, char **argv)
{
	signal(SIGINT, signal_handler);

	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	define_messages();

	read_parameters(argc, argv);

	static PARAMS params;
	static EFFORT_SPLINE_DESCRIPTOR seed = {0.0, 0.0, 0.0, 0.0};
	
	//seed = calibrate_PID_parameters(&params, seed);

	/*
	tune_pid_gain_velocity_parameters_message *pid_vel_msg = fill_pid_gain_velocity_parameters_message();
	carmen_ford_escape_publish_tune_pid_gain_velocity_parameters_message(pid_vel_msg , carmen_get_time());
	tune_pid_gain_steering_parameters_message *pid_steer_msg = fill_pid_gain_steering_parameters_message();
	carmen_ford_escape_publish_tune_pid_gain_steering_parameters_message(pid_steer_msg , carmen_get_time());
	*/

	carmen_ipc_addPeriodicTimer(timer_period, (TIMER_HANDLER_TYPE) timer_handler, NULL);
	
	carmen_ipc_dispatch();

	return 0;
}
