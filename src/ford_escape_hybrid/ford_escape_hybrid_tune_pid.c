#include <carmen/carmen.h>
#include <carmen/motion_planner.h>

typedef double (*MotionControlFunction)(double v, double w, double t);

static carmen_robot_ackerman_config_t motion_planner_config;

static carmen_ackerman_motion_command_t motion_commands_vector[NUM_MOTION_COMMANDS_PER_VECTOR];

static double max_v = 0.0;
static double max_phi = 0.0;
static double frequency = 0.0;
static double timer_period = 1.0;

static double t1, t2, t3;

static MotionControlFunction v_function;
static MotionControlFunction phi_function;

static int wave_form = 0;

static void
signal_handler(int signo __attribute__ ((unused)) )
{
	carmen_ipc_disconnect();
	
	exit(0);
}


static void
send_trajectory_to_robot()
{
//	int i;
	
//	for (i = 0; i < NUM_MOTION_COMMANDS_PER_VECTOR; i++)
//		printf("v = %lf, phi = %lf, t = %lf\n", motion_commands_vector[i].v, motion_commands_vector[i].phi, motion_commands_vector[i].time);
	carmen_base_ackerman_publish_motion_command(motion_commands_vector, NUM_MOTION_COMMANDS_PER_VECTOR, carmen_get_time());
}

double
exponential_function(double v, double tau_inv, double t)
{
	return v * exp(-t * tau_inv);
}

double
sinusoidal_function(double v, double w, double t)
{
	return (v/2.0) * sin(2.0 * M_PI * w * t - M_PI/2.0) + v/2.0;
}

double
sinusoidal_function2(double phi, double w, double t)
{
	return (phi * sin(2.0 * M_PI * w * t));
}

double
quadratic_function(double v, double w, double t)
{
	if (sin(2.0 * M_PI * w * t) > 0.0)
		return v;
	else
		return 0.05;
}

double
quadratic_function2(double phi, double w, double t)
{
	if (sin(2.0 * M_PI * w * t) > 0.0)
		return phi;
	else
		return -phi;
}

double
quadratic_function3(double phi, double w, double t)
{
	if (sin(2.0 * M_PI * w * t) > 0.0)
		return phi;
	else
		return phi;
}


double
identity_function(double phi, double w, double t)
{
	(void)w;
	(void)t;
	return phi;
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
build_trajectory_stop_smoothing(double initial_dt, double final_dt)
{
	int i;
	double v;
	double phi = 0.0;
	double t = 0.0;
	double tau = final_dt / 3.0;
	double time_first_command_sent;
	int commands_trajectory_1 = 50;
	int commands_trajectory_2 = NUM_MOTION_COMMANDS_PER_VECTOR-commands_trajectory_1;

	timer_period = initial_dt / (double) commands_trajectory_1;
	for (i = 0; i < commands_trajectory_1; i++)
	{
		motion_commands_vector[i].v = max_v;
		motion_commands_vector[i].phi = phi;
		motion_commands_vector[i].time = timer_period;
	}

	timer_period = final_dt / (double) commands_trajectory_2;
	for (i = commands_trajectory_1; i < NUM_MOTION_COMMANDS_PER_VECTOR; i++)
	{
		t+=timer_period;

		v = exponential_function(max_v, 1.0/tau, t);

		motion_commands_vector[i].v = v;
		motion_commands_vector[i].phi = phi;
		motion_commands_vector[i].time = 1.0 * timer_period;
	}

	i = 0;
	//send trajetory
	do
	{
		time_first_command_sent = carmen_get_time();

		carmen_warn("t = %lf, v = %lf, phi = %lf, time = %lf\n", t,
				motion_commands_vector[0].v,
				motion_commands_vector[0].phi,
				motion_commands_vector[0].time);

		do {
			carmen_base_ackerman_publish_motion_command(&motion_commands_vector[i], 1, carmen_get_time());
			carmen_ipc_sleep(fmin(0.05, motion_commands_vector[i].time));
		} while ((time_first_command_sent + motion_commands_vector[i].time) < carmen_get_time());

		i++;
	} while(i < NUM_MOTION_COMMANDS_PER_VECTOR);

}


void
build_trajectory_stop_smooth_trajectory()
{
	double delta_t, t;
	int i;
	
	delta_t = (t1 + t2 + t3) / (double) (NUM_MOTION_COMMANDS_PER_VECTOR - 2);
	
	for (i = 0, t = 0.0; t < t1; t += delta_t, i++)
	{
		motion_commands_vector[i].v = t * (max_v / t1);
		motion_commands_vector[i].phi = 0.0;
		motion_commands_vector[i].time = delta_t;
	}	
	
	for (t = 0.0; t < t2; t += delta_t, i++)
	{
		motion_commands_vector[i].v = max_v;
		motion_commands_vector[i].phi = 0.0;
		motion_commands_vector[i].time = delta_t;
	}	
	
	for (t = 0.0; t <= (t3 + delta_t); t += delta_t, i++)
	{
		//3 * exp(-((10 - x) * (10 - x)) / (4 * 4))
		motion_commands_vector[i].v = max_v - t * (max_v / t3);
//		motion_commands_vector[i].v = max_v * exp(-(t * t) / ((t3 * t3) / (2.5 * 2.5)));
		motion_commands_vector[i].phi = 0.0;
		motion_commands_vector[i].time = delta_t;
	}
//	printf("i = %d, NUM_MOTION_COMMANDS_PER_VECTOR = %d\n", i, NUM_MOTION_COMMANDS_PER_VECTOR);
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

	for (i = 0, t = 0.0; t < t0; t += delta_t, i++)
	{
		motion_commands_vector[i].v = 0.0;
		motion_commands_vector[i].phi = 0.0;
		motion_commands_vector[i].time = delta_t;
	}

	for (t = 0.0; t < t1; t += delta_t, i++)
	{
		motion_commands_vector[i].v = t * (max_v / t1);
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
		motion_commands_vector[i].v = max_v - t * (max_v / t3);
		motion_commands_vector[i].phi = max_phi - t * (max_phi / t3);;
		motion_commands_vector[i].time = delta_t;
	}

	for (t = 0.0; t <= (t4 + delta_t); t += delta_t, i++)
	{
		motion_commands_vector[i].v = 0.0;
		motion_commands_vector[i].phi = 0.0;
		motion_commands_vector[i].time = delta_t;
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

	delta_t = t1 / (double) (NUM_MOTION_COMMANDS_PER_VECTOR - 2);

	for (i = 0, t = 0.0; i < NUM_MOTION_COMMANDS_PER_VECTOR; t += delta_t, i++)
	{
		motion_commands_vector[i].v = max_v;

		motion_commands_vector[i].phi = max_phi * sin(frequency * t);
		motion_commands_vector[i].time = delta_t;

		printf("%lf   %lf\n", frequency * t, motion_commands_vector[i].phi);
	}

	printf("i = %d, NUM_MOTION_COMMANDS_PER_VECTOR = %d\n", i, NUM_MOTION_COMMANDS_PER_VECTOR);
	send_trajectory_to_robot();
}


static void
timer_handler()
{
	static int first_time = 1;
	
	if (first_time)
	{
//		build_trajectory_trapezoidal_phi();
//		build_trajectory_sinusoidal_phi();
		build_trajectory_trapezoidal_v_phi();
		first_time = 0;
	}
}


void
timer_handler_general()
{
	static int first_time = 1;
	static double initial_t;
	
	if (first_time)
	{
		initial_t = carmen_get_time();
		first_time = 0;
	}
	build_trajectory(carmen_get_time() - initial_t);
}

void
vel_handler()
{
	static int count = 0;

	switch(count)
	{
	case 0:
		max_v = -2.5;
		break;
	case 1:
		max_v = -3.0;
		break;
	case 2:
		max_v = -2.5;
		break;
	case 3:
		max_v = -2.5;
		break;
	case 4:
		max_v = -1.5;
		break;
	case 5:
		max_v = 0.0;
		break;
	case 6:
		max_v = 0.0;
		break;
	case 7:
		max_v = 0.0;
		break;
	case 8:
		max_v = 0.0;
		break;
	case 9:
		max_v = 0.0;
		break;
	case 10:
		max_v = 0.0;
		break;
	default:
		exit(0);
		break;
	}

	count++;

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

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME,
			IPC_VARIABLE_LENGTH, CARMEN_BASE_ACKERMAN_MOTION_COMMAND_FMT);

	carmen_test_ipc_exit(err, "Could not define message",
			CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME);
}

//static void
//select_wave_form()
//{
//	switch (wave_form)
//	{
//	case 0:
//		v_function = sinusoidal_function2;
//		phi_function = sinusoidal_function2;
//		break;
//	case 1:
//		v_function = quadratic_function;
//		phi_function = quadratic_function2;
//		break;
//	case 2:
//		v_function = quadratic_function;
//		phi_function = quadratic_function3;
//		break;
//	default:
//		v_function = identity_function;
//		phi_function = identity_function;
//	}
//}

int
main(int argc, char **argv) //./ford_escape_hybrid_train_base -max_v 5.0 -max_phi 5.0 -timer_period 1.0 -t1 4.0 -t2 2.0 -t3 4.0
{
	signal(SIGINT, signal_handler);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	define_messages();

	read_parameters(argc, argv);

//	select_wave_form();

	carmen_ipc_addPeriodicTimer(timer_period, timer_handler, NULL);
	//carmen_ipc_addPeriodicTimer(timer_period, timer_handler_general, NULL);
	//carmen_ipc_addPeriodicTimer(4.0, vel_handler, NULL);

	carmen_ipc_dispatch();

	return 0;
}
