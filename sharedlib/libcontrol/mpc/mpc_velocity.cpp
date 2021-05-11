#include "mpc.h"
#include "../control.h"


double
get_velocity_vector_from_spline_descriptors(EFFORT_SPLINE_DESCRIPTOR *descriptors, void *params_ptr)
{
	PARAMS *params = (PARAMS *) params_ptr;

	double velocity, throttle_effort, brake_effort, velocity_eror_sum=0.0, current_velocity = params->current_velocity;

	fann_type velocity_ann_input[NUM_VELOCITY_ANN_INPUTS];
	memcpy(velocity_ann_input, params->velocity_ann_input, NUM_VELOCITY_ANN_INPUTS * sizeof(fann_type));

	vector<double> effort_vector = get_effort_vector_from_spline_descriptors(descriptors, VELOCITY_PREDICTION_HORIZON);

	params->optimized_path.v.clear();

	for (unsigned int i = 0; i < effort_vector.size(); i++)
	{
		if (effort_vector[i] >= 0.0)        // If the effort is greater than 0, the car must accelerate, else it must breaks
		{
			throttle_effort = effort_vector[i];
			brake_effort = 17.0;
		}
		else
		{
			throttle_effort = 0.0;
			brake_effort = effort_vector[i];
		}
		velocity = carmen_libcarneuralmodel_compute_new_velocity_from_efforts(velocity_ann_input, params->velocity_ann, throttle_effort, brake_effort, current_velocity);

		velocity_eror_sum += fabs(params->path.v[i] - velocity);

		params->optimized_path.v.push_back(velocity);

		current_velocity = velocity;

		//printf("%lf %lf\n", throttle_effort, brake_effort);
		//printf("D %lf P %lf\n", params->path.v[i], velocity);
	}

	return (velocity_eror_sum);
}


double
cost_function(const gsl_vector *v, void *params_ptr)
{
	EFFORT_SPLINE_DESCRIPTOR d;
	PARAMS *params = (PARAMS *) params_ptr;

	d.k1 = gsl_vector_get(v, 0);
	d.k2 = gsl_vector_get(v, 1);
	d.k3 = gsl_vector_get(v, 2);
	d.k4 = gsl_vector_get(v, 3);

	// This vector may contain velocity values or phi values depending on what is been optimized
	double error_sum = params->get_vector_function(&d, params);

	double cost = error_sum;// + 0.00011 * sqrt((params->previous_k1 - d.k1) * (params->previous_k1 - d.k1));

	return (cost);
}


void
derivative_cost_function(const gsl_vector *v, void *params, gsl_vector *df)
{
	EFFORT_SPLINE_DESCRIPTOR d;
	double h = 0.1;
	double f_x = cost_function(v, params);
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
	double f_k1_h = cost_function(x_h, params);
	double df_k1_h = (f_k1_h - f_x) / h;

	gsl_vector_set(x_h, 0, d.k1);
	gsl_vector_set(x_h, 1, d.k2 + h);
	gsl_vector_set(x_h, 2, d.k3);
	gsl_vector_set(x_h, 3, d.k4);
	double f_k2_h = cost_function(x_h, params);
	double df_k2_h = (f_k2_h - f_x) / h;

	gsl_vector_set(x_h, 0, d.k1);
	gsl_vector_set(x_h, 1, d.k2);
	gsl_vector_set(x_h, 2, d.k3 + h);
	gsl_vector_set(x_h, 3, d.k4);
	double f_k3_h = cost_function(x_h, params);
	double df_k3_h = (f_k3_h - f_x) / h;

	gsl_vector_set(x_h, 0, d.k1);
	gsl_vector_set(x_h, 1, d.k2);
	gsl_vector_set(x_h, 2, d.k3);
	gsl_vector_set(x_h, 3, d.k4 + h);
	double f_k4_h = cost_function(x_h, params);
	double df_k4_h = (f_k4_h - f_x) / h;

	gsl_vector_set(df, 0, df_k1_h);
	gsl_vector_set(df, 1, df_k2_h);
	gsl_vector_set(df, 2, df_k3_h);
	gsl_vector_set(df, 3, df_k4_h);

	gsl_vector_free(x_h);
}


void
fdf(const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
	*f = cost_function (x, params);
	derivative_cost_function (x, params, df);
}


EFFORT_SPLINE_DESCRIPTOR
get_optimized_effort(PARAMS *params, EFFORT_SPLINE_DESCRIPTOR descriptors, double (*function)(EFFORT_SPLINE_DESCRIPTOR *, void *))
{
	params->get_vector_function = function;

	gsl_vector *x;
	gsl_multimin_function_fdf my_func;
	size_t iter = 0;
	int status;

	my_func.n = 4;
	my_func.f = cost_function;
	my_func.df = derivative_cost_function;
	my_func.fdf = fdf;
	my_func.params = params;

	x = gsl_vector_alloc (4);  // Num of parameters to minimize
	gsl_vector_set(x, 0, descriptors.k1);
	gsl_vector_set(x, 1, descriptors.k2);
	gsl_vector_set(x, 2, descriptors.k3);
	gsl_vector_set(x, 3, descriptors.k4);

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
	printf("iter = %ld\n", iter);

	descriptors.k1 = carmen_clamp(-100.0, gsl_vector_get(s->x, 0), 100.0);
	descriptors.k2 = carmen_clamp(-100.0, gsl_vector_get(s->x, 1), 100.0);
	descriptors.k3 = carmen_clamp(-100.0, gsl_vector_get(s->x, 2), 100.0);
	descriptors.k4 = carmen_clamp(-100.0, gsl_vector_get(s->x, 3), 100.0);

	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

	return (descriptors);
}


double
get_point_aproximating_by_line(double x1, double y1, double x2, double y2, double x_middle)
{
	// Line Equation a = yA - yB, b = xB - xA, c = xAyB - xByA
	double a = y1 - y2;
	double b = x2 - x1;
	double c = (x1 * y2) - (x2 * y1);

	double y_middle = ((-a * x_middle) -c) / b;

	return (y_middle);
}


MOTION_COMMAND
get_motion_commands_vector(carmen_robot_and_trailer_motion_command_t *current_motion_command_vector, int nun_motion_commands, double time_of_velodyne_message, double prediction_horizon)
{
	MOTION_COMMAND commands;
	double elapsed_time = carmen_get_time() - time_of_velodyne_message;
	double sum_of_motion_commands_vector_time = current_motion_command_vector[0].time;
	int j = 0;
	double time_interval = 0.0;
	double total_time = 0.0;

	// Removes the elapsed time from the motion commands vector
	while ((sum_of_motion_commands_vector_time < elapsed_time) && (j < nun_motion_commands)) // TODO Tratar se sair por j <nun_motion_commands
	{
		sum_of_motion_commands_vector_time += current_motion_command_vector[j].time;
		j++;
	}
	sum_of_motion_commands_vector_time -= current_motion_command_vector[j].time;
	current_motion_command_vector[j].time -= (elapsed_time - sum_of_motion_commands_vector_time);

	// Build motion command vector with DELTA_T time interval
	while (total_time < prediction_horizon)
	{
		commands.x.push_back(get_point_aproximating_by_line(0.0, current_motion_command_vector[j].x, current_motion_command_vector[j].time, current_motion_command_vector[j+1].x, time_interval));
		commands.y.push_back(get_point_aproximating_by_line(0.0, current_motion_command_vector[j].y, current_motion_command_vector[j].time, current_motion_command_vector[j+1].y, time_interval));
		commands.v.push_back(get_point_aproximating_by_line(0.0, current_motion_command_vector[j].v, current_motion_command_vector[j].time, current_motion_command_vector[j+1].v, time_interval));
		commands.phi.push_back(get_point_aproximating_by_line(0.0, current_motion_command_vector[j].phi, current_motion_command_vector[j].time, current_motion_command_vector[j+1].phi, time_interval));

		time_interval += DELTA_T;
		total_time += DELTA_T;
		if (time_interval > current_motion_command_vector[j].time)
		{
			j++;
			if (j >= nun_motion_commands)
				break;

			time_interval = time_interval - current_motion_command_vector[j].time;
		}
	}
//	for (j = 0, sum_of_motion_commands_vector_time = 0.0; (unsigned)j < commands.v.size(); j++)
//	{
//		sum_of_motion_commands_vector_time += DELTA_T;
////		printf("------%lf %lf %lf\n", sum_of_motion_commands_vector_time, commands.phi[j], commands.v[j]);
//		printf("------%lf %lf %lf\n", sum_of_motion_commands_vector_time, commands.x[j], commands.y[j]);
//	}
//	printf("\n\n");

	return commands;
}


bool
init_mpc(PARAMS &params, EFFORT_SPLINE_DESCRIPTOR &seed, double current_velocity, double time_of_last_motion_command,
		carmen_robot_ackerman_config_t *robot_config)
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
		carmen_libcarneuralmodel_init_velocity_ann_input(params.velocity_ann_input);

		first_time = false;
	}

	seed = {0.0, 0.0, 0.0, 0.0};

	params.current_velocity = current_velocity;
	params.understeer_coeficient = robot_config->understeer_coeficient;
	params.distance_rear_axles = robot_config->distance_between_front_and_rear_axles;
	params.max_phi = robot_config->max_phi;
	params.time_elapsed_since_last_motion_command = carmen_get_time() - time_of_last_motion_command;

	return (true);
}


void
carmen_libmpc_compute_velocity_effort(double *throttle_command, double *brake_command, int *gear_command,
		carmen_robot_and_trailer_motion_command_t *current_motion_command_vector, int nun_motion_commands,
		double current_velocity, double time_of_last_motion_command, carmen_robot_ackerman_config_t *robot_config)
{
	if (current_motion_command_vector == NULL || nun_motion_commands < 16)
		return;

	static PARAMS params;
	static EFFORT_SPLINE_DESCRIPTOR velocity_descriptors = {0.0, 0.0, 0.0, 0.0};
	double velocity;//throttle_effort, brake_effort,

	if (!init_mpc(params, velocity_descriptors, current_velocity, time_of_last_motion_command, robot_config))
		return;

	params.path = get_motion_commands_vector(current_motion_command_vector, nun_motion_commands, time_of_last_motion_command, VELOCITY_PREDICTION_HORIZON);
	velocity_descriptors = get_optimized_effort(&params, velocity_descriptors, get_velocity_vector_from_spline_descriptors);

	*gear_command = *gear_command;
	if(velocity_descriptors.k1 >= 0.0)
	{
		*throttle_command = velocity_descriptors.k1;
		*brake_command = 17.0;
	}
	else
	{
		*throttle_command = 0.0;
		*brake_command = velocity_descriptors.k1;
	}

	velocity = carmen_libcarneuralmodel_compute_new_velocity_from_efforts(params.velocity_ann_input, params.velocity_ann, *throttle_command, *brake_command, current_velocity);
	params.velocity_error_dk = current_velocity - velocity;


//	#ifdef PLOT
//		plot_velocity(&velocity_descriptors, current_velocity, &params, PREDICTION_HORIZON);
//	#endif
}
