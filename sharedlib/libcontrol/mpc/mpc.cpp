#include "mpc.h"
#include "../control.h"


using namespace std;


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
my_f(const gsl_vector *v, void *params_ptr)
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
my_df(const gsl_vector *v, void *params, gsl_vector *df)
{
	EFFORT_SPLINE_DESCRIPTOR d;
	double h = 0.1;
	double f_x = my_f(v, params);
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
	double f_k1_h = my_f(x_h, params);
	double df_k1_h = (f_k1_h - f_x) / h;

	gsl_vector_set(x_h, 0, d.k1);
	gsl_vector_set(x_h, 1, d.k2 + h);
	gsl_vector_set(x_h, 2, d.k3);
	gsl_vector_set(x_h, 3, d.k4);
	double f_k2_h = my_f(x_h, params);
	double df_k2_h = (f_k2_h - f_x) / h;

	gsl_vector_set(x_h, 0, d.k1);
	gsl_vector_set(x_h, 1, d.k2);
	gsl_vector_set(x_h, 2, d.k3 + h);
	gsl_vector_set(x_h, 3, d.k4);
	double f_k3_h = my_f(x_h, params);
	double df_k3_h = (f_k3_h - f_x) / h;

	gsl_vector_set(x_h, 0, d.k1);
	gsl_vector_set(x_h, 1, d.k2);
	gsl_vector_set(x_h, 2, d.k3);
	gsl_vector_set(x_h, 3, d.k4 + h);
	double f_k4_h = my_f(x_h, params);
	double df_k4_h = (f_k4_h - f_x) / h;

	gsl_vector_set(df, 0, df_k1_h);
	gsl_vector_set(df, 1, df_k2_h);
	gsl_vector_set(df, 2, df_k3_h);
	gsl_vector_set(df, 3, df_k4_h);

	gsl_vector_free(x_h);
}


void
my_fdf(const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
	*f = my_f (x, params);
	my_df (x, params, df);
}


EFFORT_SPLINE_DESCRIPTOR
get_optimized_effort(PARAMS *params, EFFORT_SPLINE_DESCRIPTOR seed)
{
	gsl_vector *x;
	gsl_multimin_function_fdf my_func;
	size_t iter = 0;
	int status;

	my_func.n = 4;
	my_func.f = my_f;
	my_func.df = my_df;
	my_func.fdf = my_fdf;
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


void
plot_state(EFFORT_SPLINE_DESCRIPTOR *seed, PARAMS *params, double v, double understeer_coeficient, double distance_between_front_and_rear_axles, double effort)
{
	#define PAST_SIZE 300
	static list<double> cphi_vector;
	static list<double> dphi_vector;
	static list<double> timestamp_vector;
	static list<double> effort_vector;
	static bool first_time = true;
	static double first_timestamp;
	static FILE *gnuplot_pipe;

	double t = carmen_get_time();

	if (first_time)
	{
		first_timestamp = t;
		first_time = false;

		gnuplot_pipe = popen("gnuplot", "w"); // -persist to keep last plot after program closes
		fprintf(gnuplot_pipe, "set xrange [0:PAST_SIZE/20]\n");
		fprintf(gnuplot_pipe, "set yrange [-110.0:110.0]\n");
		fprintf(gnuplot_pipe, "set y2range [-0.55:0.55]\n");
		fprintf(gnuplot_pipe, "set xlabel 'senconds'\n");
		fprintf(gnuplot_pipe, "set ylabel 'effort'\n");
		fprintf(gnuplot_pipe, "set y2label 'phi (radians)'\n");
		fprintf(gnuplot_pipe, "set ytics nomirror\n");
		fprintf(gnuplot_pipe, "set y2tics\n");
		fprintf(gnuplot_pipe, "set tics out\n");
	}

	unsigned int timed_index_to_motion_command = get_motion_timed_index_to_motion_command(params);
	dphi_vector.push_front(params->motion_commands_vector[timed_index_to_motion_command].phi);
	cphi_vector.push_front(carmen_get_phi_from_curvature(params->atan_current_curvature, v, understeer_coeficient, distance_between_front_and_rear_axles));
	timestamp_vector.push_front(t - first_timestamp);
	effort_vector.push_front(effort);

	while (cphi_vector.size() > PAST_SIZE)
	{
		cphi_vector.pop_back();
		dphi_vector.pop_back();
		timestamp_vector.pop_back();
		effort_vector.pop_back();
	}

	FILE *gnuplot_data_file = fopen("gnuplot_data.txt", "w");

	// Dados passados
	list<double>::reverse_iterator it_cphi, it_dphi, it_timestamp, it_effort;
	for (it_cphi = cphi_vector.rbegin(), it_dphi = dphi_vector.rbegin(), it_timestamp = timestamp_vector.rbegin(), it_effort = effort_vector.rbegin();
		 it_cphi != cphi_vector.rend();
		 it_cphi++, it_dphi++, it_timestamp++, it_effort++)
	{
		fprintf(gnuplot_data_file, "%lf %lf %lf %lf %d %d\n", *it_timestamp - timestamp_vector.back(), *it_cphi, *it_dphi, *it_effort, 1, 2); //1-red 2-green 3-blue 4-magenta 5-lightblue 6-yellow 7-black 8-orange 9-grey
	}
	// Dados futuros
	vector<double> phi_vector = get_phi_vector_from_spline_descriptors(seed, params);
	vector<double> future_effort_vector = get_effort_vector_from_spline_descriptors(seed, PREDICTION_HORIZON);

	double delta_t = DELTA_T;
	double phi_vector_time = 0.0;
	double begin_predition_time = timestamp_vector.front() - timestamp_vector.back();

	double motion_commands_vector_time = params->motion_commands_vector[timed_index_to_motion_command].time;
	for (unsigned int i = 0; i < phi_vector.size(); i++)
	{
		phi_vector_time += delta_t;

		fprintf(gnuplot_data_file, "%lf %lf %lf %lf %d %d\n",
				begin_predition_time + phi_vector_time, phi_vector[i], params->motion_commands_vector[timed_index_to_motion_command].phi,
				future_effort_vector[i], 8, 2);

		if (phi_vector_time > motion_commands_vector_time)
		{
			timed_index_to_motion_command++;
			if (timed_index_to_motion_command >= params->motion_commands_vector_size)
				break;
			motion_commands_vector_time += params->motion_commands_vector[timed_index_to_motion_command].time;
		}
	}

//	double time = begin_predition_time + motion_commands_vector_time;
//	for (; timed_index_to_motion_command < params->motion_commands_vector_size; timed_index_to_motion_command++)
//	{
//		begin_predition_time += params->motion_commands_vector[timed_index_to_motion_command].time;
//	}
	fclose(gnuplot_data_file);

	fprintf(gnuplot_pipe, "unset arrow\nset arrow from %lf, graph 0 to %lf, graph 1 nohead\n", begin_predition_time, begin_predition_time);

	fprintf(gnuplot_pipe, "plot "
			"'./gnuplot_data.txt' using 1:2:5 with lines linecolor variable title 'cphi' axes x1y2,"
			"'./gnuplot_data.txt' using 1:3:6 with lines linecolor variable title 'dphi' axes x1y2,"
			"'./gnuplot_data.txt' using 1:4 with lines title 'effort' axes x1y1\n");

	fflush(gnuplot_pipe);
}


int
libmpc_stiction_simulation(double effort, double v)
{
	static double first_eff;
	static unsigned int cont = 0;

	if (cont <= 40)
	{
		if (effort > -5 && effort < 5)
			cont++;
		else
			cont = 0;

		first_eff = effort;

		return (false);
	}
	else
	{
		if (fabs(first_eff - effort) < (3*v))
		{
			//printf("Entrou\n");
			//printf("Stic f%lf c%lf abs%lf %lf\n", first_eff, effort, fabs(first_eff - effort), (3*v));
			return (true);
		}
		else
		{
			printf("Saiu\n");
			cont = 0;
			return (false);
		}
	}
}


int
libmpc_stiction_simulation_new(double effort, double v)
{
	static double previous_effort = 0.0;
	bool has_stiction;

	if (fabs(effort - previous_effort) < (0.03 * v))
		has_stiction = true;
	else
		has_stiction = false;

	previous_effort = effort;

	return (has_stiction);
}


// STICTION CORRECTION PARAMETERS
#define STEERING_ADDITIVE_BUMP 3.0
#define BUMP_SLEEP_TIME 30.0
#define NUMBER_CICLES_APPLYING_BUMP 2
#define MAX_CUMULATIVE_STEERING_ERROR 0.001
#define MAX_CUMULATIVE_EFFORT_ERROR 15.0
#define MIN_VELOCITY_TO_CHECK_STICTION 8.4


double
stiction_correction(double current_phi, double desired_phi, double effort, double v)
{
	if (v < MIN_VELOCITY_TO_CHECK_STICTION)
		return (0.0);

	static unsigned int cont = 0;
	static unsigned int wait = 0;

	static double last_eff = 0.0;
	static double dif_phi = 0.0;
	static double dif_eff = 0.0;

	dif_phi += desired_phi - current_phi;
	dif_eff += fabs(fabs(effort) - fabs(last_eff));

	if (effort < 5 && effort > -5)
	{
		dif_phi = 0.0;
		dif_eff = 0.0;
		cont = 0;
	}
	else if (fabs(dif_phi) > MAX_CUMULATIVE_STEERING_ERROR && dif_eff > MAX_CUMULATIVE_EFFORT_ERROR && cont < NUMBER_CICLES_APPLYING_BUMP && wait == 0)
	{
		//printf("Aplicou\n");
		//printf("dp%lf de%lf ef%lf s%f\n", dif_phi, dif_eff,  effort, (STEERING_ADDITIVE_BUMP * v));
		cont++;
		if (effort > 0)
			return (STEERING_ADDITIVE_BUMP * v);
		else
			return (-STEERING_ADDITIVE_BUMP * v);
	}
	else
	{
		wait++;
		if (wait > BUMP_SLEEP_TIME)
			wait = 0;

		return (0.0);
	}

	return (0.0);
}


double
get_point_between_points_aproximating_with_line(double x1, double y1, double x2, double y2, double x_middle)
{
	// Line Equation a = yA - yB, b = xB - xA, c = xAyB - xByA
	double a = y1 - y2;
	double b = x2 - x1;
	double c = (x1 * y2) - (x2 * y1);

	double y_middle = ((-a * x_middle) -c) / b;

	return (y_middle);
}


bool
init_mpc(bool &first_time, PARAMS &params, EFFORT_SPLINE_DESCRIPTOR &seed, double atan_current_curvature,
		carmen_robot_and_trailers_motion_command_t *current_motion_command_vector, int nun_motion_commands,
		double v, double time_of_last_motion_command,
		carmen_robot_ackerman_config_t *robot_config,
		int initialize_neural_networks)
{
	if (first_time)
	{
		params.steering_ann = fann_create_from_file("steering_ann.net");
		if (params.steering_ann == NULL)
		{
			printf("Error: Could not create steering_ann in carmen_libmpc_get_optimized_steering_effort_using_MPC()\n");
			exit(1);
		}
		carmen_libcarneuralmodel_init_steering_ann_input(params.steering_ann_input);
		params.dk = 0.0;
		params.previous_k1 = 0.0;
		first_time = false;

	}

	if (current_motion_command_vector == NULL)
	{
		seed = {0.0, 0.0, 0.0, 0.0};

		return (false);
	}

	if (initialize_neural_networks)
	{
		carmen_libcarneuralmodel_init_steering_ann_input(params.steering_ann_input);
		seed = {0.0, 0.0, 0.0, 0.0};
	}

	params.motion_commands_vector = current_motion_command_vector;
	params.motion_commands_vector_size = nun_motion_commands;
	params.atan_current_curvature = atan_current_curvature;
	params.current_velocity = v;
	params.understeer_coeficient = robot_config->understeer_coeficient;
	params.distance_rear_axles = robot_config->distance_between_front_and_rear_axles;
	params.max_phi = robot_config->max_phi;
	params.time_elapsed_since_last_motion_command = carmen_get_time() - time_of_last_motion_command;

	return (true);
}


double
carmen_libmpc_get_optimized_steering_effort_using_MPC(double current_aoc,
		carmen_robot_and_trailers_motion_command_t *current_motion_command_vector,	int nun_motion_commands,
		double v, double yp, double time_of_last_motion_command,
		carmen_robot_ackerman_config_t *robot_config,
		int initialize_neural_networks)
{
	static PARAMS params;
	static EFFORT_SPLINE_DESCRIPTOR seed = {0.0, 0.0, 0.0, 0.0};
	static bool first_time = true;

	if (!init_mpc(first_time, params, seed, current_aoc, current_motion_command_vector, nun_motion_commands, v, time_of_last_motion_command,
					robot_config, initialize_neural_networks))
		return (0.0);

	//seed = get_optimized_effort(&params, seed); // TODO essa funcao vai aqui ou depois do car_model???
	double effort = seed.k1;

/////////////  LISTA TESTES MPC	 /////////////////////////////////////////////
////  Testar novamente MPC
////  Testar PID sem o Motion Planner
////  Verificar uso direto da curvatura dada pela torc no MPC e PID
////  Teste erro na previsão do modelo
//////////////////////////////////////////////////////////////////////////////
	double predicted_aoc = carmen_libcarneuralmodel_compute_new_aoc_from_effort(effort, current_aoc, params.steering_ann_input, params.steering_ann);
	double Cxk = carmen_get_phi_from_curvature(tan(predicted_aoc), v, params.understeer_coeficient, params.distance_rear_axles);
	Cxk = carmen_clamp(-params.max_phi, Cxk, params.max_phi);

	// Calculate dk for the next cycle, dk is the error between predicted phi and measured phi
	//double Cxk = car_steering_model(effort, current_aoc, params.current_velocity, params.steering_ann_input, &params);
	params.dk = yp - Cxk;
	params.previous_k1 = effort;

	//--------	Tentativa de correcao da oscilacao em velocidades altas  -------------------------------------------------
	// effort /= (1.0 / (1.0 + (v * v) / 200.5)); // boa
	//effort += carmen_uniform_random(-2.0, 2.0);

	//int index = get_motion_timed_index_to_motion_command(&params);
	//effort += stiction_correction(yp, current_motion_command_vector[index].phi, effort, v);
	//--------------------------------------------------------------------------------------------------------------------

	seed = get_optimized_effort(&params, seed);

	#ifdef PLOT
		plot_state(&seed, &params, v, robot_config->understeer_coeficient, robot_config->distance_between_front_and_rear_axles, effort);
	#endif

	carmen_clamp(-100.0, effort, 100.0);
	return (effort);
}
