#include <carmen/carmen.h>
#include <carmen/simulator_ackerman.h>
#include <list>
#include <vector>
#include <fann.h>
#include <fann_data.h>
#include <floatfann.h>
#include <fann_train.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_multimin.h>
#include <car_neural_model.h>
#include "mpc.h"

#define DELTA_T (1.0 / 40.0)

using namespace std;

list<double> past_motion_command_list;
vector<double> new_command_vector;


double
compute_total_trajectory_time(carmen_ackerman_motion_command_p current_motion_command_vector, int nun_motion_commands)
{
	double total_time = 0.0;

	for (int i = 0; i < nun_motion_commands; i++)
		total_time += current_motion_command_vector[i].time;

	return (total_time);
}


vector<double>
get_effort_vector_from_spline_descriptors(EFFORT_SPLINE_DESCRIPTOR *descriptors)
{
	double delta_t = DELTA_T;
	double total_time = delta_t * (2 * NUM_STEERING_ANN_INPUTS / 4); // Cada steering input da rede neural tem dois valores (ver rede neural)
	double x[4] = { 0.0, total_time / 3.0, 2.0 * total_time / 3.0, total_time };
	double y[4] = { descriptors->k1, descriptors->k2, descriptors->k3, descriptors->k4 };

	gsl_interp_accel *acc = gsl_interp_accel_alloc();
	const gsl_interp_type *type = gsl_interp_cspline;
	gsl_spline *phi_effort_spline = gsl_spline_alloc(type, 4);

	gsl_spline_init(phi_effort_spline, x, y, 4);

	vector<double> effort_vector;
	for (double t = 0.0; t < total_time; t += delta_t)
		effort_vector.push_back(gsl_spline_eval(phi_effort_spline, t, acc));

	gsl_spline_free(phi_effort_spline);
	gsl_interp_accel_free(acc);

	return (effort_vector);
}


vector<double>
get_phi_vector_from_spline_descriptors(EFFORT_SPLINE_DESCRIPTOR *descriptors, PARAMS *p)
{
	fann_type steering_ann_input[NUM_STEERING_ANN_INPUTS];
	memcpy(steering_ann_input, p->steering_ann_input, NUM_STEERING_ANN_INPUTS * sizeof(fann_type));

	vector<double> effort_vector = get_effort_vector_from_spline_descriptors(descriptors);

	vector<double> phi_vector;
	double current_atan_of_curvature = p->atan_current_curvature;
	for (unsigned int i = 0; i < effort_vector.size(); i++)
	{
		double effort = carmen_clamp(-100.0, effort_vector[i], 100.0);
		carmen_libcarneuralmodel_build_steering_ann_input(steering_ann_input, effort, current_atan_of_curvature, p->v);

		fann_type *steering_ann_output = fann_run(p->steering_ann, steering_ann_input);
		current_atan_of_curvature = steering_ann_output[0];

		double phi = carmen_get_phi_from_curvature(tan(current_atan_of_curvature), p->v, p->understeer_coeficient, p->distance_rear_axles);
		phi_vector.push_back(phi);
	}

	return (phi_vector);
}


double
my_f(const gsl_vector *v, void *params)
{
	EFFORT_SPLINE_DESCRIPTOR d;
	PARAMS *p = (PARAMS *) params;

	d.k1 = gsl_vector_get(v, 0);
	d.k2 = gsl_vector_get(v, 1);
	d.k3 = gsl_vector_get(v, 2);
	d.k4 = gsl_vector_get(v, 3);

	vector<double> phi_vector = get_phi_vector_from_spline_descriptors(&d, p);

	double delta_t = DELTA_T;
	double motion_commands_vector_time = p->motion_commands_vector[0].time;
	double phi_vector_time = 0.0;
	double error = 0.0;

	for (unsigned int i = 0, j = 0; i < phi_vector.size(); i++)
	{
		error += sqrt((phi_vector[i] - p->motion_commands_vector[j].phi) *
					  (phi_vector[i] - p->motion_commands_vector[j].phi));

		phi_vector_time += delta_t;
		if (phi_vector_time > motion_commands_vector_time)
		{
			j++;
			if (j >= p->motion_commands_vector_size)
				break;
			motion_commands_vector_time += p->motion_commands_vector[j].time;
		}
	}

	return (error);
}


void
my_df(const gsl_vector *v, void *params, gsl_vector *df)
{
	double h = 0.1;
	double f_x = my_f(v, params);
	gsl_vector *x_h;

	x_h = gsl_vector_alloc(4);

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0) + h);
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1));
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2));
	gsl_vector_set(x_h, 3, gsl_vector_get(v, 3));
	double f_k1_h = my_f(x_h, params);
	double df_k1_h = (f_k1_h - f_x) / h;

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0));
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1) + h);
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2));
	gsl_vector_set(x_h, 3, gsl_vector_get(v, 3));
	double f_k2_h = my_f(x_h, params);
	double df_k2_h = (f_k2_h - f_x) / h;

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0));
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1));
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2) + h);
	gsl_vector_set(x_h, 3, gsl_vector_get(v, 3));
	double f_k3_h = my_f(x_h, params);
	double df_k3_h = (f_k3_h - f_x) / h;

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0));
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1));
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2));
	gsl_vector_set(x_h, 3, gsl_vector_get(v, 3) + h);
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
get_optimized_effort(PARAMS *par, EFFORT_SPLINE_DESCRIPTOR seed)
{
	size_t iter = 0;
	int status;

	const gsl_multimin_fdfminimizer_type *T;
	gsl_multimin_fdfminimizer *s;

	gsl_vector *x;
	gsl_multimin_function_fdf my_func;

	my_func.n = 4;
	my_func.f = my_f;
	my_func.df = my_df;
	my_func.fdf = my_fdf;
	my_func.params = par;

	x = gsl_vector_alloc (4);  //Num of parameters to minimize
	gsl_vector_set(x, 0, seed.k1);
	gsl_vector_set(x, 1, seed.k2);
	gsl_vector_set(x, 2, seed.k3);
	gsl_vector_set(x, 3, seed.k4);

	T = gsl_multimin_fdfminimizer_conjugate_fr;
	s = gsl_multimin_fdfminimizer_alloc(T, 4);

	gsl_multimin_fdfminimizer_set(s, &my_func, x, 0.01, 0.0001);   //(function_fdf, gsl_vector, step_size, tol)

	do
	{
		iter++;
		status = gsl_multimin_fdfminimizer_iterate(s);

		if (status)
			break;

		status = gsl_multimin_test_gradient(s->gradient, 1e-3);

//		if (status == GSL_SUCCESS)
//			printf ("Minimum found at:\n");
	} while ((status == GSL_CONTINUE) && (iter < 999));

	//printf("iter = %ld\n", iter);

	// A seed inicia em zero e armazena o melhor conjunto de parametros do ciclo de otimizacao anterior
	seed.k1 = gsl_vector_get(s->x, 0);
	seed.k2 = gsl_vector_get(s->x, 1);
	seed.k3 = gsl_vector_get(s->x, 2);
	seed.k4 = gsl_vector_get(s->x, 3);

	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

	return (seed);
}


void
plot_state(EFFORT_SPLINE_DESCRIPTOR *seed, PARAMS *p, carmen_simulator_ackerman_config_t *simulator_config)
{
#define PAST_SIZE (NUM_STEERING_ANN_INPUTS * 2)
	static double cphi[PAST_SIZE];
	static double dphi[PAST_SIZE];
	static double timestamp[PAST_SIZE];
	static bool first_time = true;
	static double first_timestamp;
	static FILE *gnuplot_pipe;

	double t = carmen_get_time();
	if (first_time)
	{
		first_timestamp = t;
		first_time = false;

		gnuplot_pipe = popen("gnuplot -persist", "w");
		fprintf(gnuplot_pipe, "set xrange [0:18]\n");
		fprintf(gnuplot_pipe, "set yrange [-0.12:0.12]\n");
	}

	memmove(cphi, cphi + 1, (PAST_SIZE - 1) * sizeof(double));
	memmove(dphi, dphi + 1, (PAST_SIZE - 1) * sizeof(double));
	memmove(timestamp, timestamp + 1, (PAST_SIZE - 1) * sizeof(double));

	double v = simulator_config->v;
	double understeer_coeficient = simulator_config->understeer_coeficient;
	double distance_rear_axles = simulator_config->distance_between_front_and_rear_axles;
	cphi[PAST_SIZE - 1] = carmen_get_phi_from_curvature(p->atan_current_curvature, v, understeer_coeficient, distance_rear_axles);
	dphi[PAST_SIZE - 1] = carmen_get_phi_from_curvature(p->atan_desired_curvature, v, understeer_coeficient, distance_rear_axles);

	timestamp[PAST_SIZE - 1] = t - first_timestamp;

	if (t - first_timestamp > 16.0)
	{
		FILE *gnuplot_data_file = fopen("gnuplot_data.txt", "w");

		// Dados passados
		for (int i = 0; i < PAST_SIZE; i++)
			fprintf(gnuplot_data_file, "%lf %lf %lf\n",
					timestamp[i] - timestamp[0], cphi[i], dphi[i]);

		// Dados futuros
		vector<double> phi_vector = get_phi_vector_from_spline_descriptors(seed, p);

		double delta_t = DELTA_T;
		double motion_commands_vector_time = p->motion_commands_vector[0].time;
		double phi_vector_time = 0.0;
		double begin_predition_time = timestamp[PAST_SIZE-1] - timestamp[0];
		for (unsigned int i = 0, j = 0; i < phi_vector.size(); i++)
		{
			phi_vector_time += delta_t;
			fprintf(gnuplot_data_file, "%lf %lf %lf\n",
					(timestamp[PAST_SIZE-1] - timestamp[0]) + phi_vector_time, phi_vector[i], p->motion_commands_vector[j].phi);

			if (phi_vector_time > motion_commands_vector_time)
			{
				j++;
				if (j >= p->motion_commands_vector_size)
					break;
				motion_commands_vector_time += p->motion_commands_vector[j].time;
			}
		}

		fclose(gnuplot_data_file);

		// Plot dados passados e futuros

		fprintf(gnuplot_pipe, "unset arrow\nset arrow from %lf, %lf to %lf, %lf nohead\n",
				begin_predition_time, -0.3, begin_predition_time, 0.3);
		fprintf(gnuplot_pipe, "plot "
				"'./gnuplot_data.txt' using 1:2 with lines title 'cphi',"
				"'./gnuplot_data.txt' using 1:3 with lines title 'dphi'\n");

		fflush(gnuplot_pipe);
		//pclose(gnuplot_pipe);
		//getchar();
		//system("pkill gnuplot");
	}
}


// Core Function of Model Predictive Control
double
carmen_libmpc_get_optimized_steering_effort_using_MPC(double atan_current_curvature, double atan_desired_curvature,
											fann_type *steering_ann_input, struct fann *steering_ann,
											carmen_simulator_ackerman_config_t *simulator_config)
{
	PARAMS p;
	static EFFORT_SPLINE_DESCRIPTOR seed = {0.0, 0.0, 0.0, 0.0};

	if (simulator_config->current_motion_command_vector == NULL)
	{
		seed = {0.0, 0.0, 0.0, 0.0};
		return (0.0);
	}
//	if (simulator_config->current_motion_command_vector_index >= simulator_config->nun_motion_commands) // tem que passar o simulator config e tratar
//		return (0.0);

	p.motion_commands_vector = simulator_config->current_motion_command_vector;
	p.motion_commands_vector_size = simulator_config->nun_motion_commands;
	p.atan_current_curvature = atan_current_curvature;
	p.atan_desired_curvature = atan_desired_curvature;
	p.steering_ann = steering_ann;
	memcpy(p.steering_ann_input, steering_ann_input, NUM_STEERING_ANN_INPUTS * sizeof(fann_type));
	p.v = simulator_config->v;
	p.understeer_coeficient = simulator_config->understeer_coeficient;
	p.distance_rear_axles = simulator_config->distance_between_front_and_rear_axles;

	// (i) Usar o effort_spline_descriptor_seed para criar uma seed do vetor de esforcos.
	// (ii) Usar o simulador para, com o vetor de esforcos, gerar um vetor de motion_commands
	// O passo (ii) ocorrer como parte do conjugate gradient de forma continua, ate chagar a um
	// vetor de motion_commands otimo.
	// Retornar o primeiro (proximo) effort associado a este vetor de motion_commands otimo.

	seed = get_optimized_effort(&p, seed);
	plot_state(&seed, &p, simulator_config);
	double effort = seed.k1;

	return (carmen_clamp(-100.0, effort, 100.0));
}

/*
double
model_predictive_control(double atan_desired_curvature, double atan_current_curvature, carmen_ackerman_motion_command_p current_motion_command_vector,
		double nun_motion_commands,	double time_of_last_command, double past_steering)
{
	static double simulated_atan_current_curvature = 0.0;
	double u_t=0;

	//update_past_steering(atan_current_curvature);
	//update_past_simulated_steering(simulated_atan_current_curvature);

	//u_t = generate_optimized_steering_using_mpc(atan_desired_curvature, atan_current_curvature, current_motion_command_vector,
	//		nun_motion_commands, time_of_last_command, past_steering);

	//update_simulator_bias(past_steering, past_simulated_steering);

	return (u_t);
}


double
get_optmized_steering_effort_using_MPC(double atan_desired_curvature, double atan_current_curvature, carmen_ackerman_motion_command_p current_motion_command_vector,
		double nun_motion_commands,	double time_of_last_command, double past_steering)
{
	double u_t;

	u_t = model_predictive_control(atan_desired_curvature, atan_current_curvature, current_motion_command_vector,
			nun_motion_commands, time_of_last_command, past_steering);

	return (carmen_clamp(-100.0, u_t, 100.0));
}
*/
