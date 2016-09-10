#include <carmen/carmen.h>
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
get_effort_vector_from_spline_descriptors(EFFORT_SPLINE_DESCRIPTOR descriptors, PARAMS p)
{
	double time = (compute_total_trajectory_time(p.motion_commands_vector, p.size) / 4.0);
	double x[4] = { 0.0, time, time * 2.0, time * 3.0 };
	double y[4] = { descriptors.k1, descriptors.k2, descriptors.k3, descriptors.k4 };

	gsl_interp_accel *acc = gsl_interp_accel_alloc();
	const gsl_interp_type *type = gsl_interp_cspline;
	gsl_spline *phi_effort_spline = gsl_spline_alloc(type, 4);

	gsl_spline_init(phi_effort_spline, x, y, 4);

	vector<double> effort_vector;
	double t = 0.0;
	double delta_t = time / p.size;
	for (int i = 0; i < p.size; i++)
	{
		effort_vector.push_back(gsl_spline_eval(phi_effort_spline, t, acc));
		t += delta_t;
	}

	gsl_spline_free(phi_effort_spline);
	gsl_interp_accel_free(acc);

	return (effort_vector);
}


vector<double>
get_phi_vector_from_spline_descriptors(EFFORT_SPLINE_DESCRIPTOR descriptors, PARAMS p)
{
	double phi;
	vector<double> command_vector;
	vector<double> effort_vector;
	fann_type *steering_ann_output;
	fann_type steering_ann_input[NUM_STEERING_ANN_INPUTS];

	memcpy(steering_ann_input, p.steering_ann_input, NUM_STEERING_ANN_INPUTS * sizeof(fann_type));

	effort_vector = get_effort_vector_from_spline_descriptors(descriptors, p);

	for (int i = 0; i < p.size; i++)
	{
		carmen_libcarneuralmodel_build_steering_ann_input(steering_ann_input, effort_vector[i], p.atan_current_curvature);

		steering_ann_output = fann_run(p.steering_ann, steering_ann_input);

		phi = carmen_get_phi_from_curvature(tan(steering_ann_output[0]), p.v, p.understeer_coeficient, p.distance_rear_axles);

		command_vector.push_back(phi);
	}

	return (command_vector);
}


double
my_f(const gsl_vector *v, void *params)
{
	EFFORT_SPLINE_DESCRIPTOR d;
	PARAMS *p = (PARAMS *)params;
	vector<double> new_command_vector;
	double error = 0;

	d.k1 = gsl_vector_get(v, 0);
	d.k2 = gsl_vector_get(v, 1);
	d.k3 = gsl_vector_get(v, 2);
	d.k4 = gsl_vector_get(v, 3);

	new_command_vector = get_phi_vector_from_spline_descriptors(d, *p);
//	printf("FOIIIIIUUU\n\n");

	for (int i = 0; i < p->size; i++)
	{
		error += sqrt((new_command_vector[i] - p->motion_commands_vector[i].phi) * 
					  (new_command_vector[i] - p->motion_commands_vector[i].phi));
	}

	return (error);
}


void
my_df (const gsl_vector *v, void *params, gsl_vector *df)
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
my_fdf (const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
	*f = my_f (x, params);
	my_df (x, params, df);
}


double
get_optmized_effort (PARAMS par)
{
	static EFFORT_SPLINE_DESCRIPTOR seed = {0.0, 0.0, 0.0, 0.0};
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
	my_func.params = &par;

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
	} while ((status == GSL_CONTINUE) && (iter < 30));

	printf("iter = %ld\n", iter);

	// A seed inicia em zero e armazena o melhor conjunto de parametros do ciclo de otimizacao anterior
	seed.k1 = gsl_vector_get(s->x, 0);
	seed.k2 = gsl_vector_get(s->x, 1);
	seed.k3 = gsl_vector_get(s->x, 2);
	seed.k4 = gsl_vector_get(s->x, 3);

	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

	return (seed.k1);
}


// Core Function of Model Predictive Control
double
carmen_libmpc_get_optimized_steering_effort_using_MPC(carmen_ackerman_motion_command_p current_motion_command_vector,
											int nun_motion_commands, double atan_current_curvature, struct fann *steering_ann, fann_type *steering_ann_input,
											double v, double understeer_coeficient, double distance_between_front_and_rear_axles)
{
	PARAMS p;

//	if (simulator_config->current_motion_command_vector_index >= simulator_config->nun_motion_commands) // tem que passar o simulator config e tratar
//		return (0.0);

	p.motion_commands_vector = current_motion_command_vector;
	p.size = nun_motion_commands;
	p.atan_current_curvature = atan_current_curvature;
	p.steering_ann = steering_ann;
	memcpy(p.steering_ann_input, steering_ann_input, NUM_STEERING_ANN_INPUTS * sizeof(fann_type));
	p.v = v;
	p.understeer_coeficient = understeer_coeficient;
	p.distance_rear_axles = distance_between_front_and_rear_axles;

	// (i) Usar o effort_spline_descriptor_seed para criar uma seed do vetor de esforcos.
	// (ii) Usar o simulador para, com o vetor de esforcos, gerar um vetor de motion_commands
	// O passo (ii) ocorrer como parte do conjugate gradient de forma continua, ate chagar a um
	// vetor de motion_commands otimo.
	// Retornar o primeiro (proximo) effort associado a este vetor de motion_commands otimo.

	return (get_optmized_effort(p));
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





