#include <carmen/carmen.h>
#include <list>
#include <vector>
#include <gsl/gsl_math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_multimin.h>
#include <car_model.h>
#include "mpc.h"


#define DELTA_T (1.0 / 40.0)
#define PREDICTION_HORIZON	(1.3*0.6)

using namespace std;


vector<double>
get_effort_vector_from_spline_descriptors(EFFORT_SPLINE_DESCRIPTOR *descriptors)
{
	double delta_t = DELTA_T;
	double total_time = PREDICTION_HORIZON;
	double x[4] = { 0.0, total_time / 3.0, 2.0 * total_time / 3.0, total_time };
	double y[4] = { descriptors->k1, descriptors->k2, descriptors->k3, descriptors->k4 };

	gsl_interp_accel *acc = gsl_interp_accel_alloc();
	const gsl_interp_type *type = gsl_interp_cspline;
	gsl_spline *phi_effort_spline = gsl_spline_alloc(type, 4);

	gsl_spline_init(phi_effort_spline, x, y, 4);

	vector<double> effort_vector;
	for (double t = 0.0; t < total_time; t += delta_t)
		effort_vector.push_back(carmen_clamp(-100.0, gsl_spline_eval(phi_effort_spline, t, acc), 100.0));

	gsl_spline_free(phi_effort_spline);
	gsl_interp_accel_free(acc);

	return (effort_vector);
}


double
car_model(double steering_effort, double atan_current_curvature, fann_type *steering_ann_input, PARAMS *param)
{
	double phi = carmen_libcarneuralmodel_compute_new_phi_from_effort(steering_effort, atan_current_curvature, steering_ann_input,
			param->steering_ann, param->v, param->understeer_coeficient, param->distance_rear_axles, 2.0 * param->max_phi);
	phi = 1.0 * phi;// - 0.01;
	
	return (phi);
}


vector<double>
get_phi_vector_from_spline_descriptors(EFFORT_SPLINE_DESCRIPTOR *descriptors, PARAMS *param)
{
	fann_type steering_ann_input[NUM_STEERING_ANN_INPUTS];
	memcpy(steering_ann_input, param->steering_ann_input, NUM_STEERING_ANN_INPUTS * sizeof(fann_type));

	vector<double> effort_vector = get_effort_vector_from_spline_descriptors(descriptors);
	vector<double> phi_vector;
	double atan_current_curvature = param->atan_current_curvature;

	for (unsigned int i = 0; i < effort_vector.size(); i++)
	{
		double phi = car_model(effort_vector[i], atan_current_curvature, steering_ann_input, param);
		phi = phi + param->dk;

		phi_vector.push_back(phi);

		atan_current_curvature = carmen_get_curvature_from_phi(phi, param->v, param->understeer_coeficient, param->distance_rear_axles);
	}
	return (phi_vector);
}


double
my_f(const gsl_vector *v, void *params)
{
	EFFORT_SPLINE_DESCRIPTOR d;
	PARAMS *p = (PARAMS *) params;

	double delta_t = DELTA_T;
	double phi_vector_time = 0.0;
	double error = 0.0;
	double error_sum = 0.0;

	d.k1 = gsl_vector_get(v, 0);
	d.k2 = gsl_vector_get(v, 1);
	d.k3 = gsl_vector_get(v, 2);
	d.k4 = gsl_vector_get(v, 3);

	vector<double> phi_vector = get_phi_vector_from_spline_descriptors(&d, p);

	double motion_commands_vector_time = p->motion_commands_vector[0].time;
	unsigned int j = 0;
	while ((motion_commands_vector_time < p->time_elapsed_since_last_motion_command) && (j < p->motion_commands_vector_size))
	{
		j++;
		motion_commands_vector_time += p->motion_commands_vector[j].time;
	}

	for (unsigned int i = 0; i < phi_vector.size(); i++)
	{
		error = phi_vector[i] - p->motion_commands_vector[j].phi;
		error_sum += sqrt(error * error);

		phi_vector_time += delta_t;
		if (phi_vector_time > motion_commands_vector_time)
		{
			j++;
			if (j >= p->motion_commands_vector_size)
				break;
			motion_commands_vector_time += p->motion_commands_vector[j].time;
		}
	}

	double cost = error_sum + 0.001 * sqrt((p->previous_k1 - d.k1) * (p->previous_k1 - d.k1));
	//printf("%lf  %lf  %lf  %lf\n", cost, p->previous_k1, d.k1, p->previous_k1 - d.k1);

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
get_optimized_effort(PARAMS *par, EFFORT_SPLINE_DESCRIPTOR seed)
{
	gsl_vector *x;
	gsl_multimin_function_fdf my_func;
	size_t iter = 0;
	int status;

	my_func.n = 4;
	my_func.f = my_f;
	my_func.df = my_df;
	my_func.fdf = my_fdf;
	my_func.params = par;

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

	printf("iter = %ld\n", iter);

	seed.k1 = gsl_vector_get(s->x, 0); // The seed struct and keep the best paramter set of previous cicle
	seed.k2 = gsl_vector_get(s->x, 1);
	seed.k3 = gsl_vector_get(s->x, 2);
	seed.k4 = gsl_vector_get(s->x, 3);

	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

	return (seed);
}


void
plot_state(EFFORT_SPLINE_DESCRIPTOR *seed, PARAMS *p, double v, double understeer_coeficient, double distance_between_front_and_rear_axles, double effort)
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

	cphi_vector.push_front(carmen_get_phi_from_curvature(p->atan_current_curvature, v, understeer_coeficient, distance_between_front_and_rear_axles));
	dphi_vector.push_front(carmen_get_phi_from_curvature(p->atan_desired_curvature, v, understeer_coeficient, distance_between_front_and_rear_axles));
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
		fprintf(gnuplot_data_file, "%lf %lf %lf %lf %d %d\n", *it_timestamp - timestamp_vector.back(), *it_cphi, *it_dphi, *it_effort, 1, 2);

	// Dados futuros
	vector<double> phi_vector = get_phi_vector_from_spline_descriptors(seed, p);
	vector<double> future_effort_vector = get_effort_vector_from_spline_descriptors(seed);

	double delta_t = DELTA_T;
	double phi_vector_time = 0.0;
	double begin_predition_time = timestamp_vector.front() - timestamp_vector.back();

	double motion_commands_vector_time = p->motion_commands_vector[0].time;
	unsigned int j = 0;
	while ((motion_commands_vector_time < p->time_elapsed_since_last_motion_command) && (j < p->motion_commands_vector_size))
	{
		j++;
		motion_commands_vector_time += p->motion_commands_vector[j].time;
	}

	for (unsigned int i = 0; i < phi_vector.size(); i++)
	{
		phi_vector_time += delta_t;
		fprintf(gnuplot_data_file, "%lf %lf %lf %lf %d %d\n",
				begin_predition_time + phi_vector_time, phi_vector[i], p->motion_commands_vector[j].phi,
				future_effort_vector[i], 4, 5);

		if (phi_vector_time > motion_commands_vector_time)
		{
			j++;
			if (j >= p->motion_commands_vector_size)
				break;
			motion_commands_vector_time += p->motion_commands_vector[j].time;
		}
	}
	fclose(gnuplot_data_file);

	fprintf(gnuplot_pipe, "unset arrow\nset arrow from %lf, %lf to %lf, %lf nohead\n",
			begin_predition_time, -60.0, begin_predition_time, 60.0);

	fprintf(gnuplot_pipe, "plot "
			"'./gnuplot_data.txt' using 1:2:5 with lines linecolor variable title 'cphi' axes x1y2,"
			"'./gnuplot_data.txt' using 1:3:6 with lines linecolor variable title 'dphi' axes x1y2,"
			"'./gnuplot_data.txt' using 1:4 with lines title 'effort' axes x1y1\n");

	fflush(gnuplot_pipe);
}


void
apply_system_latencies(carmen_ackerman_motion_command_p current_motion_command_vector,	int nun_motion_commands)
{
	int i, j;

	for (i = 0; i < nun_motion_commands; i++)
	{
		j = i;
		for (double lat = 0.0; lat < 0.3; j++)
		{
			if (j >= nun_motion_commands)
				break;
			lat += current_motion_command_vector[j].time;
		}
		if (j >= nun_motion_commands)
			break;
		current_motion_command_vector[i].phi = current_motion_command_vector[j].phi;
	}

	nun_motion_commands = nun_motion_commands -(nun_motion_commands - i);
}


double
carmen_libmpc_get_optimized_steering_effort_using_MPC(double atan_desired_curvature, double atan_current_curvature,
		carmen_ackerman_motion_command_p current_motion_command_vector,	int nun_motion_commands,
		double v, double yp, double time_of_last_motion_command,
		double understeer_coeficient, double distance_between_front_and_rear_axles, double max_phi,
		int initialize_neural_networks)
{
	static PARAMS param;
	static EFFORT_SPLINE_DESCRIPTOR seed = {0.0, 0.0, 0.0, 0.0};
	static bool first_time = true;
//	static int count = 0;
//
//	count++;
//	if ((count % (80)) == 0)
//		atan_current_curvature += 0.1;

	//atan_current_curvature *= 1.1;

	//apply_system_latencies(current_motion_command_vector, nun_motion_commands);

	if (first_time)
	{
		param.steering_ann = fann_create_from_file("steering_ann.net");
		if (param.steering_ann == NULL)
		{
			printf("Error: Could not create steering_ann in carmen_libmpc_get_optimized_steering_effort_using_MPC()\n");
			exit(1);
		}
		carmen_libcarneuralmodel_init_steering_ann_input(param.steering_ann_input);
		param.dk = 0.0;
		param.previous_k1 = 0.0;
		first_time = false;
	}

	if (current_motion_command_vector == NULL)
	{
		seed = {0.0, 0.0, 0.0, 0.0};

		return (0.0);
	}

	if (initialize_neural_networks)
	{
		carmen_libcarneuralmodel_init_steering_ann_input(param.steering_ann_input);
		seed = {0.0, 0.0, 0.0, 0.0};
	}

	param.motion_commands_vector = current_motion_command_vector;
	param.motion_commands_vector_size = nun_motion_commands;
	param.atan_current_curvature = atan_current_curvature;
	param.atan_desired_curvature = atan_desired_curvature;
	param.v = v;
	param.understeer_coeficient = understeer_coeficient;
	param.distance_rear_axles = distance_between_front_and_rear_axles;
	param.max_phi = max_phi;
	param.time_elapsed_since_last_motion_command = carmen_get_time() - time_of_last_motion_command;

	seed = get_optimized_effort(&param, seed);
	seed.k1 = carmen_clamp(-100.0, seed.k1, 100.0);
	seed.k2 = carmen_clamp(-100.0, seed.k2, 100.0);
	seed.k3 = carmen_clamp(-100.0, seed.k3, 100.0);
	seed.k4 = carmen_clamp(-100.0, seed.k4, 100.0);
	double effort = seed.k1;

	// Calcula o dk do proximo ciclo
	double Cxk = car_model(effort, atan_current_curvature, param.steering_ann_input, &param);
	param.dk = yp - Cxk;
	param.previous_k1 = effort;

	plot_state(&seed, &param, v, understeer_coeficient, distance_between_front_and_rear_axles, effort);

	return (effort);
}
