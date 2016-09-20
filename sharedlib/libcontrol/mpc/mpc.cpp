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

#define DELTA_T (1.0 / 40.0)

using namespace std;

list<double> past_motion_command_list;
vector<double> new_command_vector;
double Cxk=0.0;


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
	double total_time = delta_t * (NUM_STEERING_ANN_INPUTS / 3); // Cada steering input da rede neural tem dois valores (ver rede neural)
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
get_effort_vector_from_spline_descriptors_new(EFFORT_SPLINE_DESCRIPTOR *descriptors, carmen_ackerman_motion_command_p current_motion_command_vector, int nun_motion_commands)
{
	double time = 0.0;
	double horizon = 10;//Number of commands of the plan to be considered in optimization
	double total_time = compute_total_trajectory_time(current_motion_command_vector, nun_motion_commands);
	double x[4] = { 0.0, total_time / 3.0, 2.0 * total_time / 3.0, total_time };
	double y[4] = { descriptors->k1, descriptors->k2, descriptors->k3, descriptors->k4 };

	gsl_interp_accel *acc = gsl_interp_accel_alloc();
	const gsl_interp_type *type = gsl_interp_cspline;
	gsl_spline *phi_effort_spline = gsl_spline_alloc(type, 4);

	gsl_spline_init(phi_effort_spline, x, y, 4);

	if(nun_motion_commands < horizon)
		horizon = nun_motion_commands;

	vector<double> effort_vector;
	for (int i = 0; i < horizon; i++)
	{
		effort_vector.push_back(gsl_spline_eval(phi_effort_spline, time, acc));
		time += current_motion_command_vector[i].time;
		if(time > total_time)
			break;
		//printf("eff %f\n", effort_vector[i]);
	}

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
	//vector<double> effort_vector = get_effort_vector_from_spline_descriptors(descriptors, p->motion_commands_vector, p->motion_commands_vector_size);
	vector<double> phi_vector;
	double current_atan_of_curvature = p->atan_current_curvature;

	for (unsigned int i = 0; i < effort_vector.size(); i++)
	{
		double effort = carmen_clamp(-100.0, effort_vector[i], 100.0); 				//TODO usar funcao do neuralmodel q faz isso tudo:   carmen_libcarneuralmodel_compute_new_phi_from_effort
		carmen_libcarneuralmodel_build_steering_ann_input(steering_ann_input, effort, current_atan_of_curvature);
		fann_type *steering_ann_output = fann_run(p->steering_ann, steering_ann_input);
		current_atan_of_curvature = steering_ann_output[0];

		double phi = carmen_get_phi_from_curvature(tan(current_atan_of_curvature), p->v, p->understeer_coeficient, p->distance_rear_axles);
		//phi = phi + p->dk;

		phi_vector.push_back(phi);
	}
	return (phi_vector);
}


double
my_f(const gsl_vector *v, void *params)
{
	EFFORT_SPLINE_DESCRIPTOR d;
	PARAMS *p = (PARAMS *) params;

	double delta_t = DELTA_T;
	double motion_commands_vector_time = p->motion_commands_vector[0].time;
	double phi_vector_time = 0.0;
	double error = 0.0;
	double error_sum = 0.0;

	d.k1 = gsl_vector_get(v, 0);
	d.k2 = gsl_vector_get(v, 1);
	d.k3 = gsl_vector_get(v, 2);
	d.k4 = gsl_vector_get(v, 3);

	vector<double> phi_vector = get_phi_vector_from_spline_descriptors(&d, p);

	for (unsigned int i = 0, j = 0; i < phi_vector.size(); i++)
	{
		error = phi_vector[i] - p->motion_commands_vector[j].phi;
		error_sum += sqrt(error * error);
		//printf("Err %f\n", error);

		phi_vector_time += delta_t;
		if (phi_vector_time > motion_commands_vector_time)
		{
			j++;
			if (j >= p->motion_commands_vector_size)
				break;
			motion_commands_vector_time += p->motion_commands_vector[j].time;
		}
	}
	//printf("sum %f\n", error_sum);

	return (error_sum);
}


double
my_f_new(const gsl_vector *v, void *params)
{
	EFFORT_SPLINE_DESCRIPTOR d;
	PARAMS *p = (PARAMS *) params;
	double error = 0.0;
	double error_sum = 0.0;

	d.k1 = gsl_vector_get(v, 0);
	d.k2 = gsl_vector_get(v, 1);
	d.k3 = gsl_vector_get(v, 2);
	d.k4 = gsl_vector_get(v, 3);

	vector<double> phi_vector = get_phi_vector_from_spline_descriptors(&d, p);

	for (unsigned int i = 0; i < phi_vector.size(); i++)
	{
		error = phi_vector[i] - p->motion_commands_vector[i].phi;
		error_sum += sqrt(error * error);
		printf("Err %f\n", error);
	}
	//printf("sum %f\n", error_sum);

	return (error_sum);
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

	//printf("%f %f %f %f\n", df_k1_h, df_k2_h, df_k3_h, df_k4_h);

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
	const gsl_multimin_fdfminimizer_type *T;
	gsl_multimin_fdfminimizer *s;
	gsl_vector *x;
	gsl_multimin_function_fdf my_func;
	size_t iter = 0;
	int status;

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

	} while ((status == GSL_CONTINUE) && (iter < 999));

	//if (status == GSL_SUCCESS)
	//	printf ("Minimum found at: ");
	//printf("iter = %ld\n", iter);

	seed.k1 = gsl_vector_get(s->x, 0);  //The seed struct and keep the best paramter set of previous cicle
	seed.k2 = gsl_vector_get(s->x, 1);
	seed.k3 = gsl_vector_get(s->x, 2);
	seed.k4 = gsl_vector_get(s->x, 3);

	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

	return (seed);
}


void
plot_state2(EFFORT_SPLINE_DESCRIPTOR *seed, PARAMS *p, double v, double understeer_coeficient, double distance_between_front_and_rear_axles, double effort)
{
	#define PAST_SIZE 700
	static list<double> cphi;
	static list<double> dphi;
	static list<double> timestamp;
	static list<double> ef;
	static bool first_time = true;
	static double first_timestamp;
	static FILE *gnuplot_pipe;
	list<double>::reverse_iterator itc;
	list<double>::reverse_iterator itd;
	list<double>::reverse_iterator itt;
	list<double>::reverse_iterator ite;

	double t = carmen_get_time();

	if (first_time)
	{
		first_timestamp = t;
		first_time = false;

		gnuplot_pipe = popen("gnuplot", "w"); //-persist to keep last plot after program closes
		fprintf(gnuplot_pipe, "set xrange [0:30]\n");
		fprintf(gnuplot_pipe, "set yrange [-0.12:0.12]\n");
	}

	cphi.push_front(carmen_get_phi_from_curvature(p->atan_current_curvature, v, understeer_coeficient, distance_between_front_and_rear_axles));
	dphi.push_front(carmen_get_phi_from_curvature(p->atan_desired_curvature, v, understeer_coeficient, distance_between_front_and_rear_axles));
	timestamp.push_front(t - first_timestamp);
	ef.push_front(effort);

	while(cphi.size() > PAST_SIZE)
	{
		cphi.pop_back();
		dphi.pop_back();
		timestamp.pop_back();
		ef.pop_back();
	}


	FILE *gnuplot_data_file = fopen("gnuplot_data.txt", "w");

	// Dados passados
	for (itc = cphi.rbegin(), itd = dphi.rbegin(), itt = timestamp.rbegin(), ite = ef.rbegin(); itc != cphi.rend(); itc++, itd++, itt++, ite++)
		fprintf(gnuplot_data_file, "%lf %lf %lf %lf\n", *itt - timestamp.back(), *itc, *itd, *ite/100);

	// Dados futuros
	vector<double> phi_vector = get_phi_vector_from_spline_descriptors(seed, p);
	Cxk = phi_vector[0];

	double delta_t = DELTA_T;
	double motion_commands_vector_time = p->motion_commands_vector[0].time;
	double phi_vector_time = 0.0;
	double begin_predition_time = timestamp.front() - timestamp.back();
	for (unsigned int i = 0, j = 0; i < phi_vector.size(); i++)
	{
		phi_vector_time += delta_t;
		fprintf(gnuplot_data_file, "%lf %lf %lf\n",
				(timestamp.front() - timestamp.back()) + phi_vector_time, phi_vector[i], p->motion_commands_vector[j].phi);

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
			begin_predition_time, -0.3, begin_predition_time, 0.3);

	fprintf(gnuplot_pipe, "plot "
			"'./gnuplot_data.txt' using 1:2 with lines title 'cphi',"
			"'./gnuplot_data.txt' using 1:3 with lines title 'dphi',"
			"'./gnuplot_data.txt' using 1:4 with lines title 'effort'\n");

	fflush(gnuplot_pipe);
}


void
plot_state(EFFORT_SPLINE_DESCRIPTOR *seed, PARAMS *p, double v, double understeer_coeficient, double distance_between_front_and_rear_axles, double effort)
{
//	#define PAST_SIZE 700
	static list<double> cphi;
	static list<double> dphi;
	static list<double> timestamp;
	static list<double> ef;
	static bool first_time = true;
	static double first_timestamp;
	static FILE *gnuplot_pipe;
	list<double>::reverse_iterator itc;
	list<double>::reverse_iterator itd;
	list<double>::reverse_iterator itt;
	list<double>::reverse_iterator ite;

	double t = carmen_get_time();

	if (first_time)
	{
		first_timestamp = t;
		first_time = false;

		gnuplot_pipe = popen("gnuplot", "w");
		fprintf(gnuplot_pipe, "set xrange [0:30]\n");
		fprintf(gnuplot_pipe, "set yrange [-0.3:0.3]\n");
	}

	cphi.push_front(carmen_get_phi_from_curvature(p->atan_current_curvature, v, understeer_coeficient, distance_between_front_and_rear_axles));
	dphi.push_front(carmen_get_phi_from_curvature(p->atan_desired_curvature, v, understeer_coeficient, distance_between_front_and_rear_axles));
	timestamp.push_front(t - first_timestamp);
	ef.push_front(effort);

	while(cphi.size() > PAST_SIZE)
	{
		cphi.pop_back();
		dphi.pop_back();
		timestamp.pop_back();
		ef.pop_back();
	}

	FILE *gnuplot_data_file = fopen("gnuplot_data.txt", "w");

	// Dados passados
	for (itc = cphi.rbegin(), itd = dphi.rbegin(), itt = timestamp.rbegin(), ite = ef.rbegin(); itc != cphi.rend(); itc++, itd++, itt++, ite++)
		fprintf(gnuplot_data_file, "%lf %lf %lf %lf\n", *itt - timestamp.back(), *itc, *itd, *ite/100);

	// Dados futuros
	vector<double> phi_vector = get_phi_vector_from_spline_descriptors(seed, p);
	double begin_prediction_time = timestamp.front() - timestamp.back();
	double time = begin_prediction_time;
	for (unsigned int i = 0; i < phi_vector.size(); i++)
	{
		fprintf(gnuplot_data_file, "%lf %lf %lf\n", time, phi_vector[i], p->motion_commands_vector[i].phi);
		 time += p->motion_commands_vector[i].time;
	}

	fclose(gnuplot_data_file);

	fprintf(gnuplot_pipe, "unset arrow\nset arrow from %lf, %lf to %lf, %lf nohead\n",
			begin_prediction_time, -0.3, begin_prediction_time, 0.3);

	fprintf(gnuplot_pipe, "plot "
			"'./gnuplot_data.txt' using 1:2 with lines title 'cphi',"
			"'./gnuplot_data.txt' using 1:3 with lines title 'dphi',"
			"'./gnuplot_data.txt' using 1:4 with lines title 'effort'\n");

	fflush(gnuplot_pipe);
}


//x = (y*gain)+offset
double
remove_system_offset_and_gain(double desired_phi, double current_phi, double effort)
{
	static vector<double> past_desired;
	static vector<double> past_executed;
	int i, size = past_desired.size();
	double offset_mean=0, gain_mean=0;

	past_executed.push_back(current_phi);

	if (size == 0)
		return effort;

	if(size > 3) //Horizon that will be considered to remove offset
	{
		past_desired.pop_back();
		past_executed.pop_back();
	}

	for(i=0; i < size; i++)
	{
		offset_mean += past_desired[i] - past_executed[i];
		gain_mean += past_desired[i] / past_executed[i];
	}

	offset_mean = offset_mean / size;
	gain_mean = gain_mean / size;

	past_desired.push_back(desired_phi);

	//return ((current_phi / gain_mean) + offset_mean);
	return (effort - offset_mean);
}


// (i) Usar o effort_spline_descriptor_seed para criar uma seed do vetor de esforcos.
// (ii) Usar o simulador para, com o vetor de esforcos, gerar um vetor de motion_commands
// O passo (ii) ocorrer como parte do conjugate gradient de forma continua, ate chagar a um
// vetor de motion_commands otimo.
// Retornar o primeiro (proximo) effort associado a este vetor de motion_commands otimo.
// Core Function of Model Predictive Control
double
carmen_libmpc_get_optimized_steering_effort_using_MPC(double atan_desired_curvature, double atan_current_curvature, fann_type *steering_ann_input,
														struct fann *steering_ann, carmen_ackerman_motion_command_p current_motion_command_vector,
														int nun_motion_commands, double v, double yp,
														double understeer_coeficient, double distance_between_front_and_rear_axles)
{
	PARAMS p;
	static EFFORT_SPLINE_DESCRIPTOR seed = {0.0, 0.0, 0.0, 0.0};
	//static fann_type steering_ann_input[NUM_STEERING_ANN_INPUTS];
	//static struct fann *steering_ann = NULL;

	if (current_motion_command_vector == NULL)
	{
		seed = {0.0, 0.0, 0.0, 0.0};
		return (0.0);
	}
	/*if (steering_ann == NULL)
	{
		steering_ann = fann_create_from_file("steering_ann.net");
		if (steering_ann == NULL)
		{
			printf("Error: Could not create steering_ann\n");
			exit(1);
		}
		carmen_libcarneuralmodel_init_steering_ann_input(steering_ann_input);
	}*/
//	if (simulator_config->current_motion_command_vector_index >= simulator_config->nun_motion_commands) // tem que passar o simulator config e tratar
//		return (0.0);

	p.motion_commands_vector = current_motion_command_vector;
	p.motion_commands_vector_size = nun_motion_commands;
	p.atan_current_curvature = atan_current_curvature;
	p.atan_desired_curvature = atan_desired_curvature;
	p.steering_ann = steering_ann;
	memcpy(p.steering_ann_input, steering_ann_input, NUM_STEERING_ANN_INPUTS * sizeof(fann_type));
	p.v = v;
	p.understeer_coeficient = understeer_coeficient;
	p.distance_rear_axles = distance_between_front_and_rear_axles;

//	fann_type steering_ann_inputt[NUM_STEERING_ANN_INPUTS];
//	carmen_libcarneuralmodel_build_steering_ann_input(steering_ann_input, seed.k1, atan_current_curvature);
//	fann_type *steering_ann_output = fann_run(p.steering_ann, steering_ann_inputt);
//	double current_atan_of_curvature = steering_ann_output[0];
//	double Cxk = carmen_get_phi_from_curvature(tan(current_atan_of_curvature), p.v, p.understeer_coeficient, p.distance_rear_axles);

	p.dk = yp - Cxk;
	//printf("%f\n", p.dk);

	seed = get_optimized_effort(&p, seed);
	double effort = seed.k1;// + remove_system_offset_and_gain(atan_desired_curvature, atan_current_curvature);

	//printf("%f %f\n", effort, remove_system_offset_and_gain(atan_desired_curvature, atan_current_curvature, effort));

	//effort = remove_system_offset_and_gain(atan_desired_curvature, atan_current_curvature, effort);

	plot_state2(&seed, &p, v, understeer_coeficient, distance_between_front_and_rear_axles, effort);

	return (carmen_clamp(-100.0, effort, 100.0));
}


