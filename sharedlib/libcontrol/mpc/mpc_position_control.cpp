#include "mpc.h"


FILE *gnuplot_save;
bool save_plot = false;


using namespace std;


vector<double>
get_effort_vector_from_spline_descriptors(EFFORT_SPLINE_DESCRIPTOR *descriptors)
{
	double delta_t = DELTA_T;
	double total_time = POSITION_PREDICTION_HORIZON;
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


unsigned int
get_motion_timed_index_to_motion_command_vector(PARAMS* p)
{
	double motion_commands_vector_time = p->motion_commands_vector[0].time;
	unsigned int j = 0;
	while ((motion_commands_vector_time	< p->time_elapsed_since_last_motion_command) &&
		   (j < p->motion_commands_vector_size))
	{
		j++;
		motion_commands_vector_time += p->motion_commands_vector[j].time;
	}

	return j;
}


vector<double>
get_velocity_supersampling_motion_commands_vector(PARAMS *param, unsigned int size)
{
	vector<double> velocity_vector;
	double phi_vector_time = 0.0;

	unsigned int timed_index_to_motion_command = get_motion_timed_index_to_motion_command_vector(param);
	double motion_commands_vector_time = param->motion_commands_vector[timed_index_to_motion_command].time;

	for (unsigned int i = 0; i < size; i++)
	{
		velocity_vector.push_back(param->motion_commands_vector[timed_index_to_motion_command].v);

		phi_vector_time += DELTA_T;
		if (phi_vector_time > motion_commands_vector_time)
		{
			timed_index_to_motion_command++;
			if (timed_index_to_motion_command >= param->motion_commands_vector_size)
				break;

			motion_commands_vector_time += param->motion_commands_vector[timed_index_to_motion_command].time;
		}
	}

	return (velocity_vector);
}


double
car_model(double steering_effort, double atan_current_curvature, double v, fann_type *steering_ann_input, PARAMS *param)
{
//	steering_effort = steering_effort * (1.0 / (1.0 + param->v / 7.0));
//	steering_effort *= (1.0 / (1.0 + (param->v * param->v) / 100.0)); // boa
//	steering_effort = carmen_clamp(-100.0, steering_effort, 100.0);
	double phi = carmen_libcarneuralmodel_compute_new_phi_from_effort(steering_effort, atan_current_curvature, steering_ann_input,
			param->steering_ann, v, param->understeer_coeficient, param->distance_rear_axles, 2.0 * param->max_phi);
//	phi = 1.0 * phi;// - 0.01;
//	phi *= (1.0 / (1.0 + v / 10.0));

	return (phi);
}


vector<double>
get_phi_vector_from_spline_descriptors(EFFORT_SPLINE_DESCRIPTOR *descriptors, PARAMS *params)
{
	fann_type steering_ann_input[NUM_STEERING_ANN_INPUTS];
	memcpy(steering_ann_input, params->steering_ann_input, NUM_STEERING_ANN_INPUTS * sizeof(fann_type));

	vector<double> effort_vector = get_effort_vector_from_spline_descriptors(descriptors);
	vector<double> velocity_vector = get_velocity_supersampling_motion_commands_vector(params, effort_vector.size());
	vector<double> phi_vector;
	double atan_current_curvature = params->atan_current_curvature;

	for (unsigned int i = 0; i < effort_vector.size(); i++)
	{
//		double phi = car_model(effort_vector[i], atan_current_curvature, params->v, steering_ann_input, params);
		double phi = car_model(effort_vector[i], atan_current_curvature, velocity_vector[i], steering_ann_input, params);
		phi = phi + params->dk;

//		phi_vector.push_back(phi);
		phi_vector.push_back(phi + params->dk);

		atan_current_curvature = carmen_get_curvature_from_phi(phi, params->current_velocity, params->understeer_coeficient, params->distance_rear_axles);
	}
	return (phi_vector);
}


vector<carmen_ackerman_traj_point_t>
get_pose_vector_from_spline_descriptors(EFFORT_SPLINE_DESCRIPTOR *descriptors, PARAMS *param)
{
	fann_type steering_ann_input[NUM_STEERING_ANN_INPUTS];
	memcpy(steering_ann_input, param->steering_ann_input, NUM_STEERING_ANN_INPUTS * sizeof(fann_type));

	vector<double> effort_vector = get_effort_vector_from_spline_descriptors(descriptors);
	vector<double> velocity_vector = get_velocity_supersampling_motion_commands_vector(param, effort_vector.size());
	//vector<double> phi_vector;
	vector<carmen_ackerman_traj_point_t> pose_vector;

	double atan_current_curvature = param->atan_current_curvature;

	carmen_ackerman_traj_point_t robot_state;
	robot_state.x = param->global_pos.globalpos.x;
	robot_state.y = param->global_pos.globalpos.y;
	robot_state.theta = param->global_pos.globalpos.theta;
	double distance_traveled = 0.0;

	for (unsigned int i = 0; i < effort_vector.size(); i++)
	{
		double phi = car_model(effort_vector[i], atan_current_curvature, velocity_vector[i], steering_ann_input, param);
		//phi_vector.push_back(phi + param->dk);
		atan_current_curvature = carmen_get_curvature_from_phi(phi, param->current_velocity, param->understeer_coeficient, param->distance_rear_axles);

		robot_state.v = velocity_vector[i];
		robot_state.phi = phi;
		carmen_ackerman_traj_point_t pose = carmen_libcarmodel_recalc_pos_ackerman(robot_state, velocity_vector[i], phi, DELTA_T, &distance_traveled, DELTA_T, *param->robot_config);
		pose_vector.push_back(pose);
		robot_state = pose;
	}
	return (pose_vector);
}


double
dist(carmen_ackerman_traj_point_t v, carmen_ackerman_motion_command_t w)
{
    return sqrt((carmen_square(v.x - w.x) + carmen_square(v.y - w.y)));
}


double
distance_point_to_line(carmen_ackerman_traj_point_t point, carmen_ackerman_motion_command_t line_a, carmen_ackerman_motion_command_t line_b)
{
	//printf("%lf %lf %lf %lf %lf %lf\n", point.x, point.y, line_a->x, line_a->y, line_b->x, line_b->y);
	//yA - yB = a, xB - xA = b e xAyB - xByA=c
	double a = line_a.y - line_b.y;
	double b = line_b.x - line_a.x;
	double c = (line_a.x * line_b.y) - (line_b.x * line_a.y);

	if ( a == 0.0 || b == 0.0)
		return (5.0);   //  quando os pontos sao iguais, oq fazer?

	// dist = |ax0 + by0 + c| / √(a^2 + b^2)
	double dist = fabs((a *  point.x) + (b * point.y) + c) / sqrt((a * a) + (b * b));

	return (dist);
}


double
my_f(const gsl_vector *v, void *params)
{
	EFFORT_SPLINE_DESCRIPTOR d;
	PARAMS *p = (PARAMS *) params;

	double pose_vector_time = 0.0;
	double error = 0.0;
	double error_sum = 0.0;

	d.k1 = gsl_vector_get(v, 0);
	d.k2 = gsl_vector_get(v, 1);
	d.k3 = gsl_vector_get(v, 2);
	d.k4 = gsl_vector_get(v, 3);

	vector<carmen_ackerman_traj_point_t> pose_vector = get_pose_vector_from_spline_descriptors(&d, p);

	// TODO: por que nao da para substituir isso pela funcao que (aparentemente) faz a mesma coisa?
	double motion_commands_vector_time = p->motion_commands_vector[0].time;
	unsigned int j = 0;
	while ((motion_commands_vector_time < p->time_elapsed_since_last_motion_command) &&
		   (j < p->motion_commands_vector_size))
	{
		j++;
		motion_commands_vector_time += p->motion_commands_vector[j].time;
	}
	//printf("k1 %lf k2 %lf k3 %lf k4\n", d.k1, d.k2, d.k3, d.k4);

//	printf("x %lf  y %lf t %lf,  xp %lf  yp %lf t %lf\n", p->motion_commands_vector[j].x, p->motion_commands_vector[j].y, p->motion_commands_vector[j].theta, pose_vector[0].x, pose_vector[0].y, pose_vector[0].theta);

	for (unsigned int i = 0; i < pose_vector.size(); i++)
	{
//		printf("x %lf  y %lf xp %lf  yp %lf\n", p->motion_commands_vector[j].x, p->motion_commands_vector[j].y, pose_vector[i].x, pose_vector[i].y);

//		error = dist(pose_vector[i], p->motion_commands_vector[j]);
//		printf("dist %lf\n", error);
		error = distance_point_to_line(pose_vector[i], p->motion_commands_vector[j], p->motion_commands_vector[j+1]);

		error_sum += fabs(error);

		pose_vector_time += DELTA_T;
		if (pose_vector_time > motion_commands_vector_time)
		{
			//printf("Time %lf\n", p->motion_commands_vector[j].time);
			j++;
			if (j >= p->motion_commands_vector_size)
				break;
			motion_commands_vector_time += p->motion_commands_vector[j].time;
		}
	}

	double cost = error_sum;// + 0.00011 * sqrt((p->previous_k1 - d.k1) * (p->previous_k1 - d.k1));
	//printf("erro %lf\n\n", cost);

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

	} while ((status == GSL_CONTINUE) && (iter < 30));

	printf("iter = %ld status %d\n", iter, status);

	seed.k1 = carmen_clamp(-100.0, gsl_vector_get(s->x, 0), 100.0);
	seed.k2 = carmen_clamp(-100.0, gsl_vector_get(s->x, 1), 100.0);
	seed.k3 = carmen_clamp(-100.0, gsl_vector_get(s->x, 2), 100.0);
	seed.k4 = carmen_clamp(-100.0, gsl_vector_get(s->x, 3), 100.0);

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

	unsigned int timed_index_to_motion_command = get_motion_timed_index_to_motion_command_vector(p);
	dphi_vector.push_front(p->motion_commands_vector[timed_index_to_motion_command].phi);
	cphi_vector.push_front(carmen_get_phi_from_curvature(p->atan_current_curvature, v, understeer_coeficient, distance_between_front_and_rear_axles));
	timestamp_vector.push_front(t - first_timestamp);
	effort_vector.push_front(effort);

	if (save_plot)
		fprintf(gnuplot_save, "%lf %lf %lf %lf\n", timestamp_vector.front(), cphi_vector.front(), dphi_vector.front(), effort_vector.front()/100);

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
	vector<carmen_ackerman_traj_point_t> pose_vector;
	vector<double> phi_vector = get_phi_vector_from_spline_descriptors(seed, p);
	vector<double> future_effort_vector = get_effort_vector_from_spline_descriptors(seed);

	double delta_t = DELTA_T;
	double phi_vector_time = 0.0;
	double begin_predition_time = timestamp_vector.front() - timestamp_vector.back();

	double motion_commands_vector_time = p->motion_commands_vector[timed_index_to_motion_command].time;
	for (unsigned int i = 0; i < phi_vector.size(); i++)
	{
		phi_vector_time += delta_t;
		fprintf(gnuplot_data_file, "%lf %lf %lf %lf %d %d\n",
				begin_predition_time + phi_vector_time, phi_vector[i], p->motion_commands_vector[timed_index_to_motion_command].phi,
				future_effort_vector[i], 1, 2);

		if (phi_vector_time > motion_commands_vector_time)
		{
			timed_index_to_motion_command++;
			if (timed_index_to_motion_command >= p->motion_commands_vector_size)
				break;
			motion_commands_vector_time += p->motion_commands_vector[timed_index_to_motion_command].time;
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
open_file_to_save_plot()
{
	time_t rawtime;
	struct tm * timeinfo;

	time ( &rawtime );
	timeinfo = localtime ( &rawtime );

	char name[32];
	char aux[8];
	name[0] = '\0';
	aux[0] = '\0';

	strcat (name, "mpc_plot_");

	sprintf(aux, "%d", timeinfo->tm_year + 1900);
	strcat (name, aux);

	sprintf(aux, "%d", timeinfo->tm_mon + 1);
	strcat (name, aux);

	sprintf(aux, "%d", timeinfo->tm_mday);
	strcat (name, aux);
	strcat (name,"_");

	sprintf(aux, "%d", timeinfo->tm_hour);
	strcat (name, aux);
	strcat (name, "h");

	sprintf(aux, "%d", timeinfo->tm_min);
	strcat (name, aux);
	strcat (name, "m");

	sprintf(aux, "%d", timeinfo->tm_sec);
	strcat (name, aux);

	gnuplot_save = fopen(name, "w");

//	plot "mpc_plot_20161020_22h57m36" using 1:2 with lines, "mpc_plot_20161020_22h57m36" using 1:3 with lines, "mpc_plot_20161020_22h57m36" using 1:4 with lines
}


bool
init_mpc(bool &first_time, PARAMS &param, EFFORT_SPLINE_DESCRIPTOR &seed, double atan_current_curvature,
		carmen_ackerman_motion_command_p current_motion_command_vector,	int nun_motion_commands,
		double v, double time_of_last_motion_command,
		carmen_robot_ackerman_config_t *robot_config,
		carmen_localize_ackerman_globalpos_message global_pos, int initialize_neural_networks)
{
	static bool open = true;

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
		param.robot_config = robot_config;

		first_time = false;

		seed = {0.0, 0.0, 0.0, 0.0};
	}

	if (current_motion_command_vector == NULL || nun_motion_commands < 2 )
	{
		seed = {0.0, 0.0, 0.0, 0.0};

		return (false);
	}

	if (initialize_neural_networks)
	{
		carmen_libcarneuralmodel_init_steering_ann_input(param.steering_ann_input);
		seed = {0.0, 0.0, 0.0, 0.0};
	}

	param.motion_commands_vector = current_motion_command_vector;
	param.motion_commands_vector_size = nun_motion_commands;
	param.atan_current_curvature = atan_current_curvature;
	param.current_velocity = v;
	param.understeer_coeficient = robot_config->understeer_coeficient;
	param.distance_rear_axles = robot_config->distance_between_front_and_rear_axles;
	param.max_phi = robot_config->max_phi;
	param.time_elapsed_since_last_motion_command = carmen_get_time() - time_of_last_motion_command;
	param.global_pos = global_pos;

	if (save_plot && open)
	{
		open_file_to_save_plot();
		open = false;
	}

	return (true);
}


void
publish_mpc_plan_message(vector<carmen_ackerman_traj_point_t> pose_vector)
{
	int i;
	carmen_navigator_ackerman_plan_message predicted_trajectory_message;

	predicted_trajectory_message.path_length = pose_vector.size();
	predicted_trajectory_message.path = (carmen_ackerman_traj_point_t *) malloc(sizeof(carmen_ackerman_traj_point_t) * predicted_trajectory_message.path_length);

	for (i = 0; i < predicted_trajectory_message.path_length; i++)
		predicted_trajectory_message.path[i] = pose_vector[i];

	predicted_trajectory_message.timestamp = carmen_get_time();
	predicted_trajectory_message.host = carmen_get_host();

	carmen_obstacle_avoider_publish_motion_planner_path(predicted_trajectory_message);

	free(predicted_trajectory_message.path);
}


double
carmen_libmpc_get_optimized_steering_effort_using_MPC_position_control(double atan_current_curvature,
		carmen_ackerman_motion_command_p current_motion_command_vector,	int nun_motion_commands,
		double v, double yp, double time_of_last_motion_command,
		carmen_robot_ackerman_config_t *robot_config,
		carmen_localize_ackerman_globalpos_message global_pos, int initialize_neural_networks)
{
	static PARAMS param;
	static EFFORT_SPLINE_DESCRIPTOR seed = {0.0, 0.0, 0.0, 0.0};
	static bool first_time = true;

	if (!init_mpc(first_time, param, seed, atan_current_curvature,
					current_motion_command_vector,	nun_motion_commands, v, time_of_last_motion_command,
					robot_config,
					global_pos, initialize_neural_networks))
		return (0.0);

	seed = get_optimized_effort(&param, seed);
	double effort = seed.k1;

	// Calcula o dk do proximo ciclo
	double Cxk = car_model(effort, atan_current_curvature, param.current_velocity, param.steering_ann_input, &param);
	param.dk = yp - Cxk;
	param.dk = 0.0;
	param.previous_k1 = effort;

	plot_state(&seed, &param, v, robot_config->understeer_coeficient, robot_config->distance_between_front_and_rear_axles, effort);

	vector<carmen_ackerman_traj_point_t> pose_vector;
	//get_phi_vector_from_spline_descriptors(pose_vector, &seed, &param);
	//publish_mpc_plan_message(pose_vector);

	carmen_clamp(-100.0, effort, 100.0);
	//printf("St %lf\n", effort);
	return (effort);
}
