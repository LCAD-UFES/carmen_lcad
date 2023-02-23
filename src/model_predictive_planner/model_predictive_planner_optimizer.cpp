/*
 * model_predictive_planner_optimizer.cpp
 *
 *  Created on: Jun 23, 2016
 *      Author: lcad
 */


#include <stdio.h>
#include <iostream>
#include <math.h>

#include <carmen/collision_detection.h>
#include <carmen/carmen.h>

#include "robot_state.h"
#include "model/global_state.h"
#include "util.h"
#include "model_predictive_planner_optimizer.h"

#define LINEAR_ACCELERATION false

//#define PUBLISH_PLAN_TREE
#ifdef PUBLISH_PLAN_TREE
#include "model/tree.h"
#include "publisher_util.h"

void
copy_path_to_traj(carmen_robot_and_trailers_traj_point_t *traj, vector<carmen_robot_and_trailers_path_point_t> path);
#endif

#define G_STEP_SIZE	0.001
#define F_STEP_SIZE	G_STEP_SIZE

#define G_TOL		0.01
#define F_TOL		G_TOL

#define G_EPSABS	0.016
#define F_EPSABS	G_EPSABS

////////////////////////////////////////////////////////////////////////////////////////
#define USE_STEFFEN_SPLINE

#define NEW_COMPUTE_A_AND_T_FROM_S

//#define NEW_PATH_TO_LANE_DISTANCE

//#define USE_STANLEY_METHOD
//TODO NAO ESQUECER DE DESCOMENTAR OS PARAMETROS W7 e LOOK_AHEAD NO MAIN
////////////////////////////////////////////////////////////////////////////////////////

bool use_obstacles = true;

//extern carmen_mapper_virtual_laser_message virtual_laser_message;

extern int use_unity_simulator;

double desired_v = 0.001;


void
plot_phi_profile(TrajectoryControlParameters tcp)
{
	static bool first_time = true;
	static FILE *gnuplot_pipeMP;

	if (first_time)
	{
		first_time = false;

		gnuplot_pipeMP = popen("gnuplot", "w"); // -persist to keep last plot after program closes
		fprintf(gnuplot_pipeMP, "set xrange [0:7]\n");
		fprintf(gnuplot_pipeMP, "set yrange [-0.75:0.75]\n");
		fprintf(gnuplot_pipeMP, "set xlabel 't'\n");
		fprintf(gnuplot_pipeMP, "set ylabel 'phi'\n");
		fprintf(gnuplot_pipeMP, "set size ratio -1\n");
	}

	if (!tcp.valid)
	{
		fprintf(gnuplot_pipeMP, "plot 0\n");
		fflush(gnuplot_pipeMP);
		return;
	}

	FILE *mpp_file = fopen("mpp.txt", "w");

	gsl_spline *phi_spline = get_phi_spline(tcp);
	gsl_interp_accel *acc = gsl_interp_accel_alloc();
	for (double t = 0.0; t < tcp.tt; t += tcp.tt / 100.0)
		fprintf(mpp_file, "%f %f\n", t, gsl_spline_eval(phi_spline, t, acc));

	gsl_spline_free(phi_spline);
	gsl_interp_accel_free(acc);

	fclose(mpp_file);

	fprintf(gnuplot_pipeMP, "plot "
			"'./mpp.txt' using 1:2 w l title 'phi' lt rgb 'red'");
	for (unsigned int i = 0; i < tcp.k.size(); i++)
		fprintf(gnuplot_pipeMP, ", '-' w p pt 7 ps 2");

	fprintf(gnuplot_pipeMP, "\n");

	// ************** Ver se esta igual a get_phi_spline() **************
	vector<double> knots_x;
	vector<double> knots_y;
	for (unsigned int i = 0; i < tcp.k.size(); i++)
	{
		double x = (tcp.tt / (double) (tcp.k.size() - 1)) * (double) i;
		double y = tcp.k[i];
		knots_x.push_back(x);
		knots_y.push_back(y);
	}
	knots_x[tcp.k.size() - 1] = tcp.tt; // Para evitar que erros de arredondamento na conta de x, acima, atrapalhe a leitura do ultimo ponto no spline
	if (tcp.k.size() == 4)
	{
		knots_x[1] = tcp.tt / 4.0;
		knots_x[2] = tcp.tt / 2.0;
	}
	// **************

	for (unsigned int i = 0; i < tcp.k.size(); i++)
		fprintf(gnuplot_pipeMP, "%lf %lf\ne\n", knots_x[i], knots_y[i]);

	fflush(gnuplot_pipeMP);
}


void
plot_a_profile(TrajectoryControlParameters tcp)
{
	static bool first_time = true;
	static FILE *gnuplot_pipeMP;

	if (first_time)
	{
		first_time = false;

		gnuplot_pipeMP = popen("gnuplot", "w"); // -persist to keep last plot after program closes
		fprintf(gnuplot_pipeMP, "set xrange [0:7]\n");
		fprintf(gnuplot_pipeMP, "set yrange [-2.0:2.0]\n");
		fprintf(gnuplot_pipeMP, "set xlabel 't'\n");
		fprintf(gnuplot_pipeMP, "set ylabel 'a'\n");
		fprintf(gnuplot_pipeMP, "set mytics 2\n");
		fprintf(gnuplot_pipeMP, "set grid\n");
		fprintf(gnuplot_pipeMP, "set size ratio -1\n");
	}

	if (!tcp.valid)
	{
		fprintf(gnuplot_pipeMP, "plot 0\n");
		fflush(gnuplot_pipeMP);
		return;
	}

	FILE *mpp_file = fopen("mpp.txt", "w");

	for (double t = 0.0; t < tcp.tt; t += tcp.tt / 100.0)
	{
		if ((desired_v == 0.0) && LINEAR_ACCELERATION)
			fprintf(mpp_file, "%f %f\n", t, ((tcp.tt - t) / tcp.tt) * tcp.a);
		else
			fprintf(mpp_file, "%f %f\n", t, tcp.a);
	}
	fclose(mpp_file);

	fprintf(gnuplot_pipeMP, "plot "
			"'./mpp.txt' using 1:2 w l title 'a' lt rgb 'red'");
	fprintf(gnuplot_pipeMP, "\n");

	fflush(gnuplot_pipeMP);
}


void
print_tcp(TrajectoryControlParameters tcp, double timestamp)
{
	printf("timestamp %lf, valid %d, tt %3.8lf, a %3.8lf, vf %3.8lf, sf %3.8lf, s %3.8lf\n",
			timestamp, tcp.valid, tcp.tt, tcp.a, tcp.vf, tcp.sf, tcp.s);

	for (unsigned int i = 0; i < tcp.k.size(); i++)
		printf("k%d % 3.8lf, ", (int) i, tcp.k[i]);

	printf(", ts %lf\n", carmen_get_time());
}


bool
has_valid_discretization(TrajectoryDiscreteDimensions tdd)
{
	if ((tdd.dist < N_DIST) &&
			(tdd.theta < N_THETA) &&
			(tdd.d_yaw < N_D_YAW) &&
			(tdd.phi_i < N_I_PHI) &&
			(tdd.v_i < N_I_V))
		return (true);
	else
		return (false);
}


float
geometric_progression(int n, float scale_factor, float ratio, int index_of_element_zero)
{
	if (n >= index_of_element_zero)
		return (scale_factor * pow(ratio, n - index_of_element_zero) - scale_factor);
	else
		return (-(scale_factor * pow(ratio, index_of_element_zero - n) - scale_factor));
}


double
linear_progression(int n, double common, int index_of_element_zero)
{
	if (n == index_of_element_zero)
		return (0.0);
	else if (n > index_of_element_zero)
		return (common * (n - index_of_element_zero));
	else
		return (-common * (index_of_element_zero - n));
}


float
get_initial_velocity_by_index(int index)
{
	return (geometric_progression(index, FIRST_I_V, RATIO_I_V, ZERO_I_V_I));
}


float
get_delta_velocity_by_index(int index)
{
	return (geometric_progression(index, FIRST_D_V, RATIO_D_V, ZERO_D_V_I));
}


float
get_i_phi_by_index(int index)
{
	return (geometric_progression(index, FIRST_I_PHI, RATIO_I_PHI, ZERO_I_PHI_I));
}


float
get_k2_by_index(int index)
{
	return (geometric_progression(index, FIRST_K2, RATIO_K2, ZERO_K2_I));
}


float
get_k3_by_index(int index)
{
	return (geometric_progression(index, FIRST_K3, RATIO_K3, ZERO_K3_I));
}


float
get_distance_by_index(int index)
{
	return (geometric_progression(index, FIRST_DIST, RATIO_DIST, ZERO_DIST_I));
}


float
get_theta_by_index(int index)
{
	return (linear_progression(index, COMMON_THETA, ZERO_THETA_I));
}


float
get_d_yaw_by_index(int index)
{
	return (geometric_progression(index, FIRST_D_YAW, RATIO_D_YAW, ZERO_D_YAW_I));
}


TrajectoryDimensions
convert_to_trajectory_dimensions(TrajectoryDiscreteDimensions tdd,
		TrajectoryControlParameters tcp)
{
	TrajectoryDimensions td;

	td.d_yaw = get_d_yaw_by_index(tdd.d_yaw);
	td.dist = get_distance_by_index(tdd.dist);
	td.phi_i = get_i_phi_by_index(tdd.phi_i);
	td.theta = get_theta_by_index(tdd.theta);
	td.v_i = get_initial_velocity_by_index(tdd.v_i);
	td.control_parameters = tcp;

	return (td);
}


int
binary_search_geometric_progression(double value,
		int num_elements, double first, double ratio, int zero_index)
{
	int imin = 0;
	int imax = num_elements - 1;
	while (imin < imax)
	{
		int imid = imin + (imax - imin) / 2;

		if (geometric_progression(imid, first, ratio, zero_index) < value)
			imin = imid + 1;
		else
			imax = imid;
	}
	return (imin);
}


int
binary_search_linear_progression(double value,
		int num_elements, double common, int zero_index)
{
	int imin = 0;
	int imax = num_elements - 1;
	while (imin < imax)
	{
		int imid = imin + (imax - imin) / 2;

		if (linear_progression(imid, common, zero_index) < value)
			imin = imid + 1;
		else
			imax = imid;
	}
	return (imin);
}


TrajectoryDiscreteDimensions
get_discrete_dimensions(TrajectoryDimensions td)
{
	TrajectoryDiscreteDimensions tdd;

	tdd.dist = binary_search_geometric_progression(td.dist, N_DIST, FIRST_DIST, RATIO_DIST, ZERO_DIST_I);
	tdd.theta = binary_search_linear_progression(td.theta, N_THETA, COMMON_THETA, ZERO_THETA_I);
	tdd.d_yaw = binary_search_geometric_progression(td.d_yaw, N_D_YAW, FIRST_D_YAW, RATIO_D_YAW, ZERO_D_YAW_I);
	tdd.phi_i = binary_search_geometric_progression(td.phi_i, N_I_PHI, FIRST_I_PHI, RATIO_I_PHI, ZERO_I_PHI_I);
	tdd.v_i = binary_search_geometric_progression(td.v_i, N_I_V, FIRST_I_V, RATIO_I_V, ZERO_I_V_I);

	return (tdd);
}


carmen_robot_and_trailers_path_point_t
convert_to_carmen_robot_and_trailer_path_point_t(const carmen_robot_and_trailers_traj_point_t robot_state, const double time)
{
	carmen_robot_and_trailers_path_point_t path_point;

	path_point.x = robot_state.x;
	path_point.y = robot_state.y;
	path_point.theta = robot_state.theta;
	path_point.num_trailers = robot_state.num_trailers;
	for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
		path_point.trailer_theta[z] = robot_state.trailer_theta[z];

	path_point.v = robot_state.v;
	path_point.phi = robot_state.phi;
	path_point.time = time;

	return (path_point);
}


double
compute_path_via_simulation(carmen_robot_and_trailers_traj_point_t &robot_state, Command &command,
		vector<carmen_robot_and_trailers_path_point_t> &path,
		TrajectoryControlParameters tcp,
		gsl_spline *phi_spline, double v0, double *i_trailer_theta, double delta_t)
{
	gsl_interp_accel *acc = gsl_interp_accel_alloc();

	robot_state.x = 0.0;
	robot_state.y = 0.0;
	robot_state.theta = 0.0;
//	robot_state.trailer_theta[0] = i_beta;
	for (size_t j = 0; j < MAX_NUM_TRAILERS; j++)
		robot_state.trailer_theta[j] = i_trailer_theta[j];

	robot_state.v = v0;
	robot_state.phi = tcp.k[0];

	command.v = v0;
	double multiple_delta_t = 3.0 * delta_t;
	int i = 0;
	double t;
	double distance_traveled = 0.0;
	// Cada ponto na trajetoria marca uma posicao do robo e o delta_t para chegar aa proxima
	path.push_back(convert_to_carmen_robot_and_trailer_path_point_t(robot_state, delta_t));
	for (t = delta_t; t < tcp.tt; t += delta_t)
	{
		if ((desired_v == 0.0) && LINEAR_ACCELERATION)
		{
			double a = tcp.a * ((tcp.tt - t) / tcp.tt);
			command.v += a * delta_t;
		}
		else
			command.v = v0 + tcp.a * t;

		if (command.v > GlobalState::param_max_vel)
			command.v = GlobalState::param_max_vel;
		else if (command.v < GlobalState::param_max_vel_reverse)
			command.v = GlobalState::param_max_vel_reverse;

		command.phi = gsl_spline_eval(phi_spline, t, acc);

//		if ((GlobalState::behavior_selector_task == BEHAVIOR_SELECTOR_PARK_SEMI_TRAILER) ||
//			(GlobalState::behavior_selector_task == BEHAVIOR_SELECTOR_PARK_TRUCK_SEMI_TRAILER))
		if ((GlobalState::semi_trailer_config.num_semi_trailers != 0) && (GlobalState::route_planner_state ==  EXECUTING_OFFROAD_PLAN))
			robot_state = carmen_libcarmodel_recalc_pos_ackerman(robot_state, command.v, command.phi, delta_t,
					&distance_traveled, delta_t / 10.0, GlobalState::robot_config, GlobalState::semi_trailer_config);
		else
			robot_state = carmen_libcarmodel_recalc_pos_ackerman(robot_state, command.v, command.phi, delta_t,
					&distance_traveled, delta_t / 10.0, GlobalState::robot_config, GlobalState::semi_trailer_config);

		// Cada ponto na trajetoria marca uma posicao do robo e o delta_t para chegar aa proxima
		if (GlobalState::eliminate_path_follower && (i > 70))
			delta_t = multiple_delta_t;
		path.push_back(convert_to_carmen_robot_and_trailer_path_point_t(robot_state, delta_t));

		i++;
	}

	if ((tcp.tt - (t -  delta_t)) > 0.0)
	{
		double final_delta_t = tcp.tt - (t - delta_t);

		if ((desired_v == 0.0) && LINEAR_ACCELERATION)
		{
			double a = tcp.a * ((tcp.tt - t) / tcp.tt);
			command.v += a * final_delta_t;
		}
		else
			command.v = v0 + tcp.a * tcp.tt;

		if (command.v > GlobalState::param_max_vel)
			command.v = GlobalState::param_max_vel;
		else if (command.v < GlobalState::param_max_vel_reverse)
			command.v = GlobalState::param_max_vel_reverse;
		command.phi = gsl_spline_eval(phi_spline, tcp.tt, acc);

		robot_state = carmen_libcarmodel_recalc_pos_ackerman(robot_state, command.v, command.phi, final_delta_t,
				&distance_traveled, final_delta_t, GlobalState::robot_config, GlobalState::semi_trailer_config);

		// Cada ponto na trajetoria marca uma posicao do robo e o delta_t para chegar aa proxima
		path.push_back(convert_to_carmen_robot_and_trailer_path_point_t(robot_state, 0.0));
	}

	gsl_interp_accel_free(acc);

	return (distance_traveled);
}


void
print_phi_profile(gsl_spline *phi_spline, gsl_interp_accel *acc, double total_t, bool display_phi_profile)
{
	if (!display_phi_profile)
		return;

	FILE *path_file = fopen("gnuplot_path2.txt", "w");

	for (double t = 0.0; t < total_t; t += total_t / 100.0)
		fprintf(path_file, "%f %f\n", t, gsl_spline_eval(phi_spline, t, acc));
	fclose(path_file);

	FILE *gnuplot_pipe = popen("gnuplot -persist", "w");
	fprintf(gnuplot_pipe, "plot './gnuplot_path2.txt' using 1:2 with lines\n");
	fflush(gnuplot_pipe);
	pclose(gnuplot_pipe);
	getchar();
	system("pkill gnuplot");
}


double
get_max_distance_in_path(vector<carmen_robot_and_trailers_path_point_t> path, carmen_robot_and_trailers_path_point_t &furthest_point)
{
	double max_dist = 0.0;

	furthest_point = path[0];
	for (unsigned int i = 0; i < path.size(); i++)
	{
		double distance = DIST2D(path[0], path[i]);
		if (distance > max_dist)
		{
			max_dist = distance;
			furthest_point = path[i];
		}
	}

	return (max_dist);
}


gsl_spline *
get_phi_spline(TrajectoryControlParameters tcp)
{
	// Create phi profile
	vector<double> knots_x;
	vector<double> knots_y;
	for (unsigned int i = 0; i < tcp.k.size(); i++)
	{
		double x = (tcp.tt / (double) (tcp.k.size() - 1)) * (double) (i);
		double y = tcp.k[i];
		knots_x.push_back(x);
		knots_y.push_back(y);
	}
	knots_x[tcp.k.size() - 1] = tcp.tt; // Para evitar que erros de arredondamento na conta de x, acima, atrapalhe a leitura do ultimo ponto no spline
	if (tcp.k.size() == 4)
	{
		knots_x[1] = tcp.tt / 4.0;
		knots_x[2] = tcp.tt / 2.0;
	}

#ifdef USE_STEFFEN_SPLINE
	const gsl_interp_type *type = gsl_interp_cspline;
#else
	const gsl_interp_type *type = gsl_interp_steffen;
#endif
	gsl_spline *phi_spline = gsl_spline_alloc(type, tcp.k.size());
	gsl_spline_init(phi_spline, &knots_x[0], &knots_y[0], tcp.k.size());

	return (phi_spline);
}


vector<carmen_robot_and_trailers_path_point_t>
simulate_car_from_parameters(TrajectoryDimensions &td,
		TrajectoryControlParameters &tcp, double v0, double *i_trailer_theta, double delta_t)
{
	vector<carmen_robot_and_trailers_path_point_t> path = {};
	if (!tcp.valid)
		return (path);

	gsl_spline *phi_spline = get_phi_spline(tcp);

	Command command;
	carmen_robot_and_trailers_traj_point_t robot_state;
	double distance_traveled = compute_path_via_simulation(robot_state, command, path, tcp, phi_spline, v0, i_trailer_theta, delta_t);

	gsl_spline_free(phi_spline);

	carmen_robot_and_trailers_path_point_t furthest_point;
	td.dist = get_max_distance_in_path(path, furthest_point);	// @@@ Alberto: Por que nao o ultimo do ponto do path?
	td.theta = atan2(furthest_point.y, furthest_point.x);
	td.d_yaw = furthest_point.theta;
	td.phi_i = tcp.k[0];
	td.v_i = v0;
	tcp.vf = command.v;
	tcp.sf = distance_traveled;
	td.control_parameters = tcp;

	return (path);
}


void
print_path(vector<carmen_robot_and_trailers_path_point_t> path)
{
	FILE *path_file = fopen("gnuplot_path.txt", "a");
	int i = 0;
	for (std::vector<carmen_robot_and_trailers_path_point_t>::iterator it = path.begin(); it != path.end(); ++it)
	{
		if ((i % 2) == 0)
			fprintf(path_file, "%f %f %f %f %f\n", it->x, it->y, 1.0 * cos(it->theta), 1.0 * sin(it->theta), it->trailer_theta[0]);
		i++;
	}
	fclose(path_file);
}

void
print_lane(vector<carmen_robot_and_trailers_path_point_t> path, FILE *path_file)
{
	//	int i = 0;
	for (std::vector<carmen_robot_and_trailers_path_point_t>::iterator it = path.begin(); it != path.end(); ++it)
	{
		//		if ((i % 2) == 0)
		fprintf(path_file, "%f %f %f %f\n", it->x, it->y, 1.0 * cos(it->theta), 1.0 * sin(it->theta));
		//		i++;
	}
}

void
print_lane(vector<carmen_robot_and_trailers_path_point_t> path, char *file_name)
{
	FILE *path_file = fopen(file_name, "w");
	for (std::vector<carmen_robot_and_trailers_path_point_t>::iterator it = path.begin(); it != path.end(); ++it)
		fprintf(path_file, "%f %f %f %f %f %f %f\n", it->x, it->y, it->theta, it->trailer_theta[0], it->v, it->phi, it->time);

	fclose(path_file);
}


bool
path_has_loop(double dist, double sf)
{
	if (dist < 0.05)
		return (false);

	if (sf > (M_PI * dist * 1.1)) // se sf for maior que meio arco com diametro dist mais um pouco (1.1) tem loop
		return (true);
	return (false);
}


bool
bad_tcp(TrajectoryControlParameters tcp)
{
	if (isnan(tcp.tt) || isnan(tcp.a) || isnan(tcp.s))
		return (true);
	else
		return (false);
}


#define NEW_COMPUTE_A_AND_T_FROM_S
#ifdef NEW_COMPUTE_A_AND_T_FROM_S

void
compute_a_and_t_from_s_reverse(double s, double target_v,
		TrajectoryDimensions target_td,
		TrajectoryControlParameters &tcp_seed,
		ObjectiveFunctionParams *params)
{
	double a;

	if ((desired_v == 0.0) && LINEAR_ACCELERATION)
		a = 1.33333333 * (target_v * target_v - target_td.v_i * target_td.v_i) / (2.0 * s);
	else
	// https://www.wolframalpha.com/input/?i=solve+s%3Dv*x%2B0.5*a*x%5E2
		a = (target_v * target_v - target_td.v_i * target_td.v_i) / (2.0 * s);

	a = (-1.0) * a;
	if (a == 0.0)
	{
		if (target_v != 0.0)
			tcp_seed.tt = fabs(s / target_v);
		else
			tcp_seed.tt = 0.05;
	}
	else if (a < -GlobalState::robot_config.maximum_acceleration_reverse)
	{
		a = GlobalState::robot_config.maximum_acceleration_reverse;
		double v = fabs(target_td.v_i);
		tcp_seed.tt = (sqrt(fabs(2.0 * a * s + v * v) + v)) / a;
		a = (-1.0) * a;

	}
	else if (a > GlobalState::robot_config.maximum_deceleration_reverse)
	{
		a = GlobalState::robot_config.maximum_deceleration_reverse;
		double v = fabs(target_td.v_i);
		tcp_seed.tt = (sqrt(fabs(2.0 * a * s + v * v)) + v) / a;
	}
	else
	{
		if ((desired_v == 0.0) && LINEAR_ACCELERATION)
			tcp_seed.tt = 2.0 * (target_v - target_td.v_i) / a;
		else
			tcp_seed.tt = (target_v - target_td.v_i) / a;
	}
	if (tcp_seed.tt > 200.0)
		tcp_seed.tt = 200.0;
	if (tcp_seed.tt < 0.05)
		tcp_seed.tt = 0.05;

//	if (isnan(tcp_seed.tt) || isnan(a))
//		printf("nan em compute_a_and_t_from_s_reverse()\n");

	params->suitable_tt = tcp_seed.tt;
	params->suitable_acceleration = tcp_seed.a = a;
}


void
compute_a_and_t_from_s_foward(double s, double target_v,
		TrajectoryDimensions target_td,
		TrajectoryControlParameters &tcp_seed,
		ObjectiveFunctionParams *params)
{
	double a;

	if ((desired_v == 0.0) && LINEAR_ACCELERATION)
		a = 1.33333333 * (target_v * target_v - target_td.v_i * target_td.v_i) / (2.0 * s);
	else
	// https://www.wolframalpha.com/input/?i=solve+s%3Dv*x%2B0.5*a*x%5E2
		a = (target_v * target_v - target_td.v_i * target_td.v_i) / (2.0 * s);

	if (a == 0.0)
	{
		if (target_v != 0.0)
			tcp_seed.tt = s / target_v;
		else
			tcp_seed.tt = 0.05;
	}
	else if (a > GlobalState::robot_config.maximum_acceleration_forward)
	{
		a = GlobalState::robot_config.maximum_acceleration_forward;
		double v = target_td.v_i;
		tcp_seed.tt = (sqrt(2.0 * a * s + v * v) - v) / a;
	}
	else if (a < -GlobalState::robot_config.maximum_deceleration_forward)
	{
		a = -GlobalState::robot_config.maximum_deceleration_forward;
		double v = target_td.v_i;
		tcp_seed.tt = (sqrt(2.0 * a * s + v * v) - v) / a;
	}
	else
	{
		if ((desired_v == 0.0) && LINEAR_ACCELERATION)
			tcp_seed.tt = 2.0 * (target_v - target_td.v_i) / a;
		else
			tcp_seed.tt = (target_v - target_td.v_i) / a;
	}

	if (tcp_seed.tt > 200.0)
		tcp_seed.tt = 200.0;
	if (tcp_seed.tt < 0.05)
		tcp_seed.tt = 0.05;

//	if (isnan(tcp_seed.tt) || isnan(a))
//		printf("nan em compute_a_and_t_from_s_foward()\n");

	params->suitable_tt = tcp_seed.tt;
	params->suitable_acceleration = tcp_seed.a = a;
}

#else

void
compute_a_and_t_from_s_reverse(double s, double target_v,
		TrajectoryDimensions target_td,
		TrajectoryControlParameters &tcp_seed,
		ObjectiveFunctionParams *params)
{
	// https://www.wolframalpha.com/input/?i=solve+s%3Dv*x%2B0.5*a*x%5E2
	double a = (target_v * target_v - target_td.v_i * target_td.v_i) / (2.0 * s);
	a = (-1.0) * a;
	if (a == 0.0)
	{
		if (target_v != 0.0)
			tcp_seed.tt = fabs(s / target_v);
		else
			tcp_seed.tt = 0.05;
	}
	else if (a < -GlobalState::robot_config.maximum_acceleration_reverse)
	{
		a = GlobalState::robot_config.maximum_acceleration_reverse;
		double v = fabs(target_td.v_i);
		tcp_seed.tt = (sqrt(fabs(2.0 * a * s + v * v) + v)) / a;
		a = (-1)*a;

	}
	else if (a > GlobalState::robot_config.maximum_deceleration_reverse)
	{
		a = GlobalState::robot_config.maximum_deceleration_reverse;
		double v = fabs(target_td.v_i);
		tcp_seed.tt = (sqrt(fabs(2.0 * a * s + v * v)) + v) / a;
	}
	else
		tcp_seed.tt = (target_v - target_td.v_i) / a;

	if (tcp_seed.tt > 200.0)
		tcp_seed.tt = 200.0;
	if (tcp_seed.tt < 0.05)
		tcp_seed.tt = 0.05;

//	if (isnan(tcp_seed.tt) || isnan(a))
//		printf("nan em compute_a_and_t_from_s_reverse()\n");

	params->suitable_tt = tcp_seed.tt;
	params->suitable_acceleration = tcp_seed.a = a;
}


void
compute_a_and_t_from_s_foward(double s, double target_v,
		TrajectoryDimensions target_td,
		TrajectoryControlParameters &tcp_seed,
		ObjectiveFunctionParams *params)
{
	// https://www.wolframalpha.com/input/?i=solve+s%3Dv*x%2B0.5*a*x%5E2
	double a = (target_v * target_v - target_td.v_i * target_td.v_i) / (2.0 * s);
	if (a == 0.0)
	{
		if (target_v != 0.0)
			tcp_seed.tt = s / target_v;
		else
			tcp_seed.tt = 0.05;
	}
	else if (a > GlobalState::robot_config.maximum_acceleration_forward)
	{
		a = GlobalState::robot_config.maximum_acceleration_forward;
		double v = target_td.v_i;
		tcp_seed.tt = (sqrt(2.0 * a * s + v * v) - v) / a;
	}
	else if (a < -GlobalState::robot_config.maximum_deceleration_forward)
	{
		a = -GlobalState::robot_config.maximum_deceleration_forward;
		double v = target_td.v_i;
		tcp_seed.tt = -(sqrt(2.0 * a * s + v * v) + v) / a;
	}
	else
		tcp_seed.tt = (target_v - target_td.v_i) / a;

	if (tcp_seed.tt > 200.0)
		tcp_seed.tt = 200.0;
	if (tcp_seed.tt < 0.05)
		tcp_seed.tt = 0.05;

//	if (isnan(tcp_seed.tt) || isnan(a))
//		printf("nan em compute_a_and_t_from_s_foward()\n");

	params->suitable_tt = tcp_seed.tt;
	params->suitable_acceleration = tcp_seed.a = a;
}

#endif

void
compute_a_and_t_from_s(double s, double target_v,
		TrajectoryDimensions target_td,
		TrajectoryControlParameters &tcp_seed,
		ObjectiveFunctionParams *params)
{
	if (GlobalState::reverse_planning)
		compute_a_and_t_from_s_reverse(s, target_v, target_td, tcp_seed, params);
	else
		compute_a_and_t_from_s_foward(s, target_v, target_td, tcp_seed, params);
}


void
limit_tcp_phi(TrajectoryControlParameters &tcp)
{
	double max_phi_during_planning = 1.0 * GlobalState::robot_config.max_phi;

	for (unsigned int i = 1; i < tcp.k.size(); i++)
	{
		if (tcp.k[i] > max_phi_during_planning)
			tcp.k[i] = max_phi_during_planning;
		if (tcp.k[i] < -max_phi_during_planning)
			tcp.k[i] = -max_phi_during_planning;
	}
}


TrajectoryControlParameters
fill_in_tcp(const gsl_vector *x, ObjectiveFunctionParams *params)
{
	TrajectoryControlParameters tcp = {};

	tcp.s = gsl_vector_get(x, 0);
	if (tcp.s < 0.01)
		tcp.s = 0.01;

	tcp.k.push_back(params->target_td->phi_i);
	for (unsigned int i = 1; i < x->size; i++)
		tcp.k.push_back(gsl_vector_get(x, i));
//	limit_tcp_phi(tcp);

	compute_a_and_t_from_s(tcp.s, params->target_v, *params->target_td, tcp, params);
	tcp.a = params->suitable_acceleration;
	tcp.tt = params->suitable_tt;

	if (tcp.tt < 0.05) // o tempo nao pode ser pequeno demais
		tcp.tt = 0.05;

//	if (tcp.a < -GlobalState::robot_config.maximum_deceleration_forward) // a aceleracao nao pode ser negativa demais//TODO
//		tcp.a = -GlobalState::robot_config.maximum_deceleration_forward;
//	if (tcp.a > GlobalState::robot_config.maximum_acceleration_forward) // a aceleracao nao pode ser positiva demais
//		tcp.a = GlobalState::robot_config.maximum_acceleration_forward;

	tcp.valid = true;

	return (tcp);
}


void
move_path_to_current_robot_pose(vector<carmen_robot_and_trailers_path_point_t> &path, carmen_robot_and_trailers_pose_t *localizer_pose)
{
	for (std::vector<carmen_robot_and_trailers_path_point_t>::iterator it = path.begin(); it != path.end(); ++it)
	{
		double x = localizer_pose->x + it->x * cos(localizer_pose->theta) - it->y * sin(localizer_pose->theta);
		double y = localizer_pose->y + it->x * sin(localizer_pose->theta) + it->y * cos(localizer_pose->theta);
		it->x = x;
		it->y = y;
		double old_theta = it->theta;
		it->theta = carmen_normalize_theta(it->theta + localizer_pose->theta);
		for (size_t j = 0; j < MAX_NUM_TRAILERS; j++)
		{
			double beta = convert_theta1_to_beta(old_theta, it->trailer_theta[j]);
			it->trailer_theta[j] = convert_beta_to_theta1(it->theta, beta);
		}

	}
}


carmen_robot_and_trailers_path_point_t
move_to_front_axle_WORKING(carmen_robot_and_trailers_path_point_t pose)
{
	double L = GlobalState::robot_config.distance_between_front_and_rear_axles;
	carmen_robot_and_trailers_path_point_t pose_moved = pose;
	pose_moved.x += L * cos(pose.theta);
	pose_moved.y += L * sin(pose.theta);

	return (pose_moved);
}


carmen_robot_and_trailers_path_point_t
move_to_front_axle(carmen_robot_and_trailers_path_point_t pose)
{
	return (pose);

	double L = GlobalState::robot_config.distance_between_front_and_rear_axles;
	carmen_robot_and_trailers_path_point_t pose_moved = pose;
	pose_moved.x += L * cos(pose.theta);
	pose_moved.y += L * sin(pose.theta);

	return (pose_moved);
}

//#define NEW_PATH_TO_LANE_DISTANCE

#ifdef NEW_PATH_TO_LANE_DISTANCE

double
compute_path_to_lane_distance(ObjectiveFunctionParams *my_params, vector<carmen_robot_and_trailers_path_point_t> &path)
{
	double distance = 0.0;
	double total_distance = 0.0;
	double total_points = 0.0;

	int increment;
	if (use_unity_simulator)
		increment = 1;
	else
		increment = 3;

	carmen_robot_and_trailers_path_point_t nearest_point = {};
	unsigned int i;
	int status;
	for (i = 0; i < (my_params->detailed_lane.size() - 1); i += 1)
	{
		nearest_point = carmen_get_point_nearest_to_path(&status, my_params->detailed_lane[i], my_params->detailed_lane[i + 1], path[0], 0.001);
		if (status == POINT_WITHIN_SEGMENT)
			break;
	}
	if (i >= my_params->detailed_lane.size())
		nearest_point = carmen_get_point_nearest_to_path(&status, my_params->detailed_lane[0], my_params->detailed_lane[1], path[0], 0.001);
	double angle = ANGLE2D(path[0], nearest_point);
	bool first_point_side = (carmen_normalize_theta(path[0].theta - angle) > 0.0)? true: false;

	for (unsigned int i = 0, j = 0; i < path.size(); i += increment)
	{
		for ( ; j < (my_params->detailed_lane.size() - 1); j += 1)
		{
			nearest_point = carmen_get_point_nearest_to_path(&status, my_params->detailed_lane[j], my_params->detailed_lane[j + 1], path[i], 0.001);
			if (status == POINT_WITHIN_SEGMENT)
				break;
		}

		angle = ANGLE2D(path[i], nearest_point);
		bool point_side = (carmen_normalize_theta(path[i].theta - angle) > 0.0)? true: false;
		if (point_side != first_point_side)
			distance = 5.0 * DIST2D(path[i], nearest_point);
		else
			distance = DIST2D(path[i], nearest_point);
		total_distance += distance * distance;
		total_points += 1.0;
	}

	return (total_distance / total_points);
}

#else

double
compute_path_to_lane_distance(ObjectiveFunctionParams *my_params, vector<carmen_robot_and_trailers_path_point_t> &path)
{
	double distance = 0.0;
	double total_distance = 0.0;
	double total_points = 0.0;

	int increment;
	if (use_unity_simulator)
		increment = 1;
	else
		increment = 3;
	for (unsigned int i = 0; i < path.size(); i += increment)
	{
		if ((i < my_params->path_point_nearest_to_lane.size()) &&
			(my_params->path_point_nearest_to_lane.at(i) < my_params->detailed_lane.size()))
		{
			if (GlobalState::reverse_planning)
				distance = DIST2D(path.at(i),
								my_params->detailed_lane.at(my_params->path_point_nearest_to_lane.at(i)));
			else
				distance = DIST2D(move_to_front_axle(path.at(i)),
								my_params->detailed_lane.at(my_params->path_point_nearest_to_lane.at(i)));
			total_points += 1.0;
		}
		else
			distance = 0.0;

		total_distance += distance * distance;
	}

	if (total_points > 0.0)
		return (((total_distance / total_points) > 7.0)? 7.0: total_distance / total_points);
	else
		return (0.0);
}

#endif

vector<carmen_robot_and_trailers_path_point_t>
compute_path_to_lane_distance_evaluation(ObjectiveFunctionParams *my_params, vector<carmen_robot_and_trailers_path_point_t> &path)
{
	int increment;
	if (use_unity_simulator)
		increment = 1;
	else
		increment = 3;

	vector<carmen_robot_and_trailers_path_point_t> modified_path;
	for (unsigned int i = 0; i < path.size(); i += increment)
	{
		if ((i < my_params->path_point_nearest_to_lane.size()) &&
			(my_params->path_point_nearest_to_lane.at(i) < my_params->detailed_lane.size()))
		{
			if (GlobalState::reverse_planning)
				modified_path.push_back(path.at(i));
			else
				modified_path.push_back(move_to_front_axle(path.at(i)));
		}
	}

	return (modified_path);
}


void
compute_path_points_nearest_to_lane(ObjectiveFunctionParams *param, vector<carmen_robot_and_trailers_path_point_t> &path)
{
	param->path_point_nearest_to_lane.clear();
	param->path_size = path.size();
	unsigned int index = 0;
	for (unsigned int j = 0; j < path.size(); j++)
	{
		carmen_robot_and_trailers_path_point_t axle;

#ifdef USE_STANLEY_METHOD
		if (GlobalState::reverse_planning) //mantem o eixo traseiro
			axle = path.at(j);
		else
			axle = move_to_front_axle_WORKING(path.at(j));

#else
		if (GlobalState::reverse_planning) //mantem o eixo traseiro
			axle = path.at(j);
		else
			axle = move_to_front_axle(path.at(j));
#endif

		double min_dist = DIST2D(axle, param->detailed_lane.at(index));
		for (unsigned int i = index; i < param->detailed_lane.size(); i++)
		{
			double distance = DIST2D(axle, param->detailed_lane.at(i));
			if (distance < min_dist)
			{
				min_dist = distance;
				index = i;
			}
		}
		param->path_point_nearest_to_lane.push_back(index);
	}
}


#ifdef USE_STANLEY_METHOD


void
get_between_points(carmen_robot_and_trailer_path_point_t robot, carmen_robot_and_trailer_path_point_t point_before, carmen_robot_and_trailer_path_point_t center, carmen_robot_and_trailer_path_point_t point_next,
		int index_center, int &index_p1, int &index_p2, int &mais_proxima)
{

    double centro = DIST2D(center, robot);
    double d = DIST2D(point_before, robot);
    double d2 = DIST2D(point_next, robot);

    if (d < d2)
    {
//        index_p1 = index_p1;
        index_p2 = index_center;
        mais_proxima = index_p1;
        if(centro < d)
            mais_proxima = index_center;
    }
    else
    {
        index_p1 = index_center;
//        index_p2 = index_center+1;
        mais_proxima = index_p2;
        if(centro < d2)
            mais_proxima = index_center;
    }
}


double
get_distance_between_point_to_line2(carmen_robot_and_trailer_path_point_t p1,
		carmen_robot_and_trailer_path_point_t p2,
		carmen_robot_and_trailer_path_point_t robot)
{
    //https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    double delta_x = p2.x - p1.x;
    double delta_y = p2.y - p1.y;
    double d = sqrt(delta_x * delta_x + delta_y * delta_y);
    double x2y1 =  p2.x * p1.y;
    double y2x1 =  p2.y * p1.x;

    if (d < 0.0000001)
        return DIST2D(p2, robot);

    return fabs((delta_y * robot.x) - (delta_x * robot.y) + x2y1 - y2x1) / d;

}


carmen_robot_and_trailer_path_point_t
get_best_indexes_and_nearest_point(int i, ObjectiveFunctionParams *param, carmen_robot_and_trailer_path_point_t robot_point)
{
	unsigned int index_p1;
	unsigned int index_p2;
	int point_in_trajectory_is = -1;
	carmen_robot_and_trailer_path_point_t ponto_mais_proximo;

	index_p1 = (param->path_point_nearest_to_lane.at(i) > 0) ? (param->path_point_nearest_to_lane.at(i) - 1) : param->path_point_nearest_to_lane.at(i);
	if ((param->path_point_nearest_to_lane.at(i) == index_p1) && (param->path_point_nearest_to_lane.at(i) < param->detailed_lane.size() - 2))
		index_p2 = param->path_point_nearest_to_lane.at(i) + 1;
	else
		index_p2 = param->path_point_nearest_to_lane.at(i);

	ponto_mais_proximo = carmen_get_point_nearest_to_path(&point_in_trajectory_is, param->detailed_lane.at(index_p1), param->detailed_lane.at(index_p2), robot_point, 0.005);

	if (point_in_trajectory_is == SEGMENT_TOO_SHORT)
	{
		printf("SEGMENT_TOO_SHORT!\n");
		if(index_p2 < (param->detailed_lane.size() - 2))
		{
			index_p2 +=1;
			ponto_mais_proximo = carmen_get_point_nearest_to_path(&point_in_trajectory_is, param->detailed_lane.at(index_p1), param->detailed_lane.at(index_p2), robot_point, 0.005);
		}
	}

	if (point_in_trajectory_is == POINT_AFTER_SEGMENT)
	{
		for(unsigned int j = index_p2; j < param->detailed_lane.size() - 2; j++)
		{
			ponto_mais_proximo = carmen_get_point_nearest_to_path(&point_in_trajectory_is, param->detailed_lane.at(j), param->detailed_lane.at(j+1), robot_point, 0.005);
			if (point_in_trajectory_is == POINT_WITHIN_SEGMENT)
				break;
		}
		if (point_in_trajectory_is != POINT_WITHIN_SEGMENT)
			ponto_mais_proximo = carmen_get_point_nearest_to_path(&point_in_trajectory_is, param->detailed_lane.at(index_p1), param->detailed_lane.at(index_p2), robot_point, 0.005);
	}

	else if (point_in_trajectory_is == POINT_BEFORE_SEGMENT)
	{
		for(unsigned int j = index_p1; j > 0; j--)
		{
			ponto_mais_proximo = carmen_get_point_nearest_to_path(&point_in_trajectory_is, param->detailed_lane.at(j), param->detailed_lane.at(j+1), robot_point, 0.005);
			if (point_in_trajectory_is == POINT_WITHIN_SEGMENT)
				break;
		}
	}

	return ponto_mais_proximo;
}


double
compute_error_using_stanley_method(ObjectiveFunctionParams *param, vector<carmen_robot_and_trailer_path_point_t> &path)
{
	double total_steering_error = 0.0;
	double theta_error = 0.0;
	int look_ahead = 5;
	double k = GlobalState::look_ahead_horizon;
	int total_points = 0.0; //How many
	double steerig_stanley = 0.0;
	carmen_robot_and_trailer_path_point_t robot_point;
	double error_steering = 0;
	carmen_robot_and_trailer_path_point_t ponto_mais_proximo;

	for (unsigned int i = 0; i < path.size(); i += look_ahead)
	{
		if ((i < param->path_point_nearest_to_lane.size()) &&
				(param->path_point_nearest_to_lane.at(i) < param->detailed_lane.size()))
		{
			robot_point = move_to_front_axle(path.at(i));

			if (!GlobalState::reverse_planning)
				robot_point = move_to_front_axle_WORKING(path.at(i));

			ponto_mais_proximo = get_best_indexes_and_nearest_point(i, param, robot_point);

			theta_error = carmen_normalize_theta(ponto_mais_proximo.theta - path.at(i).theta);
			double Efa = DIST2D(ponto_mais_proximo, robot_point);

			//double Efa = get_distance_between_point_to_line2(param->detailed_lane.at(param->path_point_nearest_to_lane.at(index_p1)),  param->detailed_lane.at(param->path_point_nearest_to_lane.at(index_p2)), robot_point);

			double vt = (ponto_mais_proximo.v != 0.0) ? ponto_mais_proximo.v : (ponto_mais_proximo.v + 0.00001);

			steerig_stanley = theta_error + atan(k*Efa/vt);
			//vi por ai: usar o atan2(v,k*Efa) < funciona?

			steerig_stanley = (steerig_stanley > GlobalState::robot_config.max_phi) ? GlobalState::robot_config.max_phi : steerig_stanley;
			steerig_stanley = (steerig_stanley < -GlobalState::robot_config.max_phi) ? -GlobalState::robot_config.max_phi : steerig_stanley;

			error_steering = carmen_normalize_theta(steerig_stanley - path.at(i).phi);
//			printf("Atan2: %lf Efa: %lf theta_error: %lf atan: %lf \n", atan2(vt,k*Efa), Efa, theta_error, atan(k*Efa/vt));
			total_points += 1.0;
		}
		else
		{
			error_steering = 0.0;
			total_points += 1.0;
		}

		total_steering_error += (error_steering * error_steering);
	}

	return (total_steering_error/total_points);
}

#endif

inline carmen_ackerman_path_point_t
move_path_point_to_map_coordinates(const carmen_ackerman_path_point_t point, double displacement)
{
	carmen_ackerman_path_point_t path_point_in_map_coords;
	double coss, sine;

	sincos(point.theta, &sine, &coss);
	double x_disp = point.x + displacement * coss;
	double y_disp = point.y + displacement * sine;

	sincos(GlobalState::localizer_pose->theta, &sine, &coss);
	path_point_in_map_coords.x = (GlobalState::localizer_pose->x - GlobalState::distance_map->config.x_origin + x_disp * coss - y_disp * sine) / GlobalState::distance_map->config.resolution;
	path_point_in_map_coords.y = (GlobalState::localizer_pose->y - GlobalState::distance_map->config.y_origin + x_disp * sine + y_disp * coss) / GlobalState::distance_map->config.resolution;

	return (path_point_in_map_coords);
}


double
compute_proximity_to_obstacles_using_distance_map(vector<carmen_robot_and_trailers_path_point_t> path)
{
	double proximity_to_obstacles_for_path = 0.0;
	double safety_distance = GlobalState::robot_config.model_predictive_planner_obstacles_safe_distance;

	for (unsigned int i = 0; i < path.size(); i += 1)
	{
		carmen_robot_and_trailers_pose_t point_to_check = {path[i].x, path[i].y, path[i].theta, path[i].num_trailers, {0.0}};

		for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
			point_to_check.trailer_theta[z] = path[i].trailer_theta[z];

		double proximity_point = carmen_obstacle_avoider_proximity_to_obstacles(GlobalState::localizer_pose,
				point_to_check, GlobalState::distance_map, safety_distance);

		proximity_to_obstacles_for_path += proximity_point;
//		carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, carmen_get_time());
//		getchar();
	}
/*
	if (proximity_to_obstacles_for_path > 100.0)
	{
		for (unsigned int i = 0; i < path.size(); i += 1)
		{
			carmen_robot_and_trailers_pose_t point_to_check = {path[i].x, path[i].y, path[i].theta, path[i].num_trailers, {0.0}};

					for (size_t z = 0; z < MAX_NUM_TRAILERS; z++)
						point_to_check.trailer_theta[z] = path[i].trailer_theta[z];

					double proximity_point = carmen_obstacle_avoider_proximity_to_obstacles(GlobalState::localizer_pose,
							point_to_check, GlobalState::distance_map, safety_distance);

//			printf("p %d %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", i, path[i].x, path[i].y, path[i].theta, path[i].trailer_theta[0], path[i].trailer_theta[1], path[i].trailer_theta[2], path[i].trailer_theta[3], path[i].trailer_theta[4], proximity_point);

			carmen_robot_and_trailers_pose_t temp_pose;


			temp_pose.x = GlobalState::localizer_pose->x + (path[i].x * cos(GlobalState::localizer_pose->theta + path[i].theta) - path[i].y * sin(GlobalState::localizer_pose->theta + path[i].theta));
			temp_pose.y = GlobalState::localizer_pose->y + (path[i].x * sin(GlobalState::localizer_pose->theta + path[i].theta) + path[i].y * cos(GlobalState::localizer_pose->theta + path[i].theta));
			temp_pose.theta = GlobalState::localizer_pose->theta + path[i].theta;
			temp_pose.trailer_theta[0] = convert_beta_to_theta1(temp_pose.theta, convert_theta1_to_beta(path[i].theta, path[i].trailer_theta[0]));
			temp_pose.trailer_theta[1] = convert_beta_to_theta1(temp_pose.theta, convert_theta1_to_beta(path[i].theta, path[i].trailer_theta[1]));
//			printf("l %d %lf %lf %lf %lf %lf\n", i, temp_pose.x, temp_pose.y, temp_pose.theta, temp_pose.trailer_theta[0], temp_pose.trailer_theta[1]);

		}

	}
*/
	return (proximity_to_obstacles_for_path);
}


double
get_distance_dependent_activation_factor(double threshold __attribute__((unused)), ObjectiveFunctionParams *my_params __attribute__((unused)))
{
	double activation_factor = 1.0;
	return (activation_factor);
}


//double
//get_distance_dependent_activation_factor(double threshold, ObjectiveFunctionParams *my_params)
//{
//	double activation_factor = 1.0;
//
//	if (my_params->target_td->dist < threshold)
//	{
//		if (my_params->target_td->dist > (threshold - 1.0))
//			activation_factor = my_params->target_td->dist - (threshold - 1.0);
//		else
//			activation_factor = 0.0;
//	}
//
//	return (activation_factor);
//}


double
get_beta_activation_factor()
{
	if (GlobalState::semi_trailer_config.num_semi_trailers == 0)
		return (0.0);
	else
		return (100.0);
}


double
compute_semi_trailer_to_goal_distance(vector<carmen_robot_and_trailers_path_point_t> path, TrajectoryDimensions *target_td)
{
	if (path.size() == 0)
		return (0.0);

	carmen_robot_and_trailers_path_point_t robot_pose = path[path.size() - 1];
	carmen_robot_and_trailers_pose_t expected_robot_pose = target_td->goal_pose;

	carmen_point_t semi_trailer_pose, expected_semi_trailer_pose;


//	double beta = convert_theta1_to_beta(robot_pose.theta, robot_pose.trailer_theta[0]);

	semi_trailer_pose.x = robot_pose.x - GlobalState::semi_trailer_config.semi_trailers[0].M * cos(robot_pose.theta) - GlobalState::semi_trailer_config.semi_trailers[0].d * cos(robot_pose.trailer_theta[0]);
	semi_trailer_pose.y	= robot_pose.y - GlobalState::semi_trailer_config.semi_trailers[0].M * sin(robot_pose.theta) - GlobalState::semi_trailer_config.semi_trailers[0].d * sin(robot_pose.trailer_theta[0]);

//	double expected_beta = convert_theta1_to_beta(expected_robot_pose.theta, expected_robot_pose.trailer_theta[0]);
	expected_semi_trailer_pose.x = expected_robot_pose.x - GlobalState::semi_trailer_config.semi_trailers[0].M * cos(expected_robot_pose.theta) - GlobalState::semi_trailer_config.semi_trailers[0].d * cos(expected_robot_pose.trailer_theta[0]);
	expected_semi_trailer_pose.y = expected_robot_pose.y - GlobalState::semi_trailer_config.semi_trailers[0].M * sin(expected_robot_pose.theta) - GlobalState::semi_trailer_config.semi_trailers[0].d * sin(expected_robot_pose.trailer_theta[0]);

	double semi_trailer_to_goal_distance = DIST2D(semi_trailer_pose, expected_semi_trailer_pose);

	if (GlobalState::semi_trailer_config.num_semi_trailers > 1)
	{
		carmen_point_t semi_trailers_poses[MAX_NUM_TRAILERS];
		carmen_point_t expected_semi_trailers_poses[MAX_NUM_TRAILERS];
		semi_trailers_poses[0] = semi_trailer_pose;
		expected_semi_trailers_poses[0] = expected_semi_trailer_pose;

		for (int i = 1; i < GlobalState::semi_trailer_config.num_semi_trailers; i++)
		{
			semi_trailers_poses[i].x = semi_trailers_poses[i-1].x - GlobalState::semi_trailer_config.semi_trailers[i - 1].M * cos(robot_pose.trailer_theta[i-1]) - GlobalState::semi_trailer_config.semi_trailers[i - 1].d * cos(robot_pose.trailer_theta[i]);
			semi_trailers_poses[i].y = semi_trailers_poses[i-1].y - GlobalState::semi_trailer_config.semi_trailers[i - 1].M * sin(robot_pose.trailer_theta[i-1]) - GlobalState::semi_trailer_config.semi_trailers[i - 1].d * sin(robot_pose.trailer_theta[i]);
			semi_trailers_poses[i].theta = robot_pose.trailer_theta[i];

			expected_semi_trailers_poses[i].x = expected_semi_trailers_poses[i-1].x - GlobalState::semi_trailer_config.semi_trailers[i - 1].M * cos(expected_robot_pose.trailer_theta[i-1]) - GlobalState::semi_trailer_config.semi_trailers[i - 1].d * cos(expected_robot_pose.trailer_theta[i]);
			expected_semi_trailers_poses[i].y = expected_semi_trailers_poses[i-1].y - GlobalState::semi_trailer_config.semi_trailers[i - 1].M * sin(expected_robot_pose.trailer_theta[i-1]) - GlobalState::semi_trailer_config.semi_trailers[i - 1].d * sin(expected_robot_pose.trailer_theta[i]);
			expected_semi_trailers_poses[i].theta = expected_robot_pose.trailer_theta[i];
			semi_trailer_to_goal_distance += DIST2D(semi_trailers_poses[i], expected_semi_trailers_poses[i]);
		}

	}

	return (semi_trailer_to_goal_distance);
}


double
mpp_optimization_function_f(const gsl_vector *x, void *params)
{
	ObjectiveFunctionParams *my_params = (ObjectiveFunctionParams *) params;
	TrajectoryControlParameters tcp = fill_in_tcp(x, my_params);

	if (bad_tcp(tcp))
		return (1000000.0);

	TrajectoryDimensions td;
	vector<carmen_robot_and_trailers_path_point_t> path = simulate_car_from_parameters(td, tcp, my_params->target_td->v_i, my_params->target_td->trailer_theta_i);

//	double beta_activation_factor = get_beta_activation_factor();
	my_params->plan_cost = ((td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index +
//		beta_activation_factor * (carmen_normalize_theta(td.trailer_theta[0] - my_params->target_td->beta) * carmen_normalize_theta(td.trailer_theta[0] - my_params->target_td->beta)) / 1.0 +
		(carmen_normalize_theta(td.theta - my_params->target_td->theta) * carmen_normalize_theta(td.theta - my_params->target_td->theta)) / (my_params->theta_by_index * 2.0) +
		(carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw) * carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw)) / (my_params->d_yaw_by_index * 2.0));

	double semi_trailer_to_goal_distance = 0.0;
//	if ((GlobalState::behavior_selector_task == BEHAVIOR_SELECTOR_PARK_SEMI_TRAILER) ||
//		(GlobalState::behavior_selector_task == BEHAVIOR_SELECTOR_PARK_TRUCK_SEMI_TRAILER))
	if (( (GlobalState::semi_trailer_config.num_semi_trailers != 0) && (GlobalState::route_planner_state ==  EXECUTING_OFFROAD_PLAN) ) || (GlobalState::semi_trailer_config.num_semi_trailers > 1))
		semi_trailer_to_goal_distance = compute_semi_trailer_to_goal_distance(path, my_params->target_td);
//		semi_trailer_to_goal_distance = carmen_normalize_theta(carmen_radians_to_degrees(td.trailer_theta[0]) - carmen_radians_to_degrees(my_params->target_td->beta)) *
//				carmen_normalize_theta(carmen_radians_to_degrees(td.trailer_theta[0]) - carmen_radians_to_degrees(my_params->target_td->beta));

	double w2_activation_factor = get_distance_dependent_activation_factor(2.0, my_params);
	double result = sqrt(
				GlobalState::w1 * (td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index +
//				beta_activation_factor * (carmen_normalize_theta(td.trailer_theta[0] - my_params->target_td->beta) * carmen_normalize_theta(td.trailer_theta[0] - my_params->target_td->beta)) / 1.0 +
				w2_activation_factor * 4.0 * GlobalState::w2 * (carmen_normalize_theta(td.theta - my_params->target_td->theta) * carmen_normalize_theta(td.theta - my_params->target_td->theta)) / my_params->theta_by_index +
				GlobalState::w3 * (carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw) * carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw)) / my_params->d_yaw_by_index +
				GlobalState::w6 * semi_trailer_to_goal_distance * semi_trailer_to_goal_distance);

	return (result);
}


void
my_df(const gsl_vector *v, void *params, gsl_vector *df)
{
	ObjectiveFunctionParams *my_params = (ObjectiveFunctionParams *) params;
	int size = my_params->tcp_seed->k.size();

	gsl_vector *x_h;
	x_h = gsl_vector_alloc(size);
	gsl_vector_memcpy(x_h, v);

	double h = 0.00005;
	double f_x = my_params->my_f(v, params);
	for (int i = 0; i < size; i++)
	{
		gsl_vector_set(x_h, i, gsl_vector_get(v, i) + h);
		double f_x_h = my_params->my_f(x_h, params);
		double d_f_x_h = (f_x_h - f_x) / h;
		gsl_vector_set(df, i, d_f_x_h);
		gsl_vector_set(x_h, i, gsl_vector_get(v, i));
	}

	gsl_vector_free(x_h);
}


void
my_fdf(const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
	ObjectiveFunctionParams *my_params = (ObjectiveFunctionParams *) params;
	*f = my_params->my_f(x, params);
	my_df(x, params, df);
}

int print_ws = 0;

double
mpp_optimization_function_g(const gsl_vector *x, void *params)
{
	ObjectiveFunctionParams *my_params = (ObjectiveFunctionParams *) params;
	TrajectoryControlParameters tcp = fill_in_tcp(x, my_params);
	double steering_error = 0.0;

	if (bad_tcp(tcp))
		return (1000000.0);

	TrajectoryDimensions td;
	vector<carmen_robot_and_trailers_path_point_t> path = simulate_car_from_parameters(td, tcp, my_params->target_td->v_i, my_params->target_td->trailer_theta_i);

	double path_to_lane_distance = 0.0;
	if (my_params->use_lane && (my_params->detailed_lane.size() > 0) && (path.size() > 0))
	{
#ifdef NEW_PATH_TO_LANE_DISTANCE
		path_to_lane_distance = compute_path_to_lane_distance(my_params, path);
#else

		if (path.size() != my_params->path_size)
		{
			compute_path_points_nearest_to_lane(my_params, path);
		}
		path_to_lane_distance = compute_path_to_lane_distance(my_params, path);

#ifdef USE_STANLEY_METHOD
		steering_error = compute_error_using_stanley_method(my_params, path);
		printf("\nSteering: %lf ", steering_error);
#endif
#endif
	}

	double proximity_to_obstacles = 0.0;
	if (use_obstacles && GlobalState::distance_map != NULL && path.size() > 0)
		proximity_to_obstacles = compute_proximity_to_obstacles_using_distance_map(path);

#ifdef USE_STANLEY_METHOD
		my_params->plan_cost = sqrt((steering_error/2));
#else
//	double beta_activation_factor = get_beta_activation_factor();
	my_params->plan_cost = sqrt((td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index +
			(carmen_normalize_theta(td.theta - my_params->target_td->theta) * carmen_normalize_theta(td.theta - my_params->target_td->theta)) / (my_params->theta_by_index * 0.2) +
//			beta_activation_factor * (carmen_normalize_theta(td.trailer_theta[0] - my_params->target_td->beta) * carmen_normalize_theta(td.trailer_theta[0] - my_params->target_td->beta)) / 1.0 +
			(carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw) * carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw)) / (my_params->d_yaw_by_index * 0.2));
#endif

	double w2_activation_factor = get_distance_dependent_activation_factor(2.0, my_params);
	double w4_activation_factor = get_distance_dependent_activation_factor(6.0, my_params);

	double semi_trailer_to_goal_distance = 0.0;
//	if ((GlobalState::behavior_selector_task == BEHAVIOR_SELECTOR_PARK_SEMI_TRAILER) ||
//		(GlobalState::behavior_selector_task == BEHAVIOR_SELECTOR_PARK_TRUCK_SEMI_TRAILER))
	if (( (GlobalState::semi_trailer_config.num_semi_trailers != 0) && (GlobalState::route_planner_state ==  EXECUTING_OFFROAD_PLAN) ) || (GlobalState::semi_trailer_config.num_semi_trailers > 1))
		semi_trailer_to_goal_distance = compute_semi_trailer_to_goal_distance(path, my_params->target_td);
//		semi_trailer_to_goal_distance = carmen_normalize_theta(carmen_radians_to_degrees(td.trailer_theta[0]) - carmen_radians_to_degrees(my_params->target_td->beta)) *
//				carmen_normalize_theta(carmen_radians_to_degrees(td.trailer_theta[0]) - carmen_radians_to_degrees(my_params->target_td->beta));

	double result = sqrt(
				GlobalState::w1 * ((td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist)) / my_params->distance_by_index +
				w2_activation_factor * GlobalState::w2 * (carmen_normalize_theta(td.theta - my_params->target_td->theta) * carmen_normalize_theta(td.theta - my_params->target_td->theta)) / my_params->theta_by_index +
				GlobalState::w3 * (carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw) * carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw)) / my_params->d_yaw_by_index +
				w4_activation_factor * GlobalState::w4 * path_to_lane_distance + // já é quandrática
//				beta_activation_factor * (carmen_normalize_theta(td.beta - my_params->target_td->beta) * carmen_normalize_theta(td.beta - my_params->target_td->beta)) / 1.0 +
				GlobalState::w5 * proximity_to_obstacles +
				GlobalState::w6 * semi_trailer_to_goal_distance * semi_trailer_to_goal_distance +
				GlobalState::w7 * steering_error);

	if (print_ws)
	{
		printf("* r % 3.8lf, w1 % 3.8lf, w2 % 3.8lf, w3 % 3.8lf, w4 % 3.8lf, w5 % 3.8lf, w6 %lf, ps %d, Steering: % 3.8lf\n", result,
				GlobalState::w1 * ((td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist)) / my_params->distance_by_index,
				w2_activation_factor * GlobalState::w2 * (carmen_normalize_theta(td.theta - my_params->target_td->theta) * carmen_normalize_theta(td.theta - my_params->target_td->theta)) / my_params->theta_by_index,
				GlobalState::w3 * (carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw) * carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw)) / my_params->d_yaw_by_index,
				w4_activation_factor * GlobalState::w4 * path_to_lane_distance,
				GlobalState::w5 * proximity_to_obstacles,
				GlobalState::w6 * semi_trailer_to_goal_distance * semi_trailer_to_goal_distance,
				(int) path.size(),
				(GlobalState::w7 * steering_error));
//		print_tcp(tcp);
	}

	return (result);
}


void
compute_suitable_acceleration_and_tt(ObjectiveFunctionParams &params,
		TrajectoryControlParameters &tcp_seed,
		TrajectoryDimensions target_td, double target_v)
{
	// (i) S = Vo*t + 1/2*a*t^2
	// (ii) dS/dt = Vo + a*t
	// dS/dt = 0 => máximo ou mínimo de S => 0 = Vo + a*t; a*t = -Vo; (iii) a = -Vo/t; (iv) t = -Vo/a
	// Se "a" é negativa, dS/dt = 0 é um máximo de S
	// Logo, como S = target_td.dist, "a" e "t" tem que ser tais em (iii) e (iv) que permitam que
	// target_td.dist seja alcançada.
	//
	// O valor de maxS pode ser computado substituindo (iv) em (i):
	// maxS = Vo*-Vo/a + 1/2*a*(-Vo/a)^2 = -Vo^2/a + 1/2*Vo^2/a = -1/2*Vo^2/a
	//
	// Se estou co velocidade vi e quero chagar a vt, sendo que vt < vi, a eh negativo. O tempo, tt, para
	// ir de vi a vt pode ser derivado de dS/dt = Vo + a*t -> vt = vi + a*tt; a*tt = vt - vi; tt = (vt - vi) / a

	if (!GlobalState::reverse_planning && target_v < 0.0)
		target_v = 0.0;

	tcp_seed.s = target_td.dist; // Pior caso: forcca otimizacao para o primeiro zero da distancia, evitando voltar de reh para atingir a distancia.
	compute_a_and_t_from_s(tcp_seed.s, target_v, target_td, tcp_seed, &params);
}


void
get_optimization_params(ObjectiveFunctionParams &params, double target_v,
		TrajectoryControlParameters *tcp_seed,
		TrajectoryDimensions *target_td,
		double max_plan_cost, int max_iterations,
		double (* my_f) (const gsl_vector  *x, void *params))
{
	params.distance_by_index = fabs(get_distance_by_index(N_DIST - 1));
	params.theta_by_index = fabs(get_theta_by_index(N_THETA - 1));
	params.d_yaw_by_index = fabs(get_d_yaw_by_index(N_D_YAW - 1));
	params.target_td = target_td;
	params.tcp_seed = tcp_seed;
	params.target_v = target_v;
	params.path_size = 0;
	params.my_f = my_f;
	params.max_plan_cost = max_plan_cost;
	params.o_step_size = F_STEP_SIZE;
	params.o_tol = F_TOL;
	params.o_epsabs = F_EPSABS;
	params.max_iterations = max_iterations;
}


void
get_tcp_with_n_knots(TrajectoryControlParameters &tcp, int n)
{
	gsl_spline *phi_spline = get_phi_spline(tcp);
	gsl_interp_accel *acc = gsl_interp_accel_alloc();

	if (n == 4)
	{
		tcp.k.push_back(tcp.k[2]);
		tcp.k[2] = tcp.k[1];
		tcp.k[1] = gsl_spline_eval(phi_spline, tcp.tt / 4.0, acc);
	}
	else
	{
		tcp.k = {};
		for (int i = 0; i < (n - 1); i++)
		{
			double t = (tcp.tt / (double) (n - 1)) * (double) (i);
			tcp.k.push_back(gsl_spline_eval(phi_spline, t, acc));
		}
		tcp.k.push_back(gsl_spline_eval(phi_spline, tcp.tt, acc));

	}
	gsl_spline_free(phi_spline);
	gsl_interp_accel_free(acc);
}


TrajectoryControlParameters
get_optimized_trajectory_control_parameters(TrajectoryControlParameters tcp_seed, ObjectiveFunctionParams &params)
{
	int current_eliminate_path_follower_value = GlobalState::eliminate_path_follower;
	GlobalState::eliminate_path_follower = 0;

	gsl_multimin_function_fdf my_func;

	// O phi inicial, que eh guardado em tcp_seed.k[0], nao eh otimizado, mas usamos o tamanho de k aqui
	// pois as variaveis otimizadas incluem o s. Logo, o tamanho do problema fica igual ao numero de nos mais o phi
	// inicial, que eh guardado em k[0].
	my_func.n = tcp_seed.k.size();
	my_func.f = params.my_f;
	my_func.df = my_df;
	my_func.fdf = my_fdf;
	my_func.params = &params;

	/* Starting point, x */
	gsl_vector *x = gsl_vector_alloc(tcp_seed.k.size());
	gsl_vector_set(x, 0, tcp_seed.s);
	for (unsigned int i = 1; i < tcp_seed.k.size(); i++)
		gsl_vector_set(x, i, tcp_seed.k[i]);

	const gsl_multimin_fdfminimizer_type *T = gsl_multimin_fdfminimizer_conjugate_fr;
	gsl_multimin_fdfminimizer *s = gsl_multimin_fdfminimizer_alloc(T, tcp_seed.k.size());

	gsl_multimin_fdfminimizer_set(s, &my_func, x, params.o_step_size, params.o_tol);

	int iter = 0;
	int status;
	do
	{
		iter++;

		status = gsl_multimin_fdfminimizer_iterate(s);

		if (status)
			break;

		status = gsl_multimin_test_gradient(s->gradient, params.o_epsabs); // esta funcao retorna GSL_CONTINUE ou zero

	} while ((status == GSL_CONTINUE) && (iter < params.max_iterations));
//	} while ((iter < params.max_iterations)); //Force max int to test
	//	} while ((status == GSL_CONTINUE));//Force max int to test

	TrajectoryControlParameters tcp = fill_in_tcp(s->x, &params);

	if (bad_tcp(tcp))
		tcp.valid = false;

	if ((tcp.tt < 0.05) || (params.plan_cost > params.max_plan_cost)) // too short plan or bad minimum (s->f should be close to zero) mudei de 0.05 para outro
		tcp.valid = false;

	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

	GlobalState::eliminate_path_follower = current_eliminate_path_follower_value;

	return (tcp);
}


vector<carmen_robot_and_trailers_path_point_t>
move_detailed_lane_to_front_axle(vector<carmen_robot_and_trailers_path_point_t> &detailed_lane)
{
	for (unsigned int i = 0; i < detailed_lane.size(); i++)
		detailed_lane[i] = (move_to_front_axle(detailed_lane[i]));

	return (detailed_lane);
}


double
get_path_to_lane_distance(TrajectoryDimensions td,
		TrajectoryControlParameters tcp, ObjectiveFunctionParams *my_params)
{
	vector<carmen_robot_and_trailers_path_point_t> path = simulate_car_from_parameters(td, tcp, my_params->target_td->v_i, my_params->target_td->trailer_theta_i);
	double path_to_lane_distance = 0.0;
	if (my_params->use_lane && (my_params->detailed_lane.size() > 0) && (path.size() > 0))
	{
		compute_path_points_nearest_to_lane(my_params, path);
		path_to_lane_distance = compute_path_to_lane_distance(my_params, path);
	}
	return (path_to_lane_distance);
}




TrajectoryControlParameters
get_n_knots_tcp_from_detailed_lane(vector<carmen_robot_and_trailers_path_point_t> detailed_lane,
		int n, double v_i, double phi_i, double d_yaw, double a, double s, double tt)
{
	TrajectoryControlParameters tcp = {};
	double L = GlobalState::robot_config.distance_between_front_and_rear_axles;

	tcp.valid = true;
	tcp.a = a;
	tcp.s = s;
	tcp.tt = tt;
	tcp.vf = v_i + tcp.a * tcp.tt;

	unsigned int size = n;

	vector<double> knots_x;
	for (unsigned int i = 0; i < size; i++)
	{
		double x = (tcp.tt / (double) (size - 1)) * (double) i;
		knots_x.push_back(x);
	}
	knots_x[size - 1] = tcp.tt; // Para evitar que erros de arredondamento na conta de x, acima, atrapalhe a leitura do ultimo ponto no spline
	if (size == 4)
	{
		knots_x[1] = tcp.tt / 4.0;
		knots_x[2] = tcp.tt / 2.0;
	}

	tcp.k = {};
	tcp.k.push_back(phi_i);
	if (detailed_lane.size() >= 3)
	{
		unsigned int j = 1;
		double s_consumed = 0.0;
		for (unsigned int i = 1; i < size; i++)
		{
			double t = knots_x[i];
			double s = fabs(v_i * t + 0.5 * tcp.a * t * t);
			unsigned int j_ini = j;
			double s_ini = s_consumed;
			for ( ; j < detailed_lane.size() - 1; j++)
			{
				s_consumed += DIST2D(detailed_lane[j], detailed_lane[j + 1]);
				if (s_consumed > s)
					break;
			}
			double delta_theta = carmen_normalize_theta(detailed_lane[j].theta - detailed_lane[j_ini].theta);
			double phi = atan((L * delta_theta) / (s_consumed - s_ini));

			if (GlobalState::reverse_planning)
				tcp.k.push_back(-carmen_normalize_theta((phi + tcp.k[i - 1]) / 2.0));
			else
				tcp.k.push_back(carmen_normalize_theta((phi + tcp.k[i - 1]) / 2.0));
		}

		tcp.sf = s_consumed;
	}
	else
	{
		double s_ini = 0.0;
		double s = 0.0;
		for (unsigned int i = 1; i < size; i++)
		{
			double t = knots_x[i];
			s = fabs(v_i * t + 0.5 * tcp.a * t * t);
			double delta_theta = carmen_normalize_theta((d_yaw / tcp.tt) * t);
			double phi = atan((L * delta_theta) / (s - s_ini));

			if (GlobalState::reverse_planning)
				tcp.k.push_back(-carmen_normalize_theta((phi + tcp.k[i - 1]) / 2.0));
			else
				tcp.k.push_back(carmen_normalize_theta((phi + tcp.k[i - 1]) / 2.0));

			s_ini = s;
		}

		tcp.sf = s;
	}

	return (tcp);
}


TrajectoryControlParameters
reduce_tcp_to_3_knots(TrajectoryControlParameters previous_tcp)
{
	TrajectoryControlParameters tcp = previous_tcp;

	if (previous_tcp.k.size() == 4)
	{
		tcp.k[1] = tcp.k[2];
		tcp.k[2] = tcp.k[3];
		tcp.k.pop_back();
	}
	else
	{
		tcp.k = {};
		tcp.k.push_back(previous_tcp.k[0]);
		tcp.k.push_back(previous_tcp.k[tcp.k.size() / 2]);
		tcp.k.push_back(previous_tcp.k[tcp.k.size() - 1]);
	}

	return (tcp);
}


bool
more_knots_required(TrajectoryDimensions target_td)
{
	double threshold_v = GlobalState::param_parking_speed_limit * 1.4;
	if (target_td.v_i > threshold_v)
		return (true);
	else
		return (false);
}


TrajectoryControlParameters
get_complete_optimized_trajectory_control_parameters(TrajectoryControlParameters previous_tcp,
		TrajectoryDimensions target_td, double target_v,
		vector<carmen_robot_and_trailers_path_point_t> detailed_lane, bool use_lane)
{
	GlobalState::eliminate_path_follower = 0;
	//	virtual_laser_message.num_positions = 0;

	ObjectiveFunctionParams params;
	params.use_lane = use_lane;
//	detailed_lane = {};
	if (detailed_lane.size() > 1)
	{
		if (GlobalState::reverse_planning)
			params.detailed_lane = detailed_lane;
		else
			params.detailed_lane = move_detailed_lane_to_front_axle(detailed_lane);
	}
	else
		params.use_lane = false;

	int max_iterations;
//	if ((GlobalState::behavior_selector_task == BEHAVIOR_SELECTOR_PARK_SEMI_TRAILER) ||
//		(GlobalState::behavior_selector_task == BEHAVIOR_SELECTOR_PARK_TRUCK_SEMI_TRAILER) ||
//		(GlobalState::behavior_selector_task == BEHAVIOR_SELECTOR_PARK))
	if (((GlobalState::semi_trailer_config.num_semi_trailers != 0) && (GlobalState::route_planner_state ==  EXECUTING_OFFROAD_PLAN)) ||
		(target_td.dist < GlobalState::distance_between_waypoints / 1.5))
		max_iterations = 150;
	else
//		max_iterations = 15;
		max_iterations = 30;

	TrajectoryControlParameters tcp_seed;
	if (!previous_tcp.valid)
	{
		get_optimization_params(params, target_v, &tcp_seed, &target_td, 2.0, max_iterations, mpp_optimization_function_f);
		compute_suitable_acceleration_and_tt(params, tcp_seed, target_td, target_v);
		tcp_seed = get_n_knots_tcp_from_detailed_lane(detailed_lane, 3,
				target_td.v_i, target_td.phi_i, target_td.d_yaw, tcp_seed.a,  tcp_seed.s, tcp_seed.tt);	// computa tcp com tres nos
	}
	else
	{
		tcp_seed = reduce_tcp_to_3_knots(previous_tcp);
		get_optimization_params(params, target_v, &tcp_seed, &target_td, 2.0, max_iterations, mpp_optimization_function_f);
		compute_suitable_acceleration_and_tt(params, tcp_seed, target_td, target_v);
	}

#ifdef PUBLISH_PLAN_TREE
	TrajectoryControlParameters  tcp_copy;
#endif

	// Precisa chamar o otimizador abaixo porque o seguinte (optimized_lane_trajectory_control_parameters()) pode ficar preso em minimo local
	// quando de obstaculos. O otimizador abaixo nao considera obstaculos ou a detailed_lane.
	TrajectoryControlParameters tcp_complete = get_optimized_trajectory_control_parameters(tcp_seed, params);
#ifdef PUBLISH_PLAN_TREE
	tcp_copy = tcp_complete;
#endif

	if (!tcp_complete.valid)
		return (tcp_complete);

//	if (more_knots_required(target_td))
//	if ((GlobalState::behavior_selector_task == BEHAVIOR_SELECTOR_PARK_SEMI_TRAILER) ||
//		(GlobalState::behavior_selector_task == BEHAVIOR_SELECTOR_PARK_TRUCK_SEMI_TRAILER) ||
//		(GlobalState::behavior_selector_task == BEHAVIOR_SELECTOR_PARK) ||
//		(target_td.dist < GlobalState::distance_between_waypoints / 1.5))
	if (((GlobalState::semi_trailer_config.num_semi_trailers != 0) && (GlobalState::route_planner_state ==  EXECUTING_OFFROAD_PLAN)) ||
#ifdef USE_STEFFEN_SPLINE
		(target_td.dist < GlobalState::distance_between_waypoints / 1.5))
#else
		(target_td.dist < GlobalState::distance_between_waypoints / 1.5))
#endif
		get_tcp_with_n_knots(tcp_complete, 3);
	else
		get_tcp_with_n_knots(tcp_complete, 4);

	get_optimization_params(params, target_v, &tcp_complete, &target_td, 2.5, max_iterations, mpp_optimization_function_g);
	tcp_complete = get_optimized_trajectory_control_parameters(tcp_complete, params);

//	plot_phi_profile(tcp_complete);
//	plot_a_profile(tcp_complete);
//	print_tcp(tcp_complete, carmen_get_time());

#ifdef PUBLISH_PLAN_TREE
	TrajectoryDimensions td = target_td;
	TrajectoryControlParameters tcp = tcp_complete;
	tcp.valid = true;
	vector<carmen_robot_and_trailers_path_point_t> path = simulate_car_from_parameters(td, tcp, td.v_i, td.trailer_theta[0]_i);
	print_lane(path, (char *) "caco.txt");
	vector<carmen_robot_and_trailers_path_point_t> path_prev = simulate_car_from_parameters(td, tcp_copy, td.v_i, td.trailer_theta[0]_i);

	ObjectiveFunctionParams params_copy = params;
	if (params_copy.detailed_lane.size() > 0)
	{
		Tree tree;
		tree.num_paths = 2;
		tree.num_edges = 0;
		tree.p1 = NULL;
		tree.p2 = NULL;
		tree.paths = (carmen_robot_and_trailers_traj_point_t **) malloc(tree.num_paths * sizeof(carmen_robot_and_trailers_traj_point_t *));
		tree.paths_sizes = (int *) malloc(tree.num_paths * sizeof(int));

		move_path_to_current_robot_pose(path_prev, GlobalState::localizer_pose);
		tree.paths[0] = (carmen_robot_and_trailers_traj_point_t *) malloc(path_prev.size() * sizeof(carmen_robot_and_trailers_traj_point_t));
		copy_path_to_traj(tree.paths[0], path_prev);
		tree.paths_sizes[0] = (path_prev.size() >= CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE)? CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE - 1: path_prev.size();

//		compute_path_points_nearest_to_lane(&params_copy, path);
//		vector<carmen_robot_and_trailer_path_point_t> modified_path = compute_path_to_lane_distance_evaluation(&params_copy, path);
		vector<carmen_robot_and_trailers_path_point_t> modified_path = path;
		move_path_to_current_robot_pose(modified_path, GlobalState::localizer_pose);
		tree.paths[1] = (carmen_robot_and_trailers_traj_point_t *) malloc(modified_path.size() * sizeof(carmen_robot_and_trailers_traj_point_t));
		copy_path_to_traj(tree.paths[1], modified_path);
		tree.paths_sizes[1] = (modified_path.size() >= CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE)? CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE - 1: modified_path.size();

//		move_path_to_current_robot_pose(params_copy.detailed_lane, GlobalState::localizer_pose);
//		tree.paths[2] = (carmen_robot_and_trailer_traj_point_t *) malloc(params_copy.detailed_lane.size() * sizeof(carmen_robot_and_trailer_traj_point_t));
//		copy_path_to_traj(tree.paths[2], params_copy.detailed_lane);
//		tree.paths_sizes[2] = (params_copy.detailed_lane.size() >= CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE)? CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE - 1: params_copy.detailed_lane.size();

		Publisher_Util::publish_plan_tree_message(tree);
	}
#endif
	GlobalState::eliminate_path_follower = 1;

	return (tcp_complete);
}
