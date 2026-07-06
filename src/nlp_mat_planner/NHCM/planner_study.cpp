#include <stdio.h>
#include <carmen/task_manager_interface.h>
#ifdef _REMOVE_BOOST
	// #include <boost/numeric/odeint.hpp>
#else
	#include <boost/numeric/odeint.hpp>
#endif
#include <gsl/gsl_linalg.h>
#include "g2o/types/slam2d/se2.h"
#include "../nlp_mat_planner_path_planner_astar.h"
#include "../trailer_analytical_expansion.h"

using namespace std;

#ifdef _REMOVE_BOOST
	// using namespace boost::numeric::odeint;
#else 
	using namespace boost::numeric::odeint;
#endif
using namespace g2o;


#define PLOT_PATH

carmen_robot_ackerman_config_t robot_config = {};
carmen_semi_trailers_config_t semi_trailer_config = {};
nonholonomic_heuristic_cost_trailer_p *****cost_map_trailer;

double precomputed_cost_size_trailer;
double precomputed_cost_resolution_trailer;
int precomputed_cost_theta_size_trailer, precomputed_cost_beta_size_trailer;
char *precomputed_cost_file_name_trailer;

double max_phi_multiplier;
carmen_path_planner_astar_t astar_config;

extern kuglerParameters_t Parameters;


void
alloc_trailer_cost_map()
{
	int i, j, z, a, b;
	int x_size = round(precomputed_cost_size_trailer / precomputed_cost_resolution_trailer);
	int y_size = round(precomputed_cost_size_trailer / precomputed_cost_resolution_trailer);

	cost_map_trailer = (nonholonomic_heuristic_cost_trailer_p *****) calloc(x_size, sizeof(nonholonomic_heuristic_cost_trailer_p****));
	carmen_test_alloc(cost_map_trailer);

	for (i = 0; i < x_size; i++)
	{
		cost_map_trailer[i] = (nonholonomic_heuristic_cost_trailer_p ****) calloc(y_size, sizeof(nonholonomic_heuristic_cost_trailer_p***));
		carmen_test_alloc(cost_map_trailer[i]);

		for (j = 0; j < y_size; j++)
		{
			cost_map_trailer[i][j] = (nonholonomic_heuristic_cost_trailer_p***) calloc(precomputed_cost_theta_size_trailer, sizeof(nonholonomic_heuristic_cost_trailer_p**));
			carmen_test_alloc(cost_map_trailer[i][j]);

			for (z = 0; z < precomputed_cost_theta_size_trailer; z++)
			{
				cost_map_trailer[i][j][z]= (nonholonomic_heuristic_cost_trailer_p**) calloc(precomputed_cost_beta_size_trailer, sizeof(nonholonomic_heuristic_cost_trailer_p*));
				carmen_test_alloc(cost_map_trailer[i][j][z]);

				for (a = 0; a < precomputed_cost_beta_size_trailer; a++) // beta do goal
				{
					cost_map_trailer[i][j][z][a]= (nonholonomic_heuristic_cost_trailer_p*) calloc(precomputed_cost_beta_size_trailer, sizeof(nonholonomic_heuristic_cost_trailer_p));
					carmen_test_alloc(cost_map_trailer[i][j][z][a]);

					for (b = 0; b < precomputed_cost_beta_size_trailer; b++) // beta da origem
					{
						cost_map_trailer[i][j][z][a][b]= (nonholonomic_heuristic_cost_trailer_p) malloc(sizeof(nonholonomic_heuristic_cost_trailer));
						carmen_test_alloc(cost_map_trailer[i][j][z][a][b]);
					}
				}
			}
		}
	}
}


void
clear_trailer_cost_map()
{
	int i, j, z, a, b;
	int x_size = round(precomputed_cost_size_trailer  / precomputed_cost_resolution_trailer);
	int y_size = round(precomputed_cost_size_trailer / precomputed_cost_resolution_trailer);

	for (i = 0; i < x_size; i++)
		for (j = 0; j < y_size; j++)
			for (z = 0; z < precomputed_cost_theta_size_trailer; z++)
				for (a = 0; a < precomputed_cost_beta_size_trailer; a++)
					for (b = 0; b < precomputed_cost_beta_size_trailer; b++)
						cost_map_trailer[i][j][z][a][b] = NULL;
}


double
carmen_compute_abs_angular_distance(double theta_1, double theta_2)
{
	return (carmen_normalize_theta(abs(theta_1 - theta_2)));
}


void
plot_path(vector<carmen_robot_and_trailers_traj_point_t> robot_path)
{
	static bool first_time = true;
	static FILE *gnuplot_pipeMP;

	if (first_time)
	{
		first_time = false;

		gnuplot_pipeMP = popen("gnuplot", "w"); // -persist to keep last plot after program closes
		fprintf(gnuplot_pipeMP, "set xrange [%d:%d]\n", (int) ((double) (-precomputed_cost_size_trailer * 0.1 - semi_trailer_config.semi_trailers[0].d)), (int) ((double) (precomputed_cost_size_trailer + semi_trailer_config.semi_trailers[0].d) * 1.1));
		fprintf(gnuplot_pipeMP, "set yrange [%d:%d]\n", (int) ((double) (-precomputed_cost_size_trailer * 0.1 - semi_trailer_config.semi_trailers[0].d)), (int) ((double) (precomputed_cost_size_trailer + semi_trailer_config.semi_trailers[0].d) * 1.1));
		fprintf(gnuplot_pipeMP, "set xlabel 'm'\n");
		fprintf(gnuplot_pipeMP, "set ylabel 'm'\n");
		fprintf(gnuplot_pipeMP, "set tics out\n");
	}

	FILE *gnuplot_data_file = fopen("gnuplot_data_path.txt", "w");

	for (int i = robot_path.size() - 1; i >= 0; i--)
	{
		double distance = 0.0;
		if (i < robot_path.size() - 1)
			distance = DIST2D(robot_path[i], robot_path[i + 1]);

		carmen_robot_and_trailers_traj_point_t semi_trailer_pose;
		semi_trailer_pose.x	    = robot_path[i].x - semi_trailer_config.semi_trailers[0].M * cos(robot_path[i].theta) - semi_trailer_config.semi_trailers[0].d * cos(robot_path[i].theta - robot_path[i].trailer_theta[0]);
		semi_trailer_pose.y	    = robot_path[i].y - semi_trailer_config.semi_trailers[0].M * sin(robot_path[i].theta) - semi_trailer_config.semi_trailers[0].d * sin(robot_path[i].theta - robot_path[i].trailer_theta[0]);
		semi_trailer_pose.theta = robot_path[i].theta - robot_path[i].trailer_theta[0];
		semi_trailer_pose.trailer_theta[0]  = robot_path[i].trailer_theta[0];
		semi_trailer_pose.v	    = robot_path[i].v;
		semi_trailer_pose.phi	= robot_path[i].phi;

		fprintf(gnuplot_data_file, "%lf %lf %lf %lf %lf %lf %lf %lf %lf\n", robot_path.at(i).x, robot_path.at(i).y,
				distance * cos(robot_path.at(i).theta), distance * sin(robot_path.at(i).theta), carmen_normalize_theta(robot_path.at(i).theta),
				robot_path.at(i).phi, robot_path.at(i).trailer_theta[0], semi_trailer_pose.x, semi_trailer_pose.y);
	}
	fclose(gnuplot_data_file);

	fprintf(gnuplot_pipeMP, "plot "
			"'./gnuplot_data_path.txt' using 1:2:3:4 w vectors title 'robot path', "
			"'./gnuplot_data_path.txt' using 8:9 w l title 'semi trailer path'\n");

	fflush(gnuplot_pipeMP);
}


double
trailer_analytical_expansion_cost(carmen_robot_and_trailers_traj_point_t current, carmen_robot_and_trailers_traj_point_t goal, int direction,
		carmen_robot_ackerman_config_t robot_config, carmen_semi_trailers_config_t semi_trailer_config)
{
	double path_cost = 0.0;

	// Incompleto
	// Forwards == 1; Backwards == -1
	vector<carmen_robot_and_trailers_traj_point_t> expanded_path;
	gsl_set_error_handler_off();

	double original_robot_max_phi = robot_config.max_phi;
	robot_config.max_phi = M_PI / 2.0;
	double original_semi_trailer_max_beta = semi_trailer_config.semi_trailers[0].max_beta;
	semi_trailer_config.semi_trailers[0].max_beta = M_PI;

	expanded_path = trailer_polynomial_analytical_expansion(current, goal, direction, robot_config, semi_trailer_config, NULL, max_phi_multiplier, false);

#ifdef PLOT_PATH
	plot_path(expanded_path);
#endif

	robot_config.max_phi = original_robot_max_phi;
	semi_trailer_config.semi_trailers[0].max_beta = original_semi_trailer_max_beta;

	if (expanded_path.size() < 1 || isnan(expanded_path[0].x))
		return (-1);

	for (size_t i = 1; i < expanded_path.size(); i++)
	{
		path_cost += DIST2D(expanded_path[i], expanded_path[i - 1]);
		if (isnan(path_cost))
			return (-1);

		double excess_phi = fabs(expanded_path[i - 1].phi) - original_robot_max_phi;
		if (excess_phi > 0.0)
			path_cost += 10.0 * excess_phi;
		double excess_beta = fabs(expanded_path[i - 1].trailer_theta[0]) - original_semi_trailer_max_beta;
		if (excess_beta > 0.0)
			path_cost += 10.0 * excess_beta;
	}

	return (path_cost);
}


array<double, 4>
TruckTrailerModel_new(array<double, 4> x_T, array<double, 2> u, kuglerParameters_t Parameters, kugler_Ref_t Ref)
{
	// Initialize necessary values for formula implementation
	double d_1 = Parameters.d1;
	double M = -0.8;
	double theta_0 = x_T[2];
	double theta_1 = x_T[3];
	double u_1 = u[0]; // u_1 = v_0

	// Define elements of the Truck/Trailer vector (vehicle model)
	double dx_0 = u_1 * cos(theta_0);
	double dy_0 = u_1 * sin(theta_0);
	double dtheta_0 = u_1 / M * tan(theta_0 - theta_1) - (d_1 / (M * cos(theta_0 - theta_1))) * Ref.dyxx;
	double dtheta_1 = Ref.dyxx;

	// Define Truck/Trailer vector (vehicle model)
	array<double, 4> dx_T = {dx_0,
							 dy_0,
							 dtheta_0,
							 dtheta_1};

	return (dx_T);
}


void
KuglerODEFunc_new(const solver_state_t State, solver_state_t &dState,  double t)
{
	// Calculate reference values of x and y
	kugler_Ref_t Ref;
	double eta, xRef_dot;
	Ref = KuglerCalcRefValues_ODE(eta, xRef_dot, t, Parameters);

	array<double, 4> x_T = {State[0], State[1], State[2], State[3]}; // x_T equals Truck vector [x0, y0, theta0, theta1, phi]
	array<double, 2> u = {0.5 / cos(x_T[2] - x_T[3]) * eta * xRef_dot, 0.0};

	// Simple vehicle model (time parametrized calculated state)
    array<double, 4> dx_T = TruckTrailerModel_new(x_T, u, Parameters, Ref);

    dState = {dx_T[0], dx_T[1], dx_T[2], dx_T[3]};
}


solver_state_t
get_ode_start_state_new(int direction, const kugler_state_t &start_x0, const kugler_state_t &final_x0)
{
	// Matlab [t, State] = ode45(@ODEFunc, [0,T], odeStartState, [], Parameters);
	// https://stackoverflow.com/questions/26747492/comparison-of-odeints-runge-kutta4-with-matlabs-ode45
	// https://www.codeproject.com/Articles/268589/odeint-v2-Solving-ordinary-differential-equations
	solver_state_t odeStartState(4);

	if (direction == 1)
	{
		odeStartState[0] = start_x0.x;
		odeStartState[1] = start_x0.y;
		odeStartState[2] = start_x0.theta0;
		odeStartState[3] = start_x0.theta1;
	}
	else
	{
		odeStartState[0] = final_x0.x;
		odeStartState[1] = final_x0.y;
		odeStartState[2] = final_x0.theta0;
		odeStartState[3] = final_x0.theta1;
	}

	return (odeStartState);
}


static double
get_beta(double theta0, double theta1)
{
	return (carmen_normalize_theta(theta0 - theta1));
}


carmen_robot_and_trailers_traj_point_t
get_robot_and_trailer_traj_point_from_solver_state(solver_state_t x, kuglerParameters_t Parameters)
{
	double beta = get_beta(x[2], x[3]);
	carmen_robot_and_trailers_traj_point_t robot_and_trailer_traj_point =
	{
		x[0],	// x
		x[1],	// y
		x[2],	// theta
		1,
		{beta, 0.0, 0.0, 0.0, 0.0},
		1.0 * (double) Parameters.direction,	// v
		x[4]	// phi
	};

	return (robot_and_trailer_traj_point);
}


vector<carmen_robot_and_trailers_traj_point_t>
trailer_polynomial_analytical_expansion_new(carmen_robot_and_trailers_traj_point_t current_pose, carmen_robot_and_trailers_traj_point_t goal_pose,
		int direction, carmen_robot_ackerman_config_t robot_config, carmen_semi_trailers_config_t semi_trailer_config, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map,
		double max_phi_multiplier, bool test_limits)
{
	double T = 10.0;
	double dt = 0.1;

	double d0 = robot_config.distance_between_front_and_rear_axles;
	double d1 = semi_trailer_config.semi_trailers[0].d;

	carmen_robot_and_trailers_traj_point_t current_pose_in_relative_coordinates = change_pose_to_relative_coordinates(current_pose, current_pose);
	carmen_robot_and_trailers_traj_point_t goal_pose_in_relative_coordinates = change_pose_to_relative_coordinates(current_pose, goal_pose);

	kugler_state_t start_x0, start_x1, final_x0, final_x1;
	get_start_and_final_states(start_x0, start_x1, final_x0, final_x1, direction, current_pose_in_relative_coordinates, d1, goal_pose_in_relative_coordinates);

	kugler_coef_t coef = KuglerPathPlanner(start_x1, final_x1, d0, d1, semi_trailer_config.semi_trailers[0].M);

	double k0 = 0.0625;
	double k1 = 0.5;
	double k2 = 0.75;
	double k3 = 2.0;

	Parameters = {coef, d0, d1, T, start_x1.x, final_x1.x, k0, k1, k2, k3, direction};

	solver_state_t State = get_ode_start_state_new(direction, start_x0, final_x0);
	vector<carmen_robot_and_trailers_traj_point_t> trajectory;
	runge_kutta_dopri5<solver_state_t> stepper;
	for (double t = 0.0; t <= T; t += dt)
	{
		trajectory.push_back(get_robot_and_trailer_traj_point_from_solver_state(State, Parameters));
		if (isnan(trajectory[trajectory.size() - 1].x))
			return {}; // Identifica se as coordenadas são NaN, e já descarta o path para economizar custo computacional
		stepper.do_step(KuglerODEFunc_new, State, t, dt);
	}

	change_path_to_absolute_coordinates(trajectory, current_pose);

	return (trajectory);
}


double
trailer_analytical_expansion_cost_new(carmen_robot_and_trailers_traj_point_t current, carmen_robot_and_trailers_traj_point_t goal, int direction,
		carmen_robot_ackerman_config_t robot_config, carmen_semi_trailers_config_t semi_trailer_config)
{
	double path_cost = 0.0;

	// Incompleto
	// Forwards == 1; Backwards == -1
	vector<carmen_robot_and_trailers_traj_point_t> expanded_path;
	gsl_set_error_handler_off();

	double original_robot_max_phi = robot_config.max_phi;
	robot_config.max_phi = M_PI / 2.0;
	double original_semi_trailer_max_beta = semi_trailer_config.semi_trailers[0].max_beta;
	semi_trailer_config.semi_trailers[0].max_beta = M_PI;

	expanded_path = trailer_polynomial_analytical_expansion_new(current, goal, direction, robot_config, semi_trailer_config, NULL, max_phi_multiplier, false);

#ifdef PLOT_PATH
	plot_path(expanded_path);
#endif

	robot_config.max_phi = original_robot_max_phi;
	semi_trailer_config.semi_trailers[0].max_beta = original_semi_trailer_max_beta;

	if (expanded_path.size() < 1 || isnan(expanded_path[0].x))
		return (-1);

	for (size_t i = 1; i < expanded_path.size(); i++)
	{
		path_cost += DIST2D(expanded_path[i], expanded_path[i - 1]);
		if (isnan(path_cost))
			return (-1);

		double excess_phi = fabs(expanded_path[i - 1].phi) - original_robot_max_phi;
		if (excess_phi > 0.0)
			path_cost += 10.0 * excess_phi;
		double excess_beta = fabs(expanded_path[i - 1].trailer_theta[0]) - original_semi_trailer_max_beta;
		if (excess_beta > 0.0)
			path_cost += 10.0 * excess_beta;
	}

	return (path_cost);
}


double
calculate_continuous_theta(int z, int precomputed_cost_theta_size_trailer)
{
	// Apenas a inversao do metodo get_astar_map_theta
//	return ((double) ((z * 2.0 * M_PI) / (precomputed_cost_theta_size - 1)) - M_PI);
	double theta = (double) z * ((2.0 * M_PI) / (double) precomputed_cost_theta_size_trailer);

	return (carmen_normalize_theta(theta));
}


double
calculate_continuous_beta(int z, int beta_size)
{
	// beta_size tem que ser impar
	double beta;
	if (z <= (beta_size / 2))
		beta = ((double) z * ((M_PI / 2.0) / (double) (beta_size / 2)));	// beta maximo 90 graus
	else
		beta = M_PI + ((double) (z - 1) * ((M_PI / 2.0) / (double) (beta_size / 2)));	// beta maximo 90 graus

	return (carmen_normalize_theta(beta));
}


void
make_trailer_cost_map()
{
	int x_size = round(precomputed_cost_size_trailer / precomputed_cost_resolution_trailer);
	int y_size = round(precomputed_cost_size_trailer / precomputed_cost_resolution_trailer);
	printf("sizemap = %d %d \n", x_size, y_size);

	int i;
//#ifndef PLOT_PATH
//#pragma omp parallel for
//#endif
	for (int z = 0; z < precomputed_cost_theta_size_trailer; z++)
	{
		printf("current: z = %d\n", z);
		for (int a = 0; a < precomputed_cost_beta_size_trailer; a++) // beta do goal
		{
			printf("current: a = %d\n", a);
			for (int b = 0; b < precomputed_cost_beta_size_trailer; b++) // beta da origem
			{
				for (i = 0; i < x_size; i++)
				{
					for (int j = 0; j < y_size; j++)
					{
						carmen_robot_and_trailers_traj_point_t goal;
						goal.x = i * precomputed_cost_resolution_trailer;
						goal.y = j * precomputed_cost_resolution_trailer;
						goal.theta = calculate_continuous_theta(z, precomputed_cost_theta_size_trailer);
						goal.v = 0.0;
						goal.phi = 0.0;
						goal.trailer_theta[0] = calculate_continuous_beta(a, precomputed_cost_beta_size_trailer);
						carmen_robot_and_trailers_traj_point_t current;
						current.x = (double) precomputed_cost_size_trailer / 2.0;
						current.y = (double) precomputed_cost_size_trailer / 2.0;
						current.theta = 0.0;
						current.v = 0.0;
						current.phi = 0.0;
						current.trailer_theta[0] = calculate_continuous_beta(b, precomputed_cost_beta_size_trailer);
						carmen_robot_ackerman_config_t priv_robot_config = robot_config;
						carmen_semi_trailers_config_t priv_semi_trailer_config = semi_trailer_config;
//						double path_cost = trailer_analytical_expansion_cost(current, goal, 1, priv_robot_config, priv_semi_trailer_config);
//						cost_map_trailer[i][j][z][a][b]->h_forward = path_cost;
//#ifdef PLOT_PATH
//						printf("F %d %d %d %d %d - current = %f %f %f %f goal %f %f %f %f path cost = %f\n", i, j, z, a, b, current.x, current.y, current.theta, current.trailer_theta[0], goal.x, goal.y, goal.theta, goal.trailer_theta[0], path_cost);
//						getchar();
//#endif
						double path_cost = trailer_analytical_expansion_cost(current, goal, -1, priv_robot_config, priv_semi_trailer_config);
						cost_map_trailer[i][j][z][a][b]->h_backward = path_cost;
#ifdef PLOT_PATH
						printf("B %d %d %d %d %d - current = %f %f %f %f goal %f %f %f %f path cost = %f\n", i, j, z, a, b, current.x, current.y, current.theta, current.trailer_theta[0], goal.x, goal.y, goal.theta, goal.trailer_theta[0], path_cost);
						getchar();
#endif
//						printf("%d %d %d %d %d\n", i, j, z, a, b);
//						printf("current = %f %f %f %f goal %f %f %f %f path cost = %f\n", current.x, current.y, current.theta, current.trailer_theta[0], goal.x, goal.y, goal.theta, goal.trailer_theta[0], path_cost);
					}
				}
			}
		}
	}
}


int
save_trailer_cost_map()
{
	FILE *fp;
	int x_size = round(precomputed_cost_size_trailer / precomputed_cost_resolution_trailer);
	int y_size = round(precomputed_cost_size_trailer / precomputed_cost_resolution_trailer);

	fp = fopen(precomputed_cost_file_name_trailer, "w");
//	fp = fopen("caco.txt", "w");
	if (fp == NULL)
	{
		printf ("Houve um erro ao abrir o arquivo %s.\n", precomputed_cost_file_name_trailer);
		return (1);
	}

	for (int i = 0; i < x_size; i++)
	{
		for (int j = 0; j < y_size; j++)
		{
			for (int k = 0; k < precomputed_cost_theta_size_trailer; k++)
			{
				for (int a = 0; a < precomputed_cost_beta_size_trailer; a++) // beta do goal
				{
					for (int b = 0; b < precomputed_cost_beta_size_trailer; b++) // beta da origem
					{
						if (cost_map_trailer[i][j][k][a][b] != NULL)
							int result = fprintf(fp,"%d %d %d %d %d %lf %lf\n", i, j, k, a, b, cost_map_trailer[i][j][k][a][b]->h_forward, cost_map_trailer[i][j][k][a][b]->h_backward);
						else
							exit(printf("Erro na Gravacao do Cost Map %s!\n", precomputed_cost_file_name_trailer));
					}
				}
			}
		}
	}

	fclose (fp);

	return (0);
}


static void
carmen_get_parameters(int argc, char** argv)
{
	carmen_param_t param_list[] =
	{
		{(char *) "robot",				(char *) "length",								  		CARMEN_PARAM_DOUBLE, &robot_config.length,							 				1, NULL},
		{(char *) "robot",				(char *) "width",								  		CARMEN_PARAM_DOUBLE, &robot_config.width,								 			1, NULL},
		{(char *) "robot", 				(char *) "distance_between_rear_wheels",		  		CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_wheels,			 		1, NULL},
		{(char *) "robot", 				(char *) "distance_between_front_and_rear_axles", 		CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_and_rear_axles, 			1, NULL},
		{(char *) "robot", 				(char *) "distance_between_front_car_and_front_wheels",	CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_car_and_front_wheels,		1, NULL},
		{(char *) "robot", 				(char *) "distance_between_rear_car_and_rear_wheels",	CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_car_and_rear_wheels,		1, NULL},
		{(char *) "robot", 				(char *) "max_velocity",						  		CARMEN_PARAM_DOUBLE, &robot_config.max_v,									 		1, NULL},
		{(char *) "robot", 				(char *) "max_steering_angle",					  		CARMEN_PARAM_DOUBLE, &robot_config.max_phi,									 		1, NULL},
		{(char *) "robot", 				(char *) "maximum_acceleration_forward",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_forward,					1, NULL},
		{(char *) "robot", 				(char *) "maximum_acceleration_reverse",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_reverse,					1, NULL},
		{(char *) "robot", 				(char *) "maximum_deceleration_forward",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_deceleration_forward,					1, NULL},
		{(char *) "robot", 				(char *) "maximum_deceleration_reverse",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_deceleration_reverse,					1, NULL},
		{(char *) "robot", 				(char *) "maximum_steering_command_rate",				CARMEN_PARAM_DOUBLE, &robot_config.maximum_steering_command_rate,					1, NULL},
		{(char *) "robot", 				(char *) "understeer_coeficient",						CARMEN_PARAM_DOUBLE, &robot_config.understeer_coeficient,							1, NULL},
//		{(char *) "offroad",			(char *) "planner_precomputed_cost_size", 				CARMEN_PARAM_INT, 	 &precomputed_cost_size, 										1, NULL},
//		{(char *) "offroad",			(char *) "planner_precomputed_cost_theta_size", 		CARMEN_PARAM_INT, 	 &precomputed_cost_theta_size, 									1, NULL},
//		{(char *) "offroad",			(char *) "planner_precomputed_cost_resolution", 		CARMEN_PARAM_DOUBLE, &precomputed_cost_resolution,									1, NULL},
		{(char *) "obstacle_avoider",	(char *) "obstacles_safe_distance", 					CARMEN_PARAM_ONOFF,  &astar_config.oa_obstacles_safe_distance, 						1, NULL},
		{(char *) "offroad",			(char *) "planner_max_phi_multiplier", 					CARMEN_PARAM_DOUBLE, &max_phi_multiplier, 											1, NULL},
//		{(char *) "offroad",			(char *) "planner_precomputed_cost_file_name", 			CARMEN_PARAM_STRING, &precomputed_cost_file_name,									1, NULL},
		{(char *) "semi_trailer",	 	(char *) "initial_type",								CARMEN_PARAM_INT, 	 &(semi_trailer_config.num_semi_trailers), 					0, NULL},
	};

	precomputed_cost_size_trailer = 80.0; // Tamanho de 100 está consumindo muita memória
	precomputed_cost_resolution_trailer = 5.0; // Resolução de 0.2 faz com que o mapa fique extremamente grande
	precomputed_cost_theta_size_trailer = 10; // resolução de 36 graus
	precomputed_cost_beta_size_trailer = 5; // Tem que ser impar

	precomputed_cost_file_name_trailer = (char *) "cost_matrix_semi_trailer.data";
	int num_items = sizeof(param_list) / sizeof(param_list[0]);

	carmen_param_install_params(argc, argv, param_list, num_items);

	if (semi_trailer_config.num_semi_trailers > 0)
		carmen_task_manager_read_semi_trailer_parameters(&semi_trailer_config, argc, argv, semi_trailer_config.num_semi_trailers);
}


int
get_astar_map_beta(double beta, int map_beta_resolution)
{
	int int_beta;
	if (beta < 0.0)
		int_beta = (map_beta_resolution - 1) + floor(1.0 + (beta * (double) (map_beta_resolution / 2)) / (M_PI / 2.0));
	else
		int_beta = round((beta * (double) (map_beta_resolution / 2)) / (M_PI / 2.0));

	return (int_beta);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	carmen_get_parameters(argc, argv);

//	for (int z = 0; z < precomputed_cost_theta_size_trailer; z++)
//		printf("z %d, %lf\n", z, calculate_continuous_theta(z, precomputed_cost_theta_size_trailer));

//	for (int a = 0; a < precomputed_cost_beta_size_trailer; a++) // beta do goal
//	{
//		printf("a %d, beta %lf  ", a, calculate_continuous_beta(a, precomputed_cost_beta_size_trailer));
//		printf("beta %lf, a %d\n", calculate_continuous_beta(a, precomputed_cost_beta_size_trailer), get_astar_map_beta(calculate_continuous_beta(a, precomputed_cost_beta_size_trailer), precomputed_cost_beta_size_trailer));
//	}
//	return (0);

	alloc_trailer_cost_map();
	make_trailer_cost_map();
//	save_trailer_cost_map();
	clear_trailer_cost_map();

	return (0);
}
