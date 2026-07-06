#include <stdio.h>
#include "../nlp_mat_planner_path_planner_astar.h"
#include "../trailer_analytical_expansion.h"
#include <carmen/task_manager_interface.h>

#define 	NLP_EXPANSION
//#define 	PLOT_PATH

#define HUGE_COST 1000000.0


carmen_robot_ackerman_config_t robot_config;
carmen_semi_trailers_config_t semi_trailer_config;
nonholonomic_heuristic_cost_trailer_p *****cost_map_trailer;

double max_phi_multiplier;
carmen_path_planner_astar_t astar_config;


void
alloc_trailer_cost_map()
{
	int i, j, z, a, b;
	int x_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);
	int y_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);

	cost_map_trailer = (nonholonomic_heuristic_cost_trailer_p *****) calloc(x_size, sizeof(nonholonomic_heuristic_cost_trailer_p****));
	carmen_test_alloc(cost_map_trailer);

	for (i = 0; i < x_size; i++)
	{
		cost_map_trailer[i] = (nonholonomic_heuristic_cost_trailer_p ****) calloc(y_size, sizeof(nonholonomic_heuristic_cost_trailer_p***));
		carmen_test_alloc(cost_map_trailer[i]);

		for (j = 0; j < y_size; j++)
		{
			cost_map_trailer[i][j] = (nonholonomic_heuristic_cost_trailer_p***) calloc(astar_config.precomputed_cost_trailer_theta_size, sizeof(nonholonomic_heuristic_cost_trailer_p**));
			carmen_test_alloc(cost_map_trailer[i][j]);

			for (z = 0; z < astar_config.precomputed_cost_trailer_theta_size; z++)
			{
				cost_map_trailer[i][j][z]= (nonholonomic_heuristic_cost_trailer_p**) calloc(astar_config.precomputed_cost_trailer_beta_size, sizeof(nonholonomic_heuristic_cost_trailer_p*));
				carmen_test_alloc(cost_map_trailer[i][j][z]);

				for (a = 0; a < astar_config.precomputed_cost_trailer_beta_size; a++) // beta do goal
				{
					cost_map_trailer[i][j][z][a]= (nonholonomic_heuristic_cost_trailer_p*) calloc(astar_config.precomputed_cost_trailer_beta_size, sizeof(nonholonomic_heuristic_cost_trailer_p));
					carmen_test_alloc(cost_map_trailer[i][j][z][a]);

					for (b = 0; b < astar_config.precomputed_cost_trailer_beta_size; b++) // beta da origem
					{
						cost_map_trailer[i][j][z][a][b] = (nonholonomic_heuristic_cost_trailer_p) malloc(sizeof(nonholonomic_heuristic_cost_trailer));
						carmen_test_alloc(cost_map_trailer[i][j][z][a][b]);
						cost_map_trailer[i][j][z][a][b]->h_forward_valid = false;
						cost_map_trailer[i][j][z][a][b]->h_backward_valid = false;
					}
				}
			}
		}
	}
}


void
free_trailer_cost_map()
{
	int x_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);
	int y_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);

	for (int i = 0; i < x_size; i++)
		for (int j = 0; j < y_size; j++)
			for (int z = 0; z < astar_config.precomputed_cost_trailer_theta_size; z++)
				for (int a = 0; a < astar_config.precomputed_cost_trailer_beta_size; a++)
					for (int b = 0; b < astar_config.precomputed_cost_trailer_beta_size; b++)
						free(cost_map_trailer[i][j][z][a][b]);
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
		fprintf(gnuplot_pipeMP, "set xrange [%d:%d]\n", (int) ((double) (-astar_config.precomputed_cost_trailer_size * 0.1 - semi_trailer_config.semi_trailers[0].d)), (int) ((double) (astar_config.precomputed_cost_trailer_size + semi_trailer_config.semi_trailers[0].d) * 1.1));
		fprintf(gnuplot_pipeMP, "set yrange [%d:%d]\n", (int) ((double) (-astar_config.precomputed_cost_trailer_size * 0.1 - semi_trailer_config.semi_trailers[0].d)), (int) ((double) (astar_config.precomputed_cost_trailer_size + semi_trailer_config.semi_trailers[0].d) * 1.1));
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
	gsl_set_error_handler_off();

//	double original_robot_max_phi = robot_config.max_phi;
//	robot_config.max_phi = original_robot_max_phi * 1.5;
//	double original_semi_trailer_max_beta = semi_trailer_config.max_beta;
//	semi_trailer_config.max_beta = original_semi_trailer_max_beta * 1.5;

#ifdef	NLP_EXPANSION
	vector<carmen_robot_and_trailers_traj_point_t> expanded_path = trailer_nlp_analytical_expansion(current, goal,
			robot_config, semi_trailer_config, NULL, max_phi_multiplier, true, true, 1);
#else
	vector<carmen_robot_and_trailers_traj_point_t> expanded_path = trailer_polynomial_analytical_expansion(current, goal,
			direction, robot_config, semi_trailer_config, NULL, max_phi_multiplier, true);
#endif

#ifdef PLOT_PATH
	plot_path(expanded_path);
#endif

//	robot_config.max_phi = original_robot_max_phi;
//	semi_trailer_config.max_beta = original_semi_trailer_max_beta;

	if (expanded_path.size() < 1 || isnan(expanded_path[0].x))
		return (-1);

	double path_cost = 0.0;
	for (size_t i = 1; i < expanded_path.size(); i++)
	{
		path_cost += DIST2D(expanded_path[i], expanded_path[i - 1]);
		if (isnan(path_cost))
			return (-1);

//		double excess_phi = fabs(expanded_path[i - 1].phi) - original_robot_max_phi;
//		if (excess_phi > 0.0)
//			path_cost += 10.0 * excess_phi;
//		double excess_beta = fabs(expanded_path[i - 1].trailer_theta[0]) - original_semi_trailer_max_beta;
//		if (excess_beta > 0.0)
//			path_cost += 10.0 * excess_beta;
	}

	return (path_cost);
}


double
calculate_continuous_theta(int z, int precomputed_cost_theta_size_trailer)
{
	// precomputed_cost_theta_size_trailer tem que ser impar. theta máximo 180/2 graus.
	double theta = (double) z * ((2.0 * M_PI) / (double) precomputed_cost_theta_size_trailer);
	theta = carmen_normalize_theta(theta) / 2.0;

	return (theta);
}


double
calculate_continuous_beta(int z, int beta_size)
{
	// beta_size tem que ser impar. beta máximo 180/8 graus.
	double beta = (double) z * ((2.0 * M_PI) / (double) beta_size);
	beta = carmen_normalize_theta(beta) / 8.0;

	return (beta);
}


void
make_trailer_cost_map()
{
	int x_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);
	int y_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);
	double delta_x = astar_config.precomputed_cost_trailer_size / (double) x_size;
	double delta_y = astar_config.precomputed_cost_trailer_size / (double) y_size;
	printf("sizemap = %d %d \n", x_size, y_size);

	int i;
#ifndef PLOT_PATH
#pragma omp parallel for default(none) shared (cost_map_trailer) firstprivate (astar_config, x_size, y_size, delta_x, delta_y, robot_config, semi_trailer_config)
#endif
	for (i = 0; i < x_size; i++)
	{
		printf("* current: i = %d\n", i);
		for (int j = 0; j < y_size; j++)
		{
			printf("  current: j = %d\n", j);
			for (int z = 0; z < astar_config.precomputed_cost_trailer_theta_size; z++)
			{
				for (int a = 0; a < astar_config.precomputed_cost_trailer_beta_size; a++) // beta do goal
				{
					for (int b = 0; b < astar_config.precomputed_cost_trailer_beta_size; b++) // beta da origem
					{
						carmen_robot_and_trailers_traj_point_t goal;
						goal.x = (double) i * delta_x;
						goal.y = (double) j * delta_y;
						goal.theta = calculate_continuous_theta(z, astar_config.precomputed_cost_trailer_theta_size);
						goal.trailer_theta[0] = calculate_continuous_beta(a, astar_config.precomputed_cost_trailer_beta_size);
						goal.v = 0.0;
						goal.phi = 0.0;

						carmen_robot_and_trailers_traj_point_t current;
						current.x = (double) astar_config.precomputed_cost_trailer_size / 2.0;
						current.y = (double) astar_config.precomputed_cost_trailer_size / 2.0;
						current.theta = 0.0;
						current.v = 0.0;
						current.phi = 0.0;
						current.trailer_theta[0] = calculate_continuous_beta(b, astar_config.precomputed_cost_trailer_beta_size);

						carmen_robot_ackerman_config_t priv_robot_config = robot_config;
						carmen_semi_trailers_config_t priv_semi_trailer_config = semi_trailer_config;
#ifdef	NLP_EXPANSION
						double path_cost = trailer_analytical_expansion_cost(current, goal, 1, priv_robot_config, priv_semi_trailer_config);
						cost_map_trailer[i][j][z][a][b]->h_forward = path_cost;
						if (cost_map_trailer[i][j][z][a][b]->h_forward > 0.0)
							cost_map_trailer[i][j][z][a][b]->h_forward_valid = true;
						cost_map_trailer[i][j][z][a][b]->h_backward = path_cost;
						if (cost_map_trailer[i][j][z][a][b]->h_backward > 0.0)
							cost_map_trailer[i][j][z][a][b]->h_backward_valid = true;
#ifdef PLOT_PATH
						printf("F %d %d %d %d %d - current = %f %f %5.2f %5.2f goal %f %f %5.2f %5.2f path cost = %f\n", i, j, z, a, b,
								current.x, current.y, carmen_radians_to_degrees(current.theta), carmen_radians_to_degrees(current.trailer_theta[0]),
								goal.x, goal.y, carmen_radians_to_degrees(goal.theta), carmen_radians_to_degrees(goal.trailer_theta[0]), path_cost);
						if ((path_cost > 0.0) && (path_cost < 1000.0))
							getchar();
#endif
#else
						double path_cost = trailer_analytical_expansion_cost(current, goal, 1, priv_robot_config, priv_semi_trailer_config);
						cost_map_trailer[i][j][z][a][b]->h_forward = path_cost;
						if (cost_map_trailer[i][j][z][a][b]->h_forward > 0.0)
							cost_map_trailer[i][j][z][a][b]->h_forward_valid = true;
#ifdef PLOT_PATH
						printf("F %d %d %d %d %d - current = %f %f %5.2f %5.2f goal %f %f %5.2f %5.2f path cost = %f\n", i, j, z, a, b,
								current.x, current.y, carmen_radians_to_degrees(current.theta), carmen_radians_to_degrees(current.trailer_theta[0]),
								goal.x, goal.y, carmen_radians_to_degrees(goal.theta), carmen_radians_to_degrees(goal.trailer_theta[0]), path_cost);
						if ((path_cost > 0.0) && (path_cost < 1000.0))
							getchar();
#endif
						path_cost = trailer_analytical_expansion_cost(current, goal, -1, priv_robot_config, priv_semi_trailer_config);
						cost_map_trailer[i][j][z][a][b]->h_backward = path_cost;
						if (cost_map_trailer[i][j][z][a][b]->h_backward > 0.0)
							cost_map_trailer[i][j][z][a][b]->h_backward_valid = true;
#ifdef PLOT_PATH
						printf("B %d %d %d %d %d - current = %f %f %5.2f %5.2f goal %f %f %5.2f %5.2f path cost = %f\n", i, j, z, a, b,
								current.x, current.y, carmen_radians_to_degrees(current.theta), carmen_radians_to_degrees(current.trailer_theta[0]),
								goal.x, goal.y, carmen_radians_to_degrees(goal.theta), carmen_radians_to_degrees(goal.trailer_theta[0]), path_cost);
						if ((path_cost > 0.0) && (path_cost < 1000.0))
							getchar();
#endif
#endif
//						printf("%d %d %d %d %d\n", i, j, z, a, b);
//						printf("current = %f %f %f %f goal %f %f %f %f path cost = %f\n", current.x, current.y, current.theta, current.trailer_theta[0], goal.x, goal.y, goal.theta, goal.trailer_theta[0], path_cost);
					}
				}
			}
		}
	}
}


inline void
compute_intermediate_pixel_distance(int x, int y, int z, int a, int b)
{
	for (int i = -1; i < 2; i++)
	{
		for (int j = -1; j < 2; j++)
		{
			double v_b = cost_map_trailer[x + i][y + j][z][a][b]->h_backward;
			if (v_b != -1.0)
				v_b = v_b + ((i * j != 0) ? 1.414213562 : 1.0);

			double v_f = cost_map_trailer[x + i][y + j][z][a][b]->h_forward;
			if (v_f != -1.0)
				v_f = v_f + ((i * j != 0) ? 1.414213562 : 1.0);

			double v;
			if ((v_b != -1.0) && (v_f != -1.0))
			{
				if (v_b < v_f)
					v = v_b;
				else
					v = v_f;
			}
			else if (v_b != -1.0)
				v = v_b;
			else
				v = v_f;

			double h_backward = cost_map_trailer[x][y][z][a][b]->h_backward;
			double h_forward = cost_map_trailer[x][y][z][a][b]->h_forward;
			if ((h_backward != -1.0) && (v < h_backward))
				cost_map_trailer[x][y][z][a][b]->h_backward = v;
			if ((h_forward != -1.0) && (v < h_forward))
				cost_map_trailer[x][y][z][a][b]->h_forward = v;
		}
	}
}


void
initialize_missing_cells_in_the_cost_map()
{
	// Ou h_backward e h_forward ficam iguais a HUG_COST,
	// ou um deles fica igual a -1.0 e o outro com o menor custo dos dois.
	int x_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);
	int y_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);

	for (int i = 0; i < x_size; i++)
	{
		for (int j = 0; j < y_size; j++)
		{
			for (int z = 0; z < astar_config.precomputed_cost_trailer_theta_size; z++)
			{
				for (int a = 0; a < astar_config.precomputed_cost_trailer_beta_size; a++)
				{
					for (int b = 0; b < astar_config.precomputed_cost_trailer_beta_size; b++)
					{
						if ((cost_map_trailer[i][j][z][a][b]->h_backward == -1.0) && (cost_map_trailer[i][j][z][a][b]->h_forward == -1.0))
							cost_map_trailer[i][j][z][a][b]->h_backward = cost_map_trailer[i][j][z][a][b]->h_forward = HUGE_COST;
//						else if ((cost_map_trailer[i][j][z][a][b]->h_backward != -1.0) && (cost_map_trailer[i][j][z][a][b]->h_forward != -1.0))
//						{
//							if (cost_map_trailer[i][j][z][a][b]->h_backward >= (cost_map_trailer[i][j][z][a][b]->h_forward - 0.1))
//							{
//								cost_map_trailer[i][j][z][a][b]->h_backward = -1.0;
//								cost_map_trailer[i][j][z][a][b]->h_backward_valid = false;
//							}
//							else
//							{
//								cost_map_trailer[i][j][z][a][b]->h_forward = -1.0;
//								cost_map_trailer[i][j][z][a][b]->h_forward_valid = false;
//							}
//						}
					}
				}
			}
		}
	}
}


void
fill_in_missing_cells_in_the_cost_map()
{
	// Use dynamic programming to estimate the minimum distance from
	// every map cell to an occupied map cell
	// Note that the first and last lines, and the first and last columns are not computed. But the robot never goes there... So, no problem :)

	int x_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);
	int y_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);

	for (int z = 0; z < astar_config.precomputed_cost_trailer_theta_size; z++)
	{
		for (int a = 0; a < astar_config.precomputed_cost_trailer_beta_size; a++)
		{
			for (int b = 0; b < astar_config.precomputed_cost_trailer_beta_size; b++)
			{
				// pass 1
				for (int i = 1; i < x_size - 1; i++)
					for (int j = 1; j < y_size - 1; j++)
						compute_intermediate_pixel_distance(i, j, z, a, b);

				// pass 2
				for (int i = x_size - 2; i >= 1; i--)
					for (int j = y_size - 2; j >= 1; j--)
						compute_intermediate_pixel_distance(i, j, z, a, b);
			}
		}
	}
}


static int
save_trailer_cost_map(int just_a_test)
{
	FILE *fp;
	int x_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);
	int y_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);

	fp = fopen(astar_config.precomputed_cost_trailer_file_name, "w");
//	fp = fopen("caco.txt", "w");
	if (fp == NULL)
		exit(printf ("Houve um erro ao abrir o arquivo %s.\n", astar_config.precomputed_cost_trailer_file_name));

	for (int i = 0; i < x_size; i++)
	{
		for (int j = 0; j < y_size; j++)
		{
			for (int k = 0; k < astar_config.precomputed_cost_trailer_theta_size; k++)
			{
				for (int a = 0; a < astar_config.precomputed_cost_trailer_beta_size; a++) // beta do goal
				{
					for (int b = 0; b < astar_config.precomputed_cost_trailer_beta_size; b++) // beta da origem
					{
						if (cost_map_trailer[i][j][k][a][b] != NULL)
							fprintf(fp,"%d %d %d %d %d %lf %lf %d %d\n", i, j, k, a, b,
									cost_map_trailer[i][j][k][a][b]->h_forward, cost_map_trailer[i][j][k][a][b]->h_backward,
									(cost_map_trailer[i][j][k][a][b]->h_forward_valid)? 1: 0,
									(cost_map_trailer[i][j][k][a][b]->h_backward_valid)? 1: 0);
						else
							exit(printf("Erro na gravacao de uma linha do Cost Map %s!\n", astar_config.precomputed_cost_trailer_file_name));
					}
				}
			}
		}
	}

	if (fclose(fp) == 0)
	{
		if (just_a_test == 1)
			printf("Teste de gravacao do Cost Map realizado com sucesso! Arquivo: %s\n", astar_config.precomputed_cost_trailer_file_name);
		else
			printf("Gravacao do Cost Map realizada com sucesso! Arquivo: %s\n", astar_config.precomputed_cost_trailer_file_name);
	}
	else
		printf("Erro no fechamento do aquivo Cost Map %s!\n", astar_config.precomputed_cost_trailer_file_name);

	return (0);
}


int
open_trailer_cost_map()
{
	FILE *fp;
	int x_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);
	int y_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);
	printf("Carregando mapa de heurística sem obstáculos para o robô mais semi-trailer. Arquivo: %s\n", astar_config.precomputed_cost_trailer_file_name);
	fp = fopen (astar_config.precomputed_cost_trailer_file_name, "r");
	if (fp == NULL)
	{
		printf ("Houve um erro ao abrir o mapa para robô e trailer...\n");
		printf("Verifique se o arquivo %s existe.\n", (char *) "temp_offroad_planner_trailer_cost_matrix.txt");
		return (1);
	}

	for (int i = 0; i < x_size; i++)
	{
		for (int j = 0; j < y_size; j++)
		{
			for (int k = 0; k < astar_config.precomputed_cost_trailer_theta_size; k++)
			{
				for (int a = 0; a < astar_config.precomputed_cost_trailer_beta_size; a++) // beta do goal
				{
					for (int b = 0; b < astar_config.precomputed_cost_trailer_beta_size; b++) // beta da origem
					{
						int d, e, f, g, h, h_forward_valid, h_backward_valid;
						int result = fscanf(fp, "%d %d %d %d %d %lf %lf %d %d\n", &d, &e, &f, &g, &h,
								&(cost_map_trailer[i][j][k][a][b]->h_forward), &(cost_map_trailer[i][j][k][a][b]->h_backward),
								&h_forward_valid, &h_backward_valid);
						cost_map_trailer[i][j][k][a][b]->h_forward_valid = h_forward_valid;
						cost_map_trailer[i][j][k][a][b]->h_backward_valid = h_backward_valid;
						if (result != 9)
						{
							printf("Erro in open_trailer_cost_map()\n");
							exit (1);
						}
					}
				}
			}
		}
	}

	fclose (fp);
	printf("Mapa da heurística sem obstáculos para TRAILER carregado!\n");

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
		{(char *) "offroad",			(char *) "planner_precomputed_cost_trailer_size", CARMEN_PARAM_DOUBLE, &astar_config.precomputed_cost_trailer_size, 1, NULL},
		{(char *) "offroad",			(char *) "planner_precomputed_cost_trailer_theta_size", CARMEN_PARAM_INT, &astar_config.precomputed_cost_trailer_theta_size, 1, NULL},
		{(char *) "offroad",			(char *) "planner_precomputed_cost_trailer_beta_size", CARMEN_PARAM_INT, &astar_config.precomputed_cost_trailer_beta_size, 1, NULL},
		{(char *) "offroad",			(char *) "planner_precomputed_cost_trailer_resolution", CARMEN_PARAM_DOUBLE, &astar_config.precomputed_cost_trailer_resolution, 1, NULL},
		{(char *) "offroad",			(char *) "planner_precomputed_cost_trailer_file_name", CARMEN_PARAM_STRING, &astar_config.precomputed_cost_trailer_file_name, 1, NULL},
		{(char *) "offroad",			(char *) "planner_use_matrix_cost_trailer_heuristic", CARMEN_PARAM_ONOFF, &astar_config.use_matrix_cost_trailer_heuristic, 1, NULL},
	};

//	astar_config.precomputed_cost_trailer_size = 81.0; // Tamanho de 100 está consumindo muita memória
//	astar_config.precomputed_cost_trailer_resolution = 1.0; // Resolução de 0.2 faz com que o mapa fique extremamente grande
//	astar_config.precomputed_cost_trailer_theta_size = 10; // resolução de 36 graus
//	precomputed_cost_beta_size_trailer = 5; // Tem que ser impar

	astar_config.precomputed_cost_trailer_file_name = (char *) "cost_matrix_semi_trailer.data";
	int num_items = sizeof(param_list) / sizeof(param_list[0]);

	carmen_param_install_params(argc, argv, param_list, num_items);

	if (semi_trailer_config.num_semi_trailers > 0)
		carmen_task_manager_read_semi_trailer_parameters(&semi_trailer_config, argc, argv, semi_trailer_config.num_semi_trailers);
}


// Esta funcao tem que ser igual a de mesmo nome em offroad_planner_path_planner_astar.cpp
int
get_astar_map_beta(double beta, int map_beta_resolution)
{
	int z;

	beta = carmen_normalize_theta(beta);

	if (beta >= 0.0)
	{
		z = round((beta * (double) (map_beta_resolution - 1)) / (0.25 * M_PI));
		if (z > map_beta_resolution / 2)
			z = map_beta_resolution / 2;
	}
	else
	{
		beta = -beta;
		z = round((beta * (double) (map_beta_resolution - 1)) / (0.25 * M_PI));
		if (z > map_beta_resolution / 2)
			z = map_beta_resolution / 2;

		z = map_beta_resolution - z;

		if (z > map_beta_resolution - 1)
			z = 0;
	}

	return (z);
}


// Esta funcao tem que ser igual a de mesmo nome em offroad_planner_path_planner_astar.cpp
int
get_cost_map_int_theta(double theta, int precomputed_cost_theta_size)
{
	int z;

	theta = carmen_normalize_theta(theta);

	if (theta >= 0.0)
	{
		z = round((theta * (double) (precomputed_cost_theta_size - 1)) / (1.0 * M_PI));
		if (z > precomputed_cost_theta_size / 2)
			z = precomputed_cost_theta_size / 2;
	}
	else
	{
		theta = -theta;
		z = round((theta * (double) (precomputed_cost_theta_size - 1)) / (1.0 * M_PI));
		if (z > precomputed_cost_theta_size / 2)
			z = precomputed_cost_theta_size / 2;

		z = precomputed_cost_theta_size - z;

		if (z > precomputed_cost_theta_size - 1)
			z = 0;
	}

	return (z);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	carmen_get_parameters(argc, argv);

	double t0 = carmen_get_time();
//	for (int z = 0; z < astar_config.precomputed_cost_trailer_theta_size; z++)
//	{
//		printf("z %d, theta %lf  ", z, carmen_radians_to_degrees(calculate_continuous_theta(z, astar_config.precomputed_cost_trailer_theta_size)));
//		printf("theta %lf, z %d\n", carmen_radians_to_degrees(calculate_continuous_theta(z, astar_config.precomputed_cost_trailer_theta_size)),
//				get_cost_map_int_theta(calculate_continuous_theta(z, astar_config.precomputed_cost_trailer_theta_size), astar_config.precomputed_cost_trailer_theta_size));
//	}
//
//	for (double z = 0.0; z <= 360.0; z += 1.0)
//	{
//		double z_normalized = carmen_radians_to_degrees(carmen_normalize_theta(carmen_degrees_to_radians(z)));
//		printf("theta %lf, z %d\n", z_normalized, get_cost_map_int_theta(carmen_degrees_to_radians(z), astar_config.precomputed_cost_trailer_theta_size));
//	}
//
//	for (int a = 0; a < astar_config.precomputed_cost_trailer_beta_size; a++) // beta do goal
//	{
//		printf("a %d, beta %lf  ", a, carmen_radians_to_degrees(calculate_continuous_beta(a, astar_config.precomputed_cost_trailer_beta_size)));
//		printf("beta %lf, a %d\n", carmen_radians_to_degrees(calculate_continuous_beta(a, astar_config.precomputed_cost_trailer_beta_size)),
//				get_astar_map_beta(calculate_continuous_beta(a, astar_config.precomputed_cost_trailer_beta_size), astar_config.precomputed_cost_trailer_beta_size));
//	}
//
//	for (double a = 0.0; a <= 360.0; a += 1.0)
//	{
//		double a_normalized = carmen_radians_to_degrees(carmen_normalize_theta(carmen_degrees_to_radians(a)));
//		printf("beta %lf, a %d\n", a_normalized, get_astar_map_beta(carmen_degrees_to_radians(a), astar_config.precomputed_cost_trailer_beta_size));
//	}
//	return (0);

	alloc_trailer_cost_map();
	// A linha abaixo eh soh para testar se consegue salvar o arquivo
	save_trailer_cost_map(1);
//	open_trailer_cost_map();
	make_trailer_cost_map();
	initialize_missing_cells_in_the_cost_map();
	fill_in_missing_cells_in_the_cost_map();
	save_trailer_cost_map(0);
	free_trailer_cost_map();

	double total_time = carmen_get_time() - t0;
	printf("Total time %.2lfs, %.2lfm, %.2lfh, %.2lfd\n", total_time, total_time / 60.0, total_time / (60.0 * 60.0), total_time / (60.0 * 60.0 * 24.0));

	return (0);
}
