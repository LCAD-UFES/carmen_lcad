#include <stdio.h>
#include "../nlp_mat_planner_path_planner_astar.h"
#include <carmen/task_manager_interface.h>

#define 	NLP_EXPANSION
//#define 	PLOT_PATH

#define HUGE_COST 1000000.0
char backup_file[2000];
carmen_robot_ackerman_config_t robot_config;
carmen_semi_trailers_config_t semi_trailer_config;
nonholonomic_heuristic_cost_trailer_p *****cost_map_trailer;

double max_phi_multiplier;
carmen_path_planner_astar_t astar_config;

typedef struct job_params
{
	int i;
	int j;
	int z;
	int a;
	int b;
	double path_cost;
} job_params;

void
load_current_trailer_cost_map()
{
	FILE *fp;
	char * line = NULL;
	size_t len = 0;
	ssize_t read;
	int x_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);
	int y_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);
	printf("load_current_trailer_cost_map. Arquivo: %s\n", astar_config.precomputed_cost_trailer_file_name);
	fp = fopen (astar_config.precomputed_cost_trailer_file_name, "r");
	if (fp == NULL)
	{
		printf ("Arquivo %s não existe. Iniciando do zero.\n", astar_config.precomputed_cost_trailer_file_name);
//		exit(1);
		return;
	}

	//Um backup no tmp para garantir que os valores da tabela não serão perdidos, caso aconteça algum problema
	char cmd[2000];
	sprintf(backup_file, "/tmp/backup-%s-%f", basename(astar_config.precomputed_cost_trailer_file_name), carmen_get_time());
	sprintf(cmd, "cp -r %s %s", astar_config.precomputed_cost_trailer_file_name, backup_file);
	system(cmd);

	while ((read = getline(&line, &len, fp)) != -1)
	{
		int d, e, f, g, h , h_forward_valid, h_backward_valid;
		double h_forward, h_backward;
		int result = sscanf(line, "%d %d %d %d %d %lf %lf %d %d\n", &d, &e, &f, &g, &h,
				&h_forward, &h_backward,
				&h_forward_valid, &h_backward_valid);

		//Verifica o result antes de atribuir os valores, pois pode acontecer situações onde a linha da tabela está incompleta
		if (result != 9)
			break;

		if (h_forward != -2.0 && h_backward != -2.0)
		{
			cost_map_trailer[d][e][f][g][h]->h_forward = h_forward;
			cost_map_trailer[d][e][f][g][h]->h_backward = h_backward;
			cost_map_trailer[d][e][f][g][h]->h_forward_valid = h_forward_valid;
			cost_map_trailer[d][e][f][g][h]->h_backward_valid = h_backward_valid;
		}
	}
	fclose (fp);

	if (line)
		free(line);
}


void
clear_trailer_cost_map()
{
	int i, j, z, a, b;
	int x_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);
	int y_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);


	for (i = 0; i < x_size; i++)
	{
		for (j = 0; j < y_size; j++)
		{
			for (z = 0; z < astar_config.precomputed_cost_trailer_theta_size; z++)
			{
				for (a = 0; a < astar_config.precomputed_cost_trailer_beta_size; a++) // beta do goal
				{
					for (b = 0; b < astar_config.precomputed_cost_trailer_beta_size; b++) // beta da origem
					{
						cost_map_trailer[i][j][z][a][b]->h_forward = -2.0;
						cost_map_trailer[i][j][z][a][b]->h_backward = -2.0;
						cost_map_trailer[i][j][z][a][b]->h_forward_valid = false;
						cost_map_trailer[i][j][z][a][b]->h_backward_valid = false;
					}
				}
			}
		}
	}
}

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
						cost_map_trailer[i][j][z][a][b]->indice = -1;
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
trailer_analytical_expansion_cost(std::vector<carmen_robot_and_trailers_traj_point_t> &valid_path, carmen_robot_and_trailers_traj_point_t current, carmen_robot_and_trailers_traj_point_t goal, int direction,
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

	for (size_t i = 0; i < expanded_path.size(); i++)
		valid_path.push_back(expanded_path[i]);


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
						std::vector<carmen_robot_and_trailers_traj_point_t> path_valid;
						double path_cost = trailer_analytical_expansion_cost(path_valid, current, goal, 1, priv_robot_config, priv_semi_trailer_config);

						cost_map_trailer[i][j][z][a][b]->h_forward = path_cost;
						if (cost_map_trailer[i][j][z][a][b]->h_forward > 0.0)
							cost_map_trailer[i][j][z][a][b]->h_forward_valid = true;
						cost_map_trailer[i][j][z][a][b]->h_backward = path_cost;
						if (cost_map_trailer[i][j][z][a][b]->h_backward > 0.0)
							cost_map_trailer[i][j][z][a][b]->h_backward_valid = true;

						if (path_cost > 0.0)
						{
							carmen_robot_and_trailers_traj_point_t *alloc_path = NULL;
							alloc_path = (carmen_robot_and_trailers_traj_point_t *) calloc(80, sizeof(carmen_robot_and_trailers_traj_point_t));
							do {if ((void *)(alloc_path) == NULL) exit(printf("Out of memory in (%s, line %d).\n", __FILE__, __LINE__)); } while (0); //macro do carmen_test_alloc colocado para não dar problema no pragma

							for (int ii = 0; ii < 80; ii++)
							{
								alloc_path[ii].x = path_valid[ii].x;
								alloc_path[ii].y = path_valid[ii].y;
								alloc_path[ii].theta = path_valid[ii].theta;
								alloc_path[ii].trailer_theta[0] = path_valid[ii].trailer_theta[0];
								alloc_path[ii].v = path_valid[ii].v;
								alloc_path[ii].phi = path_valid[ii].phi;
							}
							cost_map_trailer[i][j][z][a][b]->path = &alloc_path[0];
						}

#ifdef PLOT_PATH
						printf("F %d %d %d %d %d - current = %f %f %5.2f %5.2f goal %f %f %5.2f %5.2f path cost = %f\n", i, j, z, a, b,
								current.x, current.y, carmen_radians_to_degrees(current.theta), carmen_radians_to_degrees(current.trailer_theta[0]),
								goal.x, goal.y, carmen_radians_to_degrees(goal.theta), carmen_radians_to_degrees(goal.trailer_theta[0]), path_cost);
						if ((path_cost > 0.0) && (path_cost < 1000.0))
							getchar();
#endif
#else
						double path_cost = trailer_analytical_expansion_cost(path_valid, current, goal, 1, priv_robot_config, priv_semi_trailer_config);
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
						path_cost = trailer_analytical_expansion_cost(path_valid, current, goal, -1, priv_robot_config, priv_semi_trailer_config);
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
						if ((cost_map_trailer[i][j][z][a][b]->h_backward <= -1.0) && (cost_map_trailer[i][j][z][a][b]->h_forward <= -1.0))
							cost_map_trailer[i][j][z][a][b]->h_backward = cost_map_trailer[i][j][z][a][b]->h_forward = HUGE_COST;
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
	FILE *fp, *paths_file;

	fp = fopen(astar_config.precomputed_cost_trailer_file_name, "w");
//	fp = fopen("caco.txt", "w");

	if (fp == NULL)
		exit(printf ("Houve um erro ao abrir o arquivo %s.\n", astar_config.precomputed_cost_trailer_file_name));

	int paths_indice = 0;
	char binary_file_type[1024];
	sprintf(binary_file_type, "%s_paths", astar_config.precomputed_cost_trailer_file_name);
	if(just_a_test == 2)
	{
		paths_file = fopen(binary_file_type, "wb");
		if (paths_file == NULL)
			exit(printf ("Houve um erro ao abrir o arquivo %s.\n", binary_file_type));
	}
	// Parâmetros do mapa na primeira linha
	fprintf(fp, "%lf %d %d %lf\n", astar_config.precomputed_cost_trailer_size, astar_config.precomputed_cost_trailer_theta_size, astar_config.precomputed_cost_trailer_beta_size, astar_config.precomputed_cost_trailer_resolution);

	int x_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);
	int y_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);

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
						{
							if (just_a_test == 2 && cost_map_trailer[i][j][k][a][b]->h_forward_valid) // Se essa condição for falsa, o indice é gravado no arquivo como -1
								cost_map_trailer[i][j][k][a][b]->indice = paths_indice;

							fprintf(fp,"%d %d %d %d %d %lf %lf %d %d\n", i, j, k, a, b,
									cost_map_trailer[i][j][k][a][b]->h_forward, cost_map_trailer[i][j][k][a][b]->h_backward,
									(cost_map_trailer[i][j][k][a][b]->h_forward_valid)? 1: 0,
									(cost_map_trailer[i][j][k][a][b]->h_backward_valid)? 1: 0);//,
//									 cost_map_trailer[i][j][k][a][b]->indice);

							if (just_a_test == 2 && cost_map_trailer[i][j][k][a][b]->h_forward_valid)
							{
								// Parte que adiciona o path no arquivo binário
//								for (int ii = 0; ii < 80; ii++)
//									fprintf(paths_file, "%f%f%f%f%f%f", cost_map_trailer[i][j][k][a][b]->path[ii].x, cost_map_trailer[i][j][k][a][b]->path[ii].y, cost_map_trailer[i][j][k][a][b]->path[ii].theta, cost_map_trailer[i][j][k][a][b]->path[ii].trailer_theta[0], cost_map_trailer[i][j][k][a][b]->path[ii].v, cost_map_trailer[i][j][k][a][b]->path[ii].phi);

								fwrite(cost_map_trailer[i][j][k][a][b]->path , sizeof(carmen_robot_and_trailers_traj_point_t), 80, paths_file);
								free(cost_map_trailer[i][j][k][a][b]->path);
								paths_indice++;
									//
							}
//							if (cost_map_trailer[i][j][k][a][b]->h_forward_valid)
//							{
//								printf("Testando acesso ao path allocado fora do for:\n");
//								for (int ii = 0; ii < 80; ii++)
//								{
//									printf("Path[%d]: %f %f %f %f %f %f\n", ii,  cost_map_trailer[i][j][k][a][b]->path[ii].x, cost_map_trailer[i][j][k][a][b]->path[ii].y, cost_map_trailer[i][j][k][a][b]->path[ii].theta, cost_map_trailer[i][j][k][a][b]->path[ii].trailer_theta[0], cost_map_trailer[i][j][k][a][b]->path[ii].v, cost_map_trailer[i][j][k][a][b]->path[ii].phi);
//								}
//
//							}
						}
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
//		else
//			printf("Gravacao do Cost Map realizada com sucesso! Arquivo: %s\n", astar_config.precomputed_cost_trailer_file_name);
	}
	else
		printf("Erro no fechamento do arquivo Cost Map %s!\n", astar_config.precomputed_cost_trailer_file_name);

	if (just_a_test == 2 && fclose(paths_file) != 0)
		printf("Erro no fechamento do arquivo Paths %s!\n", binary_file_type);

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
						int d, e, f, g, h, h_forward_valid, h_backward_valid, indice;
						int result = fscanf(fp, "%d %d %d %d %d %lf %lf %d %d %d\n", &d, &e, &f, &g, &h,
								&(cost_map_trailer[i][j][k][a][b]->h_forward), &(cost_map_trailer[i][j][k][a][b]->h_backward),
								&h_forward_valid, &h_backward_valid, &indice);
						cost_map_trailer[i][j][k][a][b]->h_forward_valid = h_forward_valid;
						cost_map_trailer[i][j][k][a][b]->h_backward_valid = h_backward_valid;
//						cost_map_trailer[i][j][k][a][b]->indice = indice; // Comentado por conta dos problemas que podem surgir se for utilizado
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
		{(char *) "semi_trailer",	 	(char *) "initial_type",								CARMEN_PARAM_INT, 	 &(semi_trailer_config.num_semi_trailers), 									0, NULL},
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

        astar_config.goal_constraints.goal_achieved_distance = MAX_POLY_DISTANCE;
        astar_config.goal_constraints.goal_achieved_theta = MAX_POLY_THETA_DIFF;
        astar_config.goal_constraints.goal_achieved_trailer_theta = MAX_POLY_BETA_DIFF;
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
build_jobs_vector(std::vector<job_params> &job_vector, int &total_jobs)
{
	int final_index = 0;

	int x_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);
	int y_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);

	for (int b = 0; b < astar_config.precomputed_cost_trailer_beta_size; b++) // beta da origem
	{
		for (int a = 0; a < astar_config.precomputed_cost_trailer_beta_size; a++) // beta do goal
		{
			for (int z = 0; z < astar_config.precomputed_cost_trailer_theta_size; z++)
			{
				for (int j = 0; j < y_size; j++)
				{
					for (int i = 0; i < x_size; i++)
					{
						job_params current_job;
						current_job.i = i;
						current_job.j = j;
						current_job.z = z;
						current_job.a = a;
						current_job.b = b;
						current_job.path_cost = cost_map_trailer[i][j][z][a][b]->h_forward;
						job_vector.push_back(current_job);
						final_index++;
					}
				}
			}
		}
	}

	total_jobs = final_index;

	return (0);	// first job
}


void
execute_one_job(int index, std::vector<job_params> &job_vector, int job_size, int job_stride, int total_jobs)
{
	int x_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);
	int y_size = round(astar_config.precomputed_cost_trailer_size / astar_config.precomputed_cost_trailer_resolution);
	double delta_x = astar_config.precomputed_cost_trailer_size / (double) x_size;
	double delta_y = astar_config.precomputed_cost_trailer_size / (double) y_size;

	int last_job = index + (job_size * job_stride);
	if (last_job > total_jobs)
		last_job = total_jobs;

#ifndef PLOT_PATH
#pragma omp parallel for default(none) shared (cost_map_trailer, job_vector, job_size, job_stride, total_jobs, index) firstprivate (last_job, astar_config, x_size, y_size, delta_x, delta_y, robot_config, semi_trailer_config)
#endif
	for (int current_index = index; current_index < last_job; current_index += job_stride)
	{
		int i, j, z, a, b;
		i = job_vector[current_index].i;
		j = job_vector[current_index].j;
		z = job_vector[current_index].z;
		a = job_vector[current_index].a;
		b = job_vector[current_index].b;
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
		if (cost_map_trailer[i][j][z][a][b]->h_forward == -2.0)
		{
			std::vector<carmen_robot_and_trailers_traj_point_t> path_valid;
			double path_cost = trailer_analytical_expansion_cost(path_valid , current, goal, 1, priv_robot_config, priv_semi_trailer_config);

//				double path_cost = i+j+z+a+b;
			cost_map_trailer[i][j][z][a][b]->h_forward = path_cost;
			if (cost_map_trailer[i][j][z][a][b]->h_forward > 0.0)
				cost_map_trailer[i][j][z][a][b]->h_forward_valid = true;
			cost_map_trailer[i][j][z][a][b]->h_backward = path_cost;
			if (cost_map_trailer[i][j][z][a][b]->h_backward > 0.0)
				cost_map_trailer[i][j][z][a][b]->h_backward_valid = true;
//				printf("if %d %d %d %f\n", current_index, job_size*job_stride, total_jobs, cost_map_trailer[i][j][z][a][b]->h_forward);

			if (path_cost > 0.0)
			{
				carmen_robot_and_trailers_traj_point_t *alloc_path = NULL;
				alloc_path = (carmen_robot_and_trailers_traj_point_t *) calloc(80, sizeof(carmen_robot_and_trailers_traj_point_t));
				do {if ((void *)(alloc_path) == NULL) exit(printf("Out of memory in (%s, line %d).\n", __FILE__, __LINE__)); } while (0); //macro do carmen_test_alloc colocado para não dar problema no pragma

				for (int ii = 0; ii < 80; ii++)
				{
					alloc_path[ii].x = path_valid[ii].x;
					alloc_path[ii].y = path_valid[ii].y;
					alloc_path[ii].theta = path_valid[ii].theta;
					alloc_path[ii].trailer_theta[0] = path_valid[ii].trailer_theta[0];
					alloc_path[ii].v = path_valid[ii].v;
					alloc_path[ii].phi = path_valid[ii].phi;
//					printf("path: %f %f %f \n", alloc_path[ii].x, alloc_path[ii].y, alloc_path[ii].theta);
				}
				cost_map_trailer[i][j][z][a][b]->path = &alloc_path[0];

			}
		}
//			else
//				printf("else %d %d %d %f\n", current_index, job_size*job_stride, total_jobs, cost_map_trailer[i][j][z][a][b]->h_forward);
#ifdef PLOT_PATH
		printf("F %d %d %d %d %d - current = %f %f %5.2f %5.2f goal %f %f %5.2f %5.2f path cost = %f\n", i, j, z, a, b,
				current.x, current.y, carmen_radians_to_degrees(current.theta), carmen_radians_to_degrees(current.trailer_theta[0]),
				goal.x, goal.y, carmen_radians_to_degrees(goal.theta), carmen_radians_to_degrees(goal.trailer_theta[0]), path_cost);
		if ((path_cost > 0.0) && (path_cost < 1000.0))
			getchar();
#endif
#else
		double path_cost = trailer_analytical_expansion_cost(path_valid, current, goal, 1, priv_robot_config, priv_semi_trailer_config);
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
		path_cost = trailer_analytical_expansion_cost(path_valid, current, goal, -1, priv_robot_config, priv_semi_trailer_config);
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
	}
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	carmen_get_parameters(argc, argv);

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

	if (argc != 3)
		exit(printf("Error: Invalid number of parameters.\n Usage %s <job_stride> <job_size>\n", argv[0]));
	int job_stride = atoi(argv[1]);
	int job_size =  atoi(argv[2]);

	alloc_trailer_cost_map();

	clear_trailer_cost_map();

	// Le nada, parcial ou total
///	load_current_trailer_cost_map();
	// Desativado para a implementação de salvar paths

	// Monta um vetor com os indices da matriz que precisam ser computados
	int total_jobs;
	std::vector<job_params> job_vector;
	int next_job = build_jobs_vector(job_vector, total_jobs);
	printf("\ntotal_jobs = %d, job_stride = %d, job_size = %d, number of job sets (total_jobs / (job_size * job_stride)) = %d, next_job = %d\n\n", total_jobs, job_stride, job_size, total_jobs / (job_size * job_stride), next_job);
	double t0 = carmen_get_time();
	for (int i = next_job; i < total_jobs; i += (job_size * job_stride))
	{
		double t1 = carmen_get_time();
		printf("current job %d, current job set %d, ", i, i / (job_size * job_stride));
		execute_one_job(i, job_vector, job_size, job_stride, total_jobs);
		save_trailer_cost_map(0);

		double t = carmen_get_time();
		double average_job_set_time = (t - t0) / ((double) ((i + (job_size * job_stride)) - next_job) / (double) (job_size * job_stride));
		double remaining_time = ((double) (total_jobs - i) / (double) (job_size * job_stride)) * average_job_set_time;
		printf("job set time %.2lfs, average job set time %.2lfs, expected remaining time %.2lfs, %.2lfm, %.2lfh, %.2lfd\n",
				t - t1, average_job_set_time,
				remaining_time, remaining_time / 60.0, remaining_time / (60.0 * 60.0), remaining_time / (60.0 * 60.0 * 24.0));
	}

	// Suavizacao
	initialize_missing_cells_in_the_cost_map();
	fill_in_missing_cells_in_the_cost_map();
//	save_trailer_cost_map(0);
	// Passei a usar o parâmetro 2 para informar que é a última vez que vai salvar a matriz, assim salvando os paths no arquivo binário
	save_trailer_cost_map(2);
	free_trailer_cost_map();

	double total_time = carmen_get_time() - t0;
	printf("Total time %.2lfs, %.2lfm, %.2lfh, %.2lfd\n", total_time, total_time / 60.0, total_time / (60.0 * 60.0), total_time / (60.0 * 60.0 * 24.0));

	return (0);
}
