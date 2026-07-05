#include "../nlp_mat_planner_path_planner_astar.h"
#include <stdio.h>

//#define PLOT_PATH

carmen_robot_ackerman_config_t robot_config;
carmen_semi_trailers_config_t semi_trailer_config;
nonholonomic_heuristic_cost_p ***cost_map;

int precomputed_cost_theta_size;
double precomputed_cost_size;
double precomputed_cost_resolution;
char* precomputed_cost_file_name;


void
alloc_cost_map()
{
	int x_size = round(precomputed_cost_size / precomputed_cost_resolution);
	int y_size = round(precomputed_cost_size / precomputed_cost_resolution);

	cost_map = (nonholonomic_heuristic_cost_p ***) calloc(x_size, sizeof(nonholonomic_heuristic_cost_p**));
	carmen_test_alloc(cost_map);

	for (int i = 0; i < x_size; i++)
	{
		cost_map[i] = (nonholonomic_heuristic_cost_p **) calloc(y_size, sizeof(nonholonomic_heuristic_cost_p*));
		carmen_test_alloc(cost_map[i]);

		for (int j = 0; j < y_size; j++)
		{
			cost_map[i][j] = (nonholonomic_heuristic_cost_p *) calloc(precomputed_cost_theta_size, sizeof(nonholonomic_heuristic_cost_p));
			carmen_test_alloc(cost_map[i][j]);

			for (int z = 0; z < precomputed_cost_theta_size; z++)
			{
				cost_map[i][j][z]= (nonholonomic_heuristic_cost_p) malloc(sizeof(nonholonomic_heuristic_cost));
				carmen_test_alloc(cost_map[i][j][z]);
			}
		}
	}
}


void
clear_cost_map()
{
	int x_size = round(precomputed_cost_size  / precomputed_cost_resolution);
	int y_size = round(precomputed_cost_size / precomputed_cost_resolution);

	for (int i = 0; i < x_size; i++)
		for (int j = 0; j < y_size; j++)
			for (int z = 0; z < precomputed_cost_theta_size; z++)
				free(cost_map[i][j][z]);
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
		fprintf(gnuplot_pipeMP, "set xrange [%d:%d]\n", (int) ((double) -precomputed_cost_size * 0.1), (int) ((double) precomputed_cost_size * 1.1));
		fprintf(gnuplot_pipeMP, "set yrange [%d:%d]\n", (int) ((double) -precomputed_cost_size * 0.1), (int) ((double) precomputed_cost_size * 1.1));
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

		fprintf(gnuplot_data_file, "%lf %lf %lf %lf %lf %lf %lf\n", robot_path.at(i).x, robot_path.at(i).y,
				distance * cos(robot_path.at(i).theta), distance * sin(robot_path.at(i).theta), carmen_normalize_theta(robot_path.at(i).theta), robot_path.at(i).phi,
				robot_path.at(i).trailer_theta[0]);
	}
	fclose(gnuplot_data_file);

	fprintf(gnuplot_pipeMP, "plot "
			"'./gnuplot_data_path.txt' using 1:2:3:4 w vectors title 'robot path'\n");

	fflush(gnuplot_pipeMP);
}


double
reed_shepp_cost(carmen_robot_and_trailers_traj_point_t current, carmen_robot_and_trailers_traj_point_t goal)
{
	int rs_pathl;
	int rs_numero;
	double tr;
	double ur;
	double vr;
	carmen_robot_and_trailers_traj_point_t rs_points[6]; // Por alguma razão, com o valor 5 acontece stack smashing às vezes quando o rs_pathl == 5
	double v_step;
	double path_cost = 0.0;

	rs_init_parameters(robot_config.max_phi, robot_config.distance_between_front_and_rear_axles);
	double rs_length = reed_shepp(current, goal, &rs_numero, &tr, &ur, &vr);

#ifdef PLOT_PATH
	rs_pathl = constRS(rs_numero, tr, ur, vr, current, rs_points);
	vector<carmen_robot_and_trailers_traj_point_t> robot_path;
//#endif
	for (int i = rs_pathl; i > 0; i--)
	{
		carmen_robot_and_trailers_traj_point_t point = rs_points[i];
		carmen_robot_and_trailers_traj_point_t next_point = rs_points[i - 1];
		if (point.v < 0.0)
			v_step = M_SQRT2;
		else
			v_step = -M_SQRT2;

		point.v = v_step;
		while ((DIST2D(point, next_point) > 0.1) ||
			   (fabs(carmen_radians_to_degrees(carmen_normalize_theta(point.theta - next_point.theta))) > 3.0))
		{
			double distance_traveled = 0.0;
			point = carmen_libcarmodel_recalc_pos_ackerman(point, v_step, point.phi,
					0.1, &distance_traveled, 0.01, robot_config, semi_trailer_config);
			path_cost += distance_traveled;
//#ifdef PLOT_PATH
			robot_path.push_back(point);
//#endif
		}
	}

//#ifdef PLOT_PATH
	plot_path(robot_path);
#endif

	return (rs_length);
}


double
calculate_continuous_theta(int z, int precomputed_cost_theta_size)
{
	// precomputed_cost_theta_size_trailer tem que ser impar.
//	double theta = (double) z * ((2.0 * M_PI) / (double) precomputed_cost_theta_size);
//	theta = carmen_normalize_theta(theta);

	// Igualando o método do trailer analytical expansion
	double theta = (double) z * ((2.0 * M_PI) / (double) precomputed_cost_theta_size);
	theta = carmen_normalize_theta(theta) / 2.0;

	return (theta);
}


void
make_cost_matrix()
{
	int x_size = round(precomputed_cost_size / precomputed_cost_resolution);
	int y_size = round(precomputed_cost_size / precomputed_cost_resolution);
	printf("sizemap = %d %d \n", x_size, y_size);
	carmen_robot_and_trailers_traj_point_t current;
	current.x = (double) precomputed_cost_size / 2.0;
	current.y = (double) precomputed_cost_size / 2.0;
	current.theta = 0.0;
	current.v = 0.0;
	current.phi = 0.0;
	current.trailer_theta[0] = 0.0;

#pragma omp parallel for default(none) shared (cost_map) firstprivate (x_size, y_size, current, robot_config, precomputed_cost_resolution, precomputed_cost_theta_size)
	for (int z = 0; z < precomputed_cost_theta_size; z++)
	{
		printf("current: z = %d\n", z);
		for (int j = 0; j < y_size; j++)
		{
			for (int i = 0; i < x_size; i++)
			{
				carmen_robot_and_trailers_traj_point_t goal;
				goal.x = (double) i * precomputed_cost_resolution + precomputed_cost_resolution / 2.0;
				goal.y = (double) j * precomputed_cost_resolution + precomputed_cost_resolution / 2.0;
				goal.theta = carmen_normalize_theta(calculate_continuous_theta(z, precomputed_cost_theta_size));
				double path_cost = reed_shepp_cost(current, goal);
//				printf("%d %d %d - current = %f %f %f goal %f %f %f path cost = %f\n", i, j, z, current.x, current.y, current.theta, goal.x, goal.y, goal.theta, path_cost);
#ifdef PLOT_PATH
				getchar();
#endif
				cost_map[i][j][z]->h = path_cost;
			}
		}
	}
}


static int
save_map()
{
	FILE *fp;
	int result;
	int x_size = round(precomputed_cost_size / precomputed_cost_resolution);
	int y_size = round(precomputed_cost_size / precomputed_cost_resolution);
	fp = fopen(precomputed_cost_file_name, "wt");

	if (fp == NULL)
		exit(printf ("Houve um erro ao abrir o arquivo %s pata escrita.\n", precomputed_cost_file_name));

	for (int i = 0; i < x_size; i++)
	{
		for (int j = 0; j < y_size; j++)
		{
			for (int k = 0; k < precomputed_cost_theta_size; k++)
			{
				if (cost_map[i][j][k] != NULL)
					result = fprintf(fp, "%d %d %d %lf\n", i, j, k, cost_map[i][j][k]->h);
				else
					exit(printf("Erro na alocacao da matrix cost_map[][][] ou no seu acesso."));

				if (result < 0)
					exit(printf("Erro na Gravacao do arquivo %s, result = %d\n", precomputed_cost_file_name, result));
			}
		}
	}

	fclose (fp);

	printf("Arquivo com a tabela gravado com sucesso! Arquivo: %s\n", precomputed_cost_file_name);

	return (0);
}


static void
carmen_get_parameters(int argc, char** argv)
{
	double max_phi_multiplier;

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
			{(char *) "offroad",			(char *) "planner_max_phi_multiplier", 					CARMEN_PARAM_DOUBLE, &max_phi_multiplier, 1, NULL},
			{(char *) "semi_trailer",	 	(char *) "initial_type",								CARMEN_PARAM_INT, 	 &(semi_trailer_config.num_semi_trailers), 					0, NULL},
			{(char *) "offroad",			(char *) "planner_precomputed_cost_size", 				CARMEN_PARAM_DOUBLE, &precomputed_cost_size, 										1, NULL},
			{(char *) "offroad",			(char *) "planner_precomputed_cost_theta_size", 		CARMEN_PARAM_INT, 	 &precomputed_cost_theta_size, 									1, NULL},
			{(char *) "offroad",			(char *) "planner_precomputed_cost_resolution", 		CARMEN_PARAM_DOUBLE, &precomputed_cost_resolution,									1, NULL},
			{(char *) "offroad",			(char *) "planner_precomputed_cost_file_name", 			CARMEN_PARAM_STRING, &precomputed_cost_file_name,									1, NULL},
		};


	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	robot_config.max_phi *= max_phi_multiplier;
	// É necessário não ser dos parametros, porque os parametros tem um path relativo que vai dar erro ao terminar a criação da matriz
	precomputed_cost_file_name = (char *) "offroad_planner_car_cost_matrix_101x36x1.0.data";

        astar_config.goal_constraints.goal_achieved_distance = MAX_POLY_DISTANCE;
        astar_config.goal_constraints.goal_achieved_theta = MAX_POLY_THETA_DIFF;
        astar_config.goal_constraints.goal_achieved_trailer_theta = MAX_POLY_BETA_DIFF;

}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	carmen_get_parameters(argc, argv);

	alloc_cost_map();

	make_cost_matrix();

	save_map();

	clear_cost_map();

	return 0;
}
