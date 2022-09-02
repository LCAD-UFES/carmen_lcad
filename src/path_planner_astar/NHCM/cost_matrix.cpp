#include "../path_planner_astar.h"
#include <stdio.h>

carmen_robot_ackerman_config_t robot_config;
carmen_semi_trailer_config_t semi_trailer_config;
nonholonomic_heuristic_cost_p ***cost_map;

int precomputed_cost_size, precomputed_cost_theta_size;
double precomputed_cost_resolution;
char* precomputed_cost_file_name;

void
alloc_cost_map()
{
	int i, j, z;
	int x_size = round(precomputed_cost_size  / precomputed_cost_resolution);
	int y_size = round(precomputed_cost_size / precomputed_cost_resolution);

	cost_map = (nonholonomic_heuristic_cost_p ***)calloc(x_size, sizeof(nonholonomic_heuristic_cost_p**));
	carmen_test_alloc(cost_map);

	for (i = 0; i < x_size; i++)
	{
		cost_map[i] = (nonholonomic_heuristic_cost_p **)calloc(y_size, sizeof(nonholonomic_heuristic_cost_p*));
		carmen_test_alloc(cost_map[i]);

		for (j = 0; j < y_size; j++)
		{
			cost_map[i][j] = (nonholonomic_heuristic_cost_p*)calloc(precomputed_cost_theta_size, sizeof(nonholonomic_heuristic_cost_p));
			carmen_test_alloc(cost_map[i][j]);

			for (z = 0; z < precomputed_cost_theta_size; z++)
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
	int i, j, z;
	int x_size = round(precomputed_cost_size  / precomputed_cost_resolution);
	int y_size = round(precomputed_cost_size / precomputed_cost_resolution);

	for (i = 0; i < x_size; i++)
		for (j = 0; j < y_size; j++)
			for (z = 0; z < precomputed_cost_theta_size; z++)
				cost_map[i][j][z] = NULL;
}


double
carmen_compute_abs_angular_distance(double theta_1, double theta_2)
{
	return (carmen_normalize_theta(abs(theta_1 - theta_2)));
}


double
reed_shepp_cost(carmen_robot_and_trailers_traj_point_t current, carmen_robot_and_trailers_traj_point_t goal)
{
	int rs_pathl;
	int rs_numero;
	double tr;
	double ur;
	double vr;
	double distance_traveled = 0.0;
	double distance_traveled_old = 0.0;
	carmen_robot_and_trailers_traj_point_t rs_points[6]; // Por alguma razão, com o valor 5 acontece stack smashing às vezes quando o rs_pathl == 5
	double v_step;
	double step_weight;
	double path_cost = 0.0;
	carmen_robot_and_trailers_traj_point_t point_old = {0, 0, 0, 0, 0};

	rs_init_parameters(robot_config.max_phi, robot_config.distance_between_front_and_rear_axles);
	double rs_length = reed_shepp(current, goal, &rs_numero, &tr, &ur, &vr);

	rs_pathl = constRS(rs_numero, tr, ur, vr, current, rs_points);
	for (int i = rs_pathl; i > 0 /*rs_pathl*/; i--)
	{
		carmen_robot_and_trailers_traj_point_t point = rs_points[i];
		if (rs_points[i].v < 0.0)
		{
			v_step = 2.0;
			step_weight = 1.0;
		}
		else
		{
			v_step = -2.0;
			step_weight = 1.0;
		}
		while (DIST2D(point, rs_points[i-1]) > 0.2 || (abs(carmen_compute_abs_angular_distance(point.theta, rs_points[i-1].theta)) > 0.0872665))
		{
			distance_traveled_old = distance_traveled;
			point_old = point;
			point = carmen_libcarmodel_recalc_pos_ackerman(point, v_step, rs_points[i].phi,
					0.1, &distance_traveled, DELTA_T, robot_config, semi_trailer_config);
			path_cost += step_weight * (distance_traveled - distance_traveled_old);
//			printf("[rs] Comparação de pontos: %f %f\n", DIST2D(point, point_old), distance_traveled - distance_traveled_old);

		}
	}

	return path_cost;
}


double
calculate_continuous_theta(int z, int precomputed_cost_theta_size)
{
	// Apenas a inversao do metodo get_astar_map_theta
	return (double) ((z * 2.0 * M_PI)/ (precomputed_cost_theta_size - 1)) - M_PI;
}


void
make_matrix_cost()
{
	int i, j, z;
	int x_size = round(precomputed_cost_size  / precomputed_cost_resolution);
	int y_size = round(precomputed_cost_size / precomputed_cost_resolution);
	printf("sizemap = %d %d \n", x_size, y_size);
	carmen_robot_and_trailers_traj_point_t current;
	carmen_robot_and_trailers_traj_point_t goal;
	goal.x = round(x_size/2 * precomputed_cost_resolution);
	goal.y = round(y_size/2 * precomputed_cost_resolution);
	goal.theta = 0.0;
	double path_cost;

	for (i = 0; i < x_size; i++)
	{
		printf("current: x = %d\n", i);
		for (j = 0; j < y_size; j++)
		{

			for (z = 0; z < precomputed_cost_theta_size; z++)
			{
				current.x = i * precomputed_cost_resolution;
				current.y = j * precomputed_cost_resolution;
				current.theta = calculate_continuous_theta(z, precomputed_cost_theta_size);
				current.theta = carmen_normalize_theta(current.theta);
				path_cost = reed_shepp_cost(current, goal);
//				printf("current = %f %f %f goal %f %f %f path cost = %f\n", current.x, current.y, current.theta, goal.x, goal.y, goal.theta, path_cost);
				cost_map[i][j][z]->h = path_cost;


			}
		}
	}
}


static int save_map()
{
	FILE *fp;
	int i, j , k, result;
	int x_size = round(precomputed_cost_size / precomputed_cost_resolution);
	int y_size = round(precomputed_cost_size / precomputed_cost_resolution);
	fp = fopen(precomputed_cost_file_name, "wt");

	if (fp == NULL)
	{
		printf ("Houve um erro ao abrir o arquivo.\n");
		return 1;
	}
	for (i = 0; i < x_size; i++)
	{
		for (j = 0; j < y_size; j++)
		{
			for (k = 0; k < precomputed_cost_theta_size; k++)
			{
				if (cost_map[i][j][k] != NULL)
				{
					result = fprintf(fp,"%lf ", cost_map[i][j][k]->h);
				}
				else
				{
					result = fprintf(fp, "-1 ");
				}
				if (result == EOF)
					printf("Erro na Gravacao\n");
			}
		}
	}
	fclose (fp);
	return 0;
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
			{(char *) "semi_trailer",	 	(char *) "initial_type",								CARMEN_PARAM_INT, 	 &(semi_trailer_config.type), 									0, NULL},
			{(char *) "offroad",			(char *) "planner_precomputed_cost_size", 				CARMEN_PARAM_INT, 	 &precomputed_cost_size, 										1, NULL},
			{(char *) "offroad",			(char *) "planner_precomputed_cost_theta_size", 		CARMEN_PARAM_INT, 	 &precomputed_cost_theta_size, 									1, NULL},
			{(char *) "offroad",			(char *) "planner_precomputed_cost_resolution", 		CARMEN_PARAM_DOUBLE, &precomputed_cost_resolution,									1, NULL},
//			{(char *) "offroad",			(char *) "planner_precomputed_cost_file_name", 			CARMEN_PARAM_STRING, &precomputed_cost_file_name,									1, NULL},
		};

	precomputed_cost_file_name = (char *) "cost_matrix_02_101x101x72.data";
	robot_config.max_phi *= 0.6;
	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	carmen_get_parameters(argc, argv);

	alloc_cost_map();

	make_matrix_cost();

	save_map();

	clear_cost_map();

	return 0;
}
