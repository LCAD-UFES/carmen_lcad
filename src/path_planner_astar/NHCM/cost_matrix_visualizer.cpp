#include "../path_planner_astar.h"


#define HEURISTIC_THETA_SIZE 72
#define HEURISTIC_MAP_SIZE 100
#define HEURISTIC_GRID_RESOLUTION 0.2
#define FILE_NAME "cost_matrix_02_101x101x72.data"
#define USE_RS_COMPARISON 0

nonholonomic_heuristic_cost_p ***cost_map;

carmen_robot_ackerman_config_t robot_config;


double
carmen_compute_abs_angular_distance(double theta_1, double theta_2)
{
	return (carmen_normalize_theta(abs(theta_1 - theta_2)));
}


double
reed_shepp_cost(carmen_ackerman_traj_point_t current, carmen_ackerman_traj_point_t goal)
{
	int rs_pathl;
	int rs_numero;
	double tr;
	double ur;
	double vr;
	double distance_traveled = 0.0;
	double distance_traveled_old = 0.0;
	carmen_ackerman_traj_point_t rs_points[6]; // Por alguma razão, com o valor 5 acontece stack smashing às vezes quando o rs_pathl == 5
	double v_step;
	double step_weight;
	double path_cost = 0.0;
	carmen_ackerman_traj_point_t point_old = {0, 0, 0, 0, 0};
	robot_config.max_phi = 0.5237;
	robot_config.distance_between_front_and_rear_axles = 2.625;
	robot_config.understeer_coeficient = 0.0015;
	robot_config.maximum_steering_command_rate = 0.335;

	rs_init_parameters(robot_config.max_phi, robot_config.distance_between_front_and_rear_axles);
	double rs_length = reed_shepp(current, goal, &rs_numero, &tr, &ur, &vr);

	rs_pathl = constRS(rs_numero, tr, ur, vr, current, rs_points);
	for (int i = rs_pathl; i > 0 /*rs_pathl*/; i--)
	{
		carmen_ackerman_traj_point_t point = rs_points[i];
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
					0.1, &distance_traveled, DELTA_T, robot_config);
			path_cost += step_weight * (distance_traveled - distance_traveled_old);
//			printf("[rs] Comparação de pontos: %f %f\n", DIST2D(point, point_old), distance_traveled - distance_traveled_old);

		}
	}

	return path_cost;
}


static int to_ppm(char *output_filename, int theta)
{
	int x, y;
	char c;
	FILE *fp;

	fp = fopen(output_filename, "w");
	if (fp == NULL)
	{
		printf("fopen falhou\n");
		return -1;
	}
	carmen_ackerman_traj_point_t goal;
	carmen_ackerman_traj_point_t current;
	int map_size = round(HEURISTIC_MAP_SIZE  / HEURISTIC_GRID_RESOLUTION);

	if(USE_RS_COMPARISON)
	{
		goal.x = round(map_size/2 * HEURISTIC_GRID_RESOLUTION);
		goal.y = round(map_size/2 * HEURISTIC_GRID_RESOLUTION);
		goal.theta = 0.0;
	}

	fprintf(fp, "P6\n%d %d\n255\n", int(HEURISTIC_MAP_SIZE/HEURISTIC_GRID_RESOLUTION), int(HEURISTIC_MAP_SIZE/HEURISTIC_GRID_RESOLUTION));
	printf("Theta = %d\n", theta);
	for(x = 0; x < HEURISTIC_MAP_SIZE/HEURISTIC_GRID_RESOLUTION; x++)
	{
		for(y = 0; y < HEURISTIC_MAP_SIZE/HEURISTIC_GRID_RESOLUTION; y++)
		{
			if(USE_RS_COMPARISON)
			{
				current.x = x*HEURISTIC_GRID_RESOLUTION;
				current.y = y*HEURISTIC_GRID_RESOLUTION;
				current.theta = carmen_degrees_to_radians(theta*5);
				current.theta = carmen_normalize_theta(current.theta);
				printf("x = %d, y = %d theta = %d cost = %lf\n", x, y, theta, cost_map[x][y][theta]->h);
				printf("Real Reed-Shepp cost = %f %f %f %f", current.x, current.y, current.theta, reed_shepp_cost(current, goal));
//				printf("Real goal = %f %f %f\n", goal.x, goal.y, goal.theta);
			}

			if(cost_map[x][y][theta]->h == -1 || cost_map[x][y][theta]->h >= 255)
			{
				fputc(255, fp); fputc(255, fp); fputc(255, fp);
			}
			else
			{
//				c = 255 - (cost_map[x][y][theta]->h) * 100;
				c =  cost_map[x][y][theta]->h;
				fputc(c, fp); fputc(c, fp); fputc(c, fp);
			}
		}
	}
	fclose(fp);

	return 0;
}


void
alloc_cost_map()
{
	int i, j, z;
	int x_size = round(HEURISTIC_MAP_SIZE  / HEURISTIC_GRID_RESOLUTION);
	int y_size = round(HEURISTIC_MAP_SIZE / HEURISTIC_GRID_RESOLUTION);

	cost_map = (nonholonomic_heuristic_cost_p ***)calloc(x_size, sizeof(nonholonomic_heuristic_cost_p**));
	carmen_test_alloc(cost_map);

	for (i = 0; i < x_size; i++)
	{
		cost_map[i] = (nonholonomic_heuristic_cost_p **)calloc(y_size, sizeof(nonholonomic_heuristic_cost_p*));
		carmen_test_alloc(cost_map[i]);

		for (j = 0; j < y_size; j++)
		{
			cost_map[i][j] = (nonholonomic_heuristic_cost_p*)calloc(HEURISTIC_THETA_SIZE, sizeof(nonholonomic_heuristic_cost_p));
			carmen_test_alloc(cost_map[i][j]);

			for (z = 0; z < HEURISTIC_THETA_SIZE; z++)
			{
				cost_map[i][j][z]= (nonholonomic_heuristic_cost_p) malloc(sizeof(nonholonomic_heuristic_cost));
				carmen_test_alloc(cost_map[i][j][z]);


			}
		}
	}
}

static int
open_cost_map()
{
	FILE *fp;
	int i, j, k, result;

	fp = fopen (FILE_NAME, "rw");
	if (fp == NULL)
	{
		printf ("Houve um erro ao abrir o arquivo.\n");
		return 1;
	}

	for (i = 0; i < (HEURISTIC_MAP_SIZE/HEURISTIC_GRID_RESOLUTION); i++)
	{
		for (j = 0; j < (HEURISTIC_MAP_SIZE/HEURISTIC_GRID_RESOLUTION); j++)
		{
			for (k = 0; k < HEURISTIC_THETA_SIZE; k++)
			{
				result = fscanf(fp, "%lf ", &cost_map[i][j][k]->h);
//				printf("aqui %d %d %d\n", i, j, k);
				if (result == EOF)
				{
					printf("acho que acabou\n");
					break;
				}
			}
		}
	}
	fclose (fp);
	return 0;
}


void
clear_cost_map()
{
	int i, j, z;
	int x_size = round(HEURISTIC_MAP_SIZE  / HEURISTIC_GRID_RESOLUTION);
	int y_size = round(HEURISTIC_MAP_SIZE / HEURISTIC_GRID_RESOLUTION);

	for (i = 0; i < x_size; i++)
		for (j = 0; j < y_size; j++)
			for (z = 0; z < HEURISTIC_THETA_SIZE; z++)
				cost_map[i][j][z] = NULL;
}


int
main(int argc, char **argv)
{

	alloc_cost_map();
	open_cost_map();
	char szBuffer[1024];
	char file_name[1024];

	for (int k = 0; k < HEURISTIC_THETA_SIZE; k++)
	{
		memset (&file_name, 0, sizeof (file_name) );
		snprintf (szBuffer, sizeof (szBuffer), "temp_maps/trim_%d.ppm", k);
		//strcat(file_name, FILE_NAME);
		strcat(file_name, szBuffer);
		to_ppm(file_name, k);

}	clear_cost_map();

	return 0;
}


