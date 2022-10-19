/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/
#include <carmen/carmen.h>
#include <assert.h>
#include "planner_ackerman_interface.h"
#include "conventional_ackerman.h"
#include "trajectory_ackerman.h"
#include "navigator_precomputed_cost_ackerman.h"
#include "navigator_astar.hpp"

int precomputed_cost_call_cont = 0;

carmen_robot_ackerman_config_t robot_config;
carmen_navigator_ackerman_astar_t astar_config;


double*** map;

#define FILE_NAME "mapas/precomp_ackerman_1000_phi20_r0.2_i0.01.data"
int OPEN_MAP = 0;
int x_size = 1000;
int y_size = 1000;
double resolution = 0.2;
double interval = 0.01;

carmen_astar_node_p ***astar_map;
FH* heap_pcc = NULL;
carmen_robot_and_trailers_traj_point_t msg_point[500];
carmen_robot_and_trailers_traj_point_t GOAL;
carmen_robot_and_trailers_traj_point_t START;

double RAIO;

double ORIENTATION[ORIENTATION_LENGHT];
double DIRECTION[DIRECTION_LENGHT];


static int
get_astar_map_theta(double theta)
{
	theta = theta < 0 ? (2 * M_PI + theta) : theta;
	return  (int)round((carmen_radians_to_degrees(theta) / astar_config.state_map_theta_resolution)) % (int)round(360 / astar_config.state_map_theta_resolution);
}


static int print_map_theta(int theta)
{
	int x, y;
	for(y = 0; y < y_size; y++)
	{printf("%d\t",y);
		for(x = 0; x < x_size; x++)
		{
			printf("%.2f\t",map[x][y][theta]);
		}
		printf("\n");
	}
	printf("\n\n");
	return 0;
}


static int to_ppm(char *output_filename, int theta)
{
	int x, y;
	char c;
	FILE *fp;

	fp = fopen(output_filename, "w");
	if (fp == NULL)
		return -1;

	fprintf(fp, "P6\n%d %d\n255\n", x_size, y_size);
	for(y = 0; y < y_size; y++)
	{
		for(x = 0; x < x_size; x++)
		{
			if(map[x][y][theta] == -1)
			{
				fputc(0, fp); fputc(255, fp); fputc(255, fp);
			}
			else
			{
				c = 255 - (map[x][y][theta]) * 100;
				fputc(c, fp); fputc(c, fp); fputc(c, fp);
			}
		}
	}
	fclose(fp);

	return 0;
}


static int to_ppm_trim_ration(char *output_filename, int theta)
{
	int x, y;
	char c;
	FILE *fp;

	fp = fopen(output_filename, "w");
	if (fp == NULL)
		return -1;

	fprintf(fp, "P6\n%d %d\n255\n", x_size, y_size);
	for(y = 0; y < y_size; y++)
	{
		for(x = 0; x < x_size; x++)
		{

			if(map[x][y][theta] == -1)
			{
				fputc(255, fp); fputc(0, fp); fputc(255, fp);
			}
			else
			{
				c = (sqrt(pow(START.x - x * resolution, 2) + pow(START.y - y * resolution, 2)) / (map[x][y][theta] / resolution)) * 200;
				fputc(c, fp); fputc(c, fp); fputc(c, fp);
			}
		}
		//printf("\n");
	}
	fclose(fp);

	return 0;
}


int cont = 0;
static carmen_robot_and_trailers_traj_point_t
carmen_precomputed_cost_ackerman_kinematic(carmen_robot_and_trailers_traj_point_t point, float lenght, float phi, float v)
{
	phi = carmen_degrees_to_radians(phi);
	point.theta += v * (tan(phi) / lenght);
	//point.theta = normalize_radians(point.theta);
	point.theta = carmen_normalize_theta(point.theta);
	//point.theta = point.theta < 0 ? (1 * M_PI - point.theta) : point.theta;
	point.x += v * cos(point.theta);
	point.y += v * sin(point.theta);
	point.v = v;
	point.phi = phi;
	return point;
}

//todo modificar para que um nó do A* aponte para o noh do FH, pois ele precisa ser repriorizado
static void add_list_fh(carmen_astar_node_p new_node)
{
	int x, y, theta;
	x = round(new_node->point.x / resolution);
	y = round(new_node->point.y / resolution);
	theta = get_astar_map_theta(new_node->point.theta);
	//theta = round((carmen_radians_to_degrees(new_node->point.theta)) / astar_config.state_map_theta_resolution);

	if (astar_map[x][y][theta] == NULL || astar_map[x][y][theta]->f_score > new_node->f_score
			|| astar_map[x][y][theta]->astar_call_cont != precomputed_cost_call_cont)
	{
		if (astar_map[x][y][theta]==NULL|| astar_map[x][y][theta]->astar_call_cont != precomputed_cost_call_cont)
		{
			astar_map[x][y][theta] = new_node;
			astar_map[x][y][theta]->fh_node = NULL;
		}
		else
		{
			astar_map[x][y][theta]->prev_point = new_node->prev_point;
			astar_map[x][y][theta]->f_score = new_node->f_score;
			astar_map[x][y][theta]->g_score = new_node->g_score;
			astar_map[x][y][theta]->h_score = new_node->h_score;
			astar_map[x][y][theta]->range = new_node->range;
			astar_map[x][y][theta]->point = new_node->point;
			astar_map[x][y][theta]->direction = new_node->direction;
			free(new_node);
		}

		if(theta == 71)
		{
			//printf("x %d y %d theta %d score %f\n", x, y, theta, new_node->f_score);
			cont ++;
			//printf("%d\n",cont);
		}

		FH_NODE* fh_node = NULL;
		fh_node = FH_MAKE_NODE(round((astar_map[x][y][theta]->f_score * 10000) / resolution));
		FH_INSERT(heap_pcc, fh_node);
		fh_node->PTR = astar_map[x][y][theta];
		astar_map[x][y][theta]->fh_node = fh_node;
		astar_map[x][y][theta]->status = CLOSE;

	}
	else
	{
		free(new_node);
	}
}

/*static double calc_delta_theta(double theta1, double theta2){
	double delta_theta;
	delta_theta = normalize_radians((theta1 - theta2));
	if(delta_theta>M_PI)delta_theta=2*M_PI-delta_theta;
	return delta_theta;
}*/


void
free_astar_map()
{
	int i, j, k;
	int theta_size = round(365 / astar_config.state_map_theta_resolution);
	for(i = 0; i < x_size; i++)
	{
		for(j = 0; j < y_size; j++)
		{
			for(k = 0; k < theta_size; k++)
			{
				if( astar_map[i][j][k] != NULL )
				{
					free(astar_map[i][j][k]);
					astar_map[i][j][k] = NULL;
				}
			}
		}
	}
}


void
alloc_astar_map()
{
	int i, j;
	int theta_size = round(365 / astar_config.state_map_theta_resolution);

	astar_map = (carmen_astar_node_p ***)calloc(x_size + 1, sizeof(carmen_astar_node_p**));
	carmen_test_alloc(astar_map);
	for(i = 0; i < x_size + 1; i++)
	{
		astar_map[i] = (carmen_astar_node_p **)calloc(y_size + 1, sizeof(carmen_astar_node_p*));
		carmen_test_alloc(astar_map[i]);
		for(j = 0;j < y_size + 1; j++)
		{
			astar_map[i][j] = (carmen_astar_node_p*)calloc(theta_size, sizeof(carmen_astar_node_p));
			carmen_test_alloc(astar_map[i][j]);
		}
	}
}


static int save_map()
{
	FILE *fp;
	int i, j , k, result;
	fp = fopen (FILE_NAME, "wt");
	int theta_size = round(365 / astar_config.state_map_theta_resolution);

	if (fp == NULL)
	{
		printf ("Houve um erro ao abrir o arquivo.\n");
		return 1;
	}
	for (i = 0; i < x_size; i++)
	{
		for (j = 0; j < y_size; j++)
		{
			for (k = 0; k < theta_size; k++)
			{
				if (astar_map[i][j][k] != NULL)
				{
					result = fprintf(fp,"%lf ", astar_map[i][j][k]->f_score);
					map[i][j][k] = astar_map[i][j][k]->f_score;
				}
				else
				{
					result = fprintf(fp, "-1 ");
					map[i][j][k] = -1;
				}
				if (result == EOF)
					printf("Erro na Gravacao\n");
			}
		}
	}
	fclose (fp);
	return 0;
}


static int open_map()
{
	FILE *fp;
	int i, j, k, result;
	int theta_size = round(365 / astar_config.state_map_theta_resolution);

	fp = fopen (FILE_NAME, "rw");
	if (fp == NULL)
	{
		printf ("Houve um erro ao abrir o arquivo.\n");
		return 1;
	}

	for (i = 0; i < x_size; i++)
	{
		for (j = 0; j < y_size; j++)
		{
			for (k = 0; k < theta_size; k++)
			{
				result = fscanf(fp, "%lf ", &map[i][j][k]);
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
static void open_node(carmen_astar_node_p node)
{
	int i, j;
	double k;
	double x, y;
	carmen_robot_and_trailers_traj_point_t new_point;
	carmen_astar_node_p new_node;
	for (i = 0; i < DIRECTION_LENGHT; i++)
	{
		for (j = 0; j < ORIENTATION_LENGHT; j++)
		{
			for (k = 0; k <= astar_config.path_interval; k = k + interval)
			{
				new_point = carmen_precomputed_cost_ackerman_kinematic(
						node->point, robot_config.distance_between_front_and_rear_axles, ORIENTATION[j], DIRECTION[i] * k);
				x = round((new_point.x) / resolution);
				y = round((new_point.y) / resolution);
				if (x < 0 || x >= x_size || y < 0 || y >= y_size)
					continue;

				new_node = malloc(sizeof(carmen_astar_node_t));
				new_node->g_score = node->g_score + astar_config.path_interval * k;
				new_node->f_score = new_node->g_score;
				new_node->direction = DIRECTION[i] * k;
				new_node->range = node->range + 1;
				new_node->point = new_point;
				new_node->prev_point = node->point;
				new_node->astar_call_cont = precomputed_cost_call_cont;
				add_list_fh(new_node);
			}
		}
	}
}

void
carmen_precomputed_cost_ackerman_dijkstra()
{
	DIRECTION[0] = -astar_config.path_interval;
	DIRECTION[1] = astar_config.path_interval;

	ORIENTATION[0] = -carmen_radians_to_degrees(robot_config.max_phi);
	ORIENTATION[1] = 0;
	ORIENTATION[2] = carmen_radians_to_degrees(robot_config.max_phi);

	carmen_robot_and_trailers_traj_point_t start;
	start.x = (x_size / 2) * resolution;
	start.y = (y_size / 2) * resolution;
	start.theta = 0;
	start.v = 0;
	start.phi = 0;
	START = start;

	if (!OPEN_MAP)
	{
		printf("calculando\n");
		if (precomputed_cost_call_cont == 0)
			alloc_astar_map();
		precomputed_cost_call_cont++;
		carmen_astar_node_p node = malloc(sizeof(carmen_astar_node_t));


		//start.theta = normalize_radians(start.theta);
		start.theta = carmen_normalize_theta(start.theta);
		node->g_score = 0.0;
		node->f_score = node->g_score;
		node->range = 1;
		node->prev_point = start;
		node->point = start;
		node->fh_node = NULL;

		heap_pcc = FH_MAKE_HEAP();
		add_list_fh(node);
		FH_NODE* node_fh = NULL;
		int index = 0;
		while((node_fh = FH_EXTRACT_MIN(heap_pcc)))
		{
			node = node_fh->PTR;
			free(node_fh);
			node->fh_node = NULL;
			node->status = OPEN;
			open_node(node);
			index++;
		}
		save_map();
		//FH_FREE_HEAP(heap_pcc, true);
		//free_astar_map();

	}
	else
	{
		printf("abrindo mapa\n");
		open_map();
		printf("return\n");
	}
	return;
}

int
main(int argc, char **argv)
{
	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	int num_items;

	carmen_param_t param_list[] =
	{
		{"robot", "length", CARMEN_PARAM_DOUBLE, &robot_config.length, 0, NULL},
		{"robot", "width", CARMEN_PARAM_DOUBLE, &robot_config.width, 0, NULL},
		{"robot", "maximum_acceleration_forward", CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_forward, 1, NULL},
		{"robot", "reaction_time", CARMEN_PARAM_DOUBLE, &robot_config.reaction_time, 0, NULL},
		{"robot", "max_steering_angle", CARMEN_PARAM_DOUBLE, &robot_config.max_phi, 1, NULL},
		{"robot", "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE,
			&robot_config.distance_between_front_and_rear_axles , 1, NULL},
		{"navigator_astar", "path_interval", CARMEN_PARAM_DOUBLE, &astar_config.path_interval, 1, NULL},
		{"navigator_astar", "state_map_resolution", CARMEN_PARAM_INT, &astar_config.state_map_resolution, 1, NULL},
		{"navigator_astar", "state_map_theta_resolution", CARMEN_PARAM_INT, &astar_config.state_map_theta_resolution, 1, NULL}
	};
	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	int i, j;
	map = calloc(x_size + 1, sizeof(double**));
	carmen_test_alloc(map);
	for (i = 0; i < x_size + 1; i++)
	{
		map[i] = calloc(y_size + 1, sizeof(double*));
		carmen_test_alloc(map[i]);
		for (j = 0;j < y_size + 1; j++)
		{
			map[i][j] = calloc(365 / astar_config.state_map_theta_resolution, sizeof(double));
			carmen_test_alloc(map[i][j]);
		}
	}

	carmen_precomputed_cost_ackerman_dijkstra();
	char szBuffer[1024];
	char file_name[1024];

	int k;
//	for (k = 70; k <= 360 / astar_config.state_map_theta_resolution; k++)
//	{
//		printf("k %d\n",k);
//
//		print_map_theta(k);
////	}
	print_map_theta(0);

	for (k = 0; k < 360 / astar_config.state_map_theta_resolution; k++)
	{
		memset (&file_name, 0, sizeof (file_name) );
		snprintf (szBuffer, sizeof (szBuffer), "mapa_%d.ppm", k);
		strcat(file_name, FILE_NAME);
		strcat(file_name, szBuffer);
		to_ppm(file_name, k);
	}

	for (k = 0; k <= 360 / astar_config.state_map_theta_resolution; k++)
	{
		memset (&file_name, 0, sizeof (file_name) );
		snprintf (szBuffer, sizeof (szBuffer), "trim_%d.ppm", k);
		strcat(file_name, FILE_NAME);
		strcat(file_name, szBuffer);
		to_ppm_trim_ration(file_name, k);

	}

/*
	for (k = 0; k < 360 / astar_config.state_map_theta_resolution; k++)
	{
		memset (&file_name, 0, sizeof (file_name) );
		snprintf (szBuffer, sizeof (szBuffer), "trim_theta_%d.ppm", k);
		strcat(file_name, FILE_NAME);
		strcat(file_name, szBuffer);
		to_ppm_trim_ration_with_theta(file_name, k);

	}
*/
/*	for (k = 0; k < 360 / astar_config.state_map_theta_resolution; k++)
	{
		memset (&file_name, 0, sizeof (file_name) );
		snprintf (szBuffer, sizeof (szBuffer), "map_%d.data", k);
		strcat(file_name, FILE_NAME);
		strcat(file_name, szBuffer);
		to_data(file_name, k);

	}*/
	/* Loop forever waiting for messages */
	//carmen_ipc_dispatch();

	return (0);
}
