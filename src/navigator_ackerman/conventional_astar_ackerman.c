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
#include "conventional_astar_ackerman.h"
#include "path_optimization_ackerman.h"
#include "FH/fib.h"
#include "navigator_ackerman_ipc.h"
#include "navigator_ackerman.h"
#include "trajectory_ackerman.h"
#include "rs.h"

extern carmen_behavior_selector_task_t current_task;

int astar_call_cont = 0;
int cont_nos_abertos_novos = 0, cont_nos_abertos_alterados = 0, cont_nos_podados = 0, cont_nos_abertos_alterados_fechados = 0;
carmen_map_point_t robot_posn;

extern carmen_robot_ackerman_config_t robot_conf_g;
extern carmen_map_t *carmen_planner_map;
extern carmen_navigator_ackerman_astar_t astar_config;

carmen_astar_node_p ***astar_map;
double ***precomputed_cost_map;
struct fibheap*  heap = NULL;


carmen_ackerman_traj_point_t GOAL;
//double RAIO;

double ORIENTATION[ORIENTATION_LENGHT];
double DIRECTION[DIRECTION_LENGHT];

int is_obstacle_cost(carmen_ackerman_traj_point_t point);
int is_obstacle_new(carmen_ackerman_traj_point_t point);


void print_path(carmen_planner_path_p path)
{
	int i;
	for (i = 0; i < path->length; i++)
	{
		printf(" x %.2f y %.2f phi %.2f\ttheta %.2f T x %.2f\t\n", path->points[i].x, path->points[i].y,carmen_radians_to_degrees(path->points[i].phi), carmen_radians_to_degrees(path->points[i].theta),path->points[i].v);

	}
}


static int
get_astar_map_x(double x)
{
	x = round((x - carmen_planner_map->config.x_origin) / carmen_planner_map->config.resolution / astar_config.state_map_resolution);
	return x;
}


static int
get_astar_map_y(double y)
{
	y = round((y - carmen_planner_map->config.y_origin) / carmen_planner_map->config.resolution / astar_config.state_map_resolution);
	return y;
}


static int
get_astar_map_theta(double theta)
{
	theta = theta < 0 ? (2 * M_PI + theta) : theta;
	return  (int)round((carmen_radians_to_degrees(theta) / astar_config.state_map_theta_resolution)) % (int)round(360 / astar_config.state_map_theta_resolution);
}

static int
get_astar_map_theta_2(double theta)
{
	theta = theta < 0 ? (2 * M_PI + theta) : theta;
	return  (int)round((carmen_radians_to_degrees(theta) / 5)) % (int)round(360 / 5);
}


carmen_ackerman_traj_point_t
carmen_conventional_astar_ackerman_kinematic_2(carmen_ackerman_traj_point_t point, float lenght, float phi, float v)
{
	double interval_time = 1;
	double time, rest;
	double phi_signal = phi / fabs(phi);

	if(phi != 0)
		phi = point.phi + carmen_degrees_to_radians(4) * phi_signal ;


	int i;
	int	   n;
	time = 0.1;
	n = interval_time / time;
	rest = interval_time - (n * time);

	robot_conf_g.maximum_steering_command_rate = 0.05;

	for (i = 0; i < n; i++)
	{

		if (fabs(point.phi - phi) < carmen_degrees_to_radians(1) || fabs(robot_conf_g.max_phi) < fabs(point.phi))
		{
			point.phi = phi;
		}
		else
		{
			point.phi += atan(lenght * robot_conf_g.maximum_steering_command_rate * fabs(v) * time) * phi_signal;
		}

		point.theta += v * time * ((tan(point.phi)) / lenght);
		point.x += v * time * cos(point.theta);
		point.y += v * time * sin(point.theta);

	}

	if (fabs(point.phi - phi) < carmen_degrees_to_radians(1) || fabs(robot_conf_g.max_phi) < fabs(point.phi))
	{
		point.phi = phi;
	}
	else
	{
		point.phi += atan(lenght * robot_conf_g.maximum_steering_command_rate * fabs(v) * time) * phi_signal;
	}

	if (fabs(point.phi - phi) < carmen_degrees_to_radians(4) || fabs(robot_conf_g.max_phi) < fabs(point.phi))
	{
		point.phi = phi;
	}

	point.theta += v * rest * ((tan(point.phi)) / lenght);
	point.theta = carmen_normalize_theta(point.theta);
	point.x += v * rest * cos(point.theta);
	point.y += v * rest * sin(point.theta);


	point.v = v;

	return point;
}

carmen_ackerman_traj_point_t
carmen_conventional_astar_ackerman_kinematic(carmen_ackerman_traj_point_t point, double lenght, double phi, double v)
{
	point.theta += v * ((tan(phi)) / lenght);
	point.theta = carmen_normalize_theta(point.theta);
	point.x += v * cos(point.theta);
	point.y += v * sin(point.theta);
	point.v = v;
	point.phi = phi;
	return point;
}


carmen_ackerman_traj_point_t
carmen_conventional_astar_ackerman_kinematic_3(carmen_ackerman_traj_point_t point, double lenght, double phi, double v)
{

	double	radcurv = lenght / tan(fabs(phi));

	if(phi == 0)
	{

		point.x += v * cos(point.theta);
		point.y += v * sin(point.theta);
		point.theta = carmen_normalize_theta(point.theta);
		point.phi = phi;
		point.v = v;
	}
	else
	{
		double temp_v = fabs(v) / radcurv;
		int direction_signal = phi >= 0 ? -1 : 1;

		double center_x = point.x + radcurv * sin(point.theta) * direction_signal;
		double center_y = point.y - radcurv * cos(point.theta) * direction_signal;
		double va1 = carmen_normalize_theta(point.theta + 1.5707963268 * direction_signal);
		double va2;

		if (v >= 0)
		{
			va2 = va1 - temp_v * direction_signal;
		}
		else
		{
			va2 = va1 + temp_v * direction_signal;
		}

		point.x = center_x + radcurv * cos(va2);
		point.y = center_y + radcurv * sin(va2);
		point.theta = point.theta - v / radcurv * direction_signal;

		point.theta = carmen_normalize_theta(point.theta);
		point.v = v;
		point.phi = phi;

	}

	return point;
}


void get_astar_path(carmen_astar_node_p node, carmen_planner_path_p path)
{
	int i;
	int x, y, theta;
	path->length = node->range;
	check_path_capacity(path);
	i = node->range - 1;  //sem o -1 pega a origem.. start e pula o destino

	while (i >= 0)
	{
		path->points[i] = node->point;
		x = get_astar_map_x(node->prev_point.x);
		y = get_astar_map_y(node->prev_point.y);
		theta = get_astar_map_theta(node->prev_point.theta);
		node =  astar_map[x][y][theta];
		i--;
	}
}


void merging_astar_path(carmen_planner_path_p path_start, carmen_planner_path_p path_goal)
{
	int i;
	for (i = 1; i <  path_goal->length; i++)
	{
		check_path_capacity(path_start);
		path_start->points[path_start->length] = path_goal->points[i];
		path_start->length++;

	}
}


int rs_get_astar_path(int rs_pathl, carmen_ackerman_traj_point_p points, carmen_planner_path_p path)
{
	int i, j;
	path->capacity = 0;
	check_path_capacity(path);
	path->length = 0;//todo verificar
	path->points[path->length].x = points[0].x;
	path->points[path->length].y = points[0].y;
	path->points[path->length].theta = points[0].theta;
	path->points[path->length].v = points[0].v;
	path->points[path->length].phi = points[0].phi;
	path->length++;
	for (i = 0; i < rs_pathl; i++)
	{
		double space_interval = astar_config.path_interval;
		int n = (fabs(points[i + 1].v)) / space_interval;
		double space_rest = fabs(points[i + 1].v) - (n * space_interval);
		double velocity_signal =  (fabs(points[i + 1].v) / points[i + 1].v);
		for (j = 0; j < n; j++)
		{
			check_path_capacity(path);
			path->points[path->length] = carmen_conventional_astar_ackerman_kinematic_3(
					path->points[path->length - 1], robot_conf_g.distance_between_front_and_rear_axles, points[i + 1].phi, space_interval * velocity_signal);
			if (is_obstacle_new(path->points[path->length]))
				return 1;
			path->length++;

		}
		check_path_capacity(path);
		path->points[path->length] = carmen_conventional_astar_ackerman_kinematic_3(
				path->points[path->length - 1], robot_conf_g.distance_between_front_and_rear_axles, points[i + 1].phi, space_rest * velocity_signal);
		if (is_obstacle_new(path->points[path->length]))
			return 1;

		path->length++;
		check_path_capacity(path);
	}
	return 0;
}


void
astar_init_parameters(carmen_ackerman_traj_point_t goal)
{

	DIRECTION[0] = astar_config.path_interval;
	DIRECTION[1] = astar_config.path_interval;
	ORIENTATION[0] = -robot_conf_g.max_phi;
	ORIENTATION[1] = 0;
	ORIENTATION[2] = robot_conf_g.max_phi;
	GOAL = goal;
	cont_nos_abertos_novos = 0;
	cont_nos_abertos_alterados = 0;
	cont_nos_podados = 0;
	cont_nos_abertos_alterados_fechados = 0;
	if (astar_config.use_rs)
		rs_init_parameters(robot_conf_g.max_phi, robot_conf_g.distance_between_front_and_rear_axles);
}


void
carmen_conventional_astar_ackerman_astar(carmen_ackerman_traj_point_t start, carmen_ackerman_traj_point_t goal, carmen_planner_path_p path)
{
	carmen_ackerman_traj_point_t rs_points[5];
	carmen_planner_path_t rs_path;
	int index;
	int rs_pathl;
	int rs_numero;
	double tr;
	double ur;
	double vr;
	double distance_2d = 0;
	double distance_theta = 0;
	double t1, t2;

	t1 = carmen_get_time();

	distance_2d = carmen_conventional_get_utility(
			round((start.x - carmen_planner_map->config.x_origin) / carmen_planner_map->config.resolution),
			round((start.y - carmen_planner_map->config.y_origin) / carmen_planner_map->config.resolution));

	distance_theta = fabs(carmen_normalize_theta(GOAL.theta - start.theta));
	if (distance_2d > (999.5))
	{
		if (distance_theta < carmen_degrees_to_radians(15))
		{
			//printf("in the GOAL\n");
			return;
		}
	}


	if (is_obstacle_new(goal))
	{
		//printf("GOAL is obstacle\n");
		return;
	}

	astar_init_parameters(goal);

	if (astar_call_cont == 0)
	{
		alloc_astar_map();
		if (current_task == BEHAVIOR_SELECTOR_PARK)
		{
			alloc_precomputed_cost_map();
			open_precomputed_cost_map();
		}
	}

	astar_call_cont++;

	heap = fh_makekeyheap();
	start.theta = carmen_normalize_theta(start.theta);
	carmen_astar_node_p node = malloc(sizeof(carmen_astar_node_t));
	node->g_score = 0;
	node->h_score = h_score(start);
	node->f_score = node->g_score + node->h_score;
	node->range = 0;
	node->prev_point = start;
	node->point = start;
	node->astar_call_cont = astar_call_cont;
	node->fh_node = NULL;
	add_list_fh(node);

	t2 = carmen_get_time();
	index = 0;
	while ((node = fh_extractmin(heap)))
	{
		if (astar_config.onroad_max_plan_time < t2 - t1 && current_task == BEHAVIOR_SELECTOR_FOLLOW_ROUTE)
		{
			node = NULL;
			break;
		}
		t2 = carmen_get_time();
		node->fh_node = NULL;
		node->status = OPEN;
		open_node(node);

		if (astar_config.use_rs)
			if (index % 1 == 0)
			{
				reed_shepp(node->point, GOAL, &rs_numero, &tr, &ur, &vr);
				rs_pathl = constRS(rs_numero, tr, ur, vr, node->point, rs_points);
				if (rs_get_astar_path(rs_pathl, rs_points, &rs_path) == 0)
					break;
			}
		index++;

		distance_2d = carmen_conventional_get_utility(
				round((node->point.x - carmen_planner_map->config.x_origin) / carmen_planner_map->config.resolution),
				round((node->point.y - carmen_planner_map->config.y_origin) / carmen_planner_map->config.resolution));

		distance_theta = fabs(carmen_normalize_theta(GOAL.theta - node->point.theta));
		if (distance_2d > (999.9 - astar_config.path_interval))
		{
			if (distance_theta < carmen_degrees_to_radians(10))
			{
				break;
			}
		}
	}
	if (node != NULL)
	{

		get_astar_path(node, path);
		if (astar_config.use_rs)
		{
			reed_shepp(node->point, GOAL, &rs_numero, &tr, &ur, &vr);
			rs_pathl = constRS(rs_numero, tr, ur, vr, node->point, rs_points);
			if (rs_get_astar_path(rs_pathl, rs_points, &rs_path) == 0)
			{
				merging_astar_path(path, &rs_path);
			}
		}
		if (astar_config.smooth_path && current_task == BEHAVIOR_SELECTOR_FOLLOW_ROUTE)
			smooth_path_astar(path);
		//print_path(path);
	}
	else
	{
		//printf("SEM CAMINHO!!\n");
	}
	//clean_astar_map();
	fh_deleteheap(heap);
	free_astar_map();

	t2 = carmen_get_time();

//	printf("i: %d  r: %d t: %.3f g: x %.2f y %.2f\n",
//		index,
//		path->length,
//		t2 - t1,
//		GOAL.x  - carmen_planner_map->config.x_origin,
//		GOAL.y  - carmen_planner_map->config.y_origin);

//	printf("iteracoes: %d  range: %d iterações/s: %.2f mil tempo: %.3f cont_nos_abertos_novos: %d cont_nos_abertos_alterados: %d  cont_nos_abertos_alterados_fechados %d cont_nos_podados: %d\n",
//		index,
//		path->length,
//		(index * 0.001) / (t2 - t1),
//		(t2 - t1),
//		cont_nos_abertos_novos,
//		cont_nos_abertos_alterados,
//		cont_nos_abertos_alterados_fechados,
//		cont_nos_podados);
	return;
}


void 
open_node(carmen_astar_node_p node)
{
	int i, j;
	double cost_weight;
	carmen_ackerman_traj_point_t new_point;
	carmen_astar_node_p new_node;

	for (i = 0; i < DIRECTION_LENGHT; i++)
	{
		for (j = 0; j < ORIENTATION_LENGHT; j++)
		{

			cost_weight = 1;
			if (current_task == BEHAVIOR_SELECTOR_FOLLOW_ROUTE)
				new_point = carmen_conventional_astar_ackerman_kinematic_3(
						node->point, robot_conf_g.distance_between_front_and_rear_axles, ORIENTATION[j], DIRECTION[i]);
			else
				new_point = carmen_conventional_astar_ackerman_kinematic_3(
						node->point, robot_conf_g.distance_between_front_and_rear_axles, ORIENTATION[j], DIRECTION[i]);

			//if (is_obstacle_1d(new_point))
			//if (is_obstacle(new_point))
			if (is_obstacle_new(new_point))

			//if (is_obstacle_cost(new_point))
				continue;


			if (fabs(node->point.phi - new_point.phi) > carmen_degrees_to_radians(10))
			{
				cost_weight += cost_weight;
				//continue;
			}


			if (DIRECTION[i] < 0) cost_weight += cost_weight;
			if (DIRECTION[i] != node->direction)
			{
				cost_weight += cost_weight;
			}

			new_node = malloc(sizeof(carmen_astar_node_t));
			new_node->h_score = h_score(new_point);
			new_node->g_score = node->g_score + astar_config.path_interval * cost_weight;
			new_node->f_score = new_node->g_score + new_node->h_score;
			new_node->direction = DIRECTION[i];
			new_node->range = node->range + 1;
			new_node->point = new_point;
			new_node->prev_point = node->point;
			new_node->astar_call_cont = astar_call_cont;
			add_list_fh(new_node);
		}
	}
}


double
h_score(carmen_ackerman_traj_point_t point)
{
	double obstacle_cost, precomputed_cost, euclidean_cost, h_score;
	int x, y, theta;

	x = round((point.x - carmen_planner_map->config.x_origin) / carmen_planner_map->config.resolution);
	y = round((point.y - carmen_planner_map->config.y_origin) / carmen_planner_map->config.resolution);

	obstacle_cost = (1000 - carmen_conventional_get_utility(x, y)) * 1.9;

	euclidean_cost = sqrt(pow(GOAL.x - point.x, 2) + pow(GOAL.y - point.y, 2));

	x = (((point.x - GOAL.x) * cos(-point.theta) - (point.y - GOAL.y) * sin(-point.theta))) / carmen_planner_map->config.resolution;
	y = (((point.x - GOAL.x) * sin(-point.theta) + (point.y - GOAL.y) * cos(-point.theta))) / carmen_planner_map->config.resolution;

	if ((x <= 0 && y >= 0) || (x >= 0 && y <= 0))
		theta = get_astar_map_theta_2(carmen_normalize_theta(-(GOAL.theta - point.theta)));
	else
		theta = get_astar_map_theta_2(carmen_normalize_theta(GOAL.theta - point.theta));

	x = abs(x);
	y = abs(y);

	if (current_task == BEHAVIOR_SELECTOR_PARK)
	{
		if (x >= astar_config.precomputed_cost_size / 2 || y >= astar_config.precomputed_cost_size / 2)
			precomputed_cost = obstacle_cost;
		else
			precomputed_cost =  precomputed_cost_map[x + astar_config.precomputed_cost_size / 2][y + astar_config.precomputed_cost_size / 2][theta] * 1;//todo antes era * 10??
	}
	else
	{
		precomputed_cost = -1;
	}
	//printf("obstacle_cost %.2f precomputed_cost %.2f x %d y %d\n",obstacle_cost,precomputed_cost,x,y);
	h_score = carmen_fmax(obstacle_cost, precomputed_cost);

	if (current_task == BEHAVIOR_SELECTOR_FOLLOW_ROUTE)
		return euclidean_cost;

	return h_score;
}


void
add_list_fh(carmen_astar_node_p new_node)
{
	int x, y, theta;
	x = get_astar_map_x(new_node->point.x);
	y = get_astar_map_y(new_node->point.y);
	theta = get_astar_map_theta(new_node->point.theta);
	if (astar_map[x][y][theta] == NULL || astar_map[x][y][theta]->f_score > new_node->f_score
			|| astar_map[x][y][theta]->astar_call_cont != astar_call_cont)
	{
		if (astar_map[x][y][theta] == NULL || astar_map[x][y][theta]->astar_call_cont != astar_call_cont)
		{
			astar_map[x][y][theta] = new_node;
			astar_map[x][y][theta]->fh_node = 	fh_insertkey(heap,
					round((astar_map[x][y][theta]->f_score / carmen_planner_map->config.resolution) * 10000),
					astar_map[x][y][theta]);
			cont_nos_abertos_novos++;
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
			cont_nos_abertos_alterados++;

			if (astar_map[x][y][theta]->status == CLOSE)
			{
				fh_replacekey(heap, astar_map[x][y][theta]->fh_node,
						round((astar_map[x][y][theta]->f_score / carmen_planner_map->config.resolution) * 10000));
				cont_nos_abertos_alterados_fechados++;
			}
			else
			{
				astar_map[x][y][theta]->fh_node = 	fh_insertkey(heap,
						round((astar_map[x][y][theta]->f_score / carmen_planner_map->config.resolution) * 10000),
						astar_map[x][y][theta]);
			}
		}
		astar_map[x][y][theta]->status = CLOSE;
	}
	else
	{
		free(new_node);
		cont_nos_podados++;
	}
}


int  is_obstacle_new(carmen_ackerman_traj_point_t point)
{
	int vertical_size   = ceil((robot_conf_g.length + astar_config.robot_fat_space) / carmen_planner_map->config.resolution);
	int horizontal_size = ceil(((robot_conf_g.width + astar_config.robot_fat_space) / carmen_planner_map->config.resolution) / 2.0);
	int distance_between_rear_car_and_rear_wheels = ceil(robot_conf_g.distance_between_rear_car_and_rear_wheels / carmen_planner_map->config.resolution);
	double obstacle_value = 0;
	int v, h, i;

	carmen_ackerman_traj_point_t vertical_pose, horizontal_pose[2];
	double delta_vertical_x, delta_vertical_y, delta_horizontal_x, delta_horizontal_y;

	point.x = (point.x - carmen_planner_map->config.x_origin) / carmen_planner_map->config.resolution;
	point.y = (point.y - carmen_planner_map->config.y_origin) / carmen_planner_map->config.resolution;

	delta_vertical_x = cos(point.theta);
	delta_vertical_y = sin(point.theta);

	delta_horizontal_x = cos(M_PI / 2.0 - point.theta);
	delta_horizontal_y = sin(M_PI / 2.0 - point.theta);

	point.x -= distance_between_rear_car_and_rear_wheels * delta_vertical_x;
	point.y -= distance_between_rear_car_and_rear_wheels * delta_vertical_y;

	vertical_pose = point;

	for (v = 0; v < vertical_size; v++)
	{
		horizontal_pose[0] = vertical_pose;
		horizontal_pose[1] = vertical_pose;

		for (h = 0; h < horizontal_size; h++)
		{
			for (i = 0; i < 2; i++)
			{
				obstacle_value = carmen_conventional_get_cost(ceil(horizontal_pose[i].x), ceil(horizontal_pose[i].y));
				if (obstacle_value > 0.9 || obstacle_value < 0)
					return TRUE;
			}

			horizontal_pose[0].x = horizontal_pose[0].x - delta_horizontal_x;
			horizontal_pose[0].y = horizontal_pose[0].y + delta_horizontal_y;

			horizontal_pose[1].x = horizontal_pose[1].x + delta_horizontal_x;
			horizontal_pose[1].y = horizontal_pose[1].y - delta_horizontal_y;
		}
		vertical_pose.x = vertical_pose.x + delta_vertical_x;
		vertical_pose.y = vertical_pose.y + delta_vertical_y;
	}
	return FALSE;
}


int
is_obstacle(carmen_ackerman_traj_point_t point)
{
	double width_m = (robot_conf_g.width + astar_config.robot_fat_space) / carmen_planner_map->config.resolution;
	double height_m = (robot_conf_g.length + astar_config.robot_fat_space) / carmen_planner_map->config.resolution;

	double x = (point.x - carmen_planner_map->config.x_origin) / carmen_planner_map->config.resolution;
	double y = (point.y - carmen_planner_map->config.y_origin) / carmen_planner_map->config.resolution;

	double current_x = x;
	double current_y = y;

	double obstacle_value;


	double delta_x, delta_y;
	double alfa = M_PI / 2 - point.theta;
	double h;
	for (h = 0; h < height_m; h = h + 0.5)
	{
		delta_x = cos(point.theta) * h;
		delta_y = sin(point.theta) * h;

		current_x = x + delta_x;
		current_y = y + delta_y;
		double w;
		for (w = 0; w < width_m / 2; w = w + 0.5)
		{
			int sinal = -1;

			delta_x = cos(alfa) * w;
			delta_y = sin(alfa) * w;
			int i;
			for (i = 0; i < 2; i++)
			{
				int aux_x, aux_y;

				aux_x = (int)(current_x + sinal * delta_x);
				sinal *= -1;
				aux_y = (int)(current_y + sinal * delta_y);

				obstacle_value = carmen_conventional_get_cost(aux_x, aux_y);

				if (obstacle_value > 0.9 || obstacle_value < 0)
					return TRUE;
			}
		}
	}
	return FALSE;
}


int
is_obstacle_1d(carmen_ackerman_traj_point_t point)
{
	double x = (point.x - carmen_planner_map->config.x_origin) / carmen_planner_map->config.resolution;
	double y = (point.y - carmen_planner_map->config.y_origin) / carmen_planner_map->config.resolution;
	double obstacle_value;
	double front = round(robot_conf_g.length / carmen_planner_map->config.resolution);
	int i;

	if (x < 0 || x >= carmen_planner_map->config.x_size || y < 0 || y >= carmen_planner_map->config.y_size)
		return TRUE;

	for (i = 0; i <= front; i++) {
		obstacle_value = carmen_conventional_get_cost(round(x + cos(point.theta) * i), round(y + sin(point.theta) * i));
		if (obstacle_value > 0.40 || obstacle_value < 0)
			return TRUE;
	}
	return FALSE;
}


int
is_obstacle_cost(carmen_ackerman_traj_point_t point)
{
	int obstacle_value = carmen_conventional_get_cost(
			(point.x - carmen_planner_map->config.x_origin) / carmen_planner_map->config.resolution,
			(point.y - carmen_planner_map->config.y_origin) / carmen_planner_map->config.resolution);
	if (obstacle_value > 0.40 || obstacle_value < 0)
		return TRUE;
	else
		return FALSE;
}


void
clean_astar_map()
{
	int i, j, k;
	int x_size = round(carmen_planner_map->config.x_size / astar_config.state_map_resolution + 1);
	int y_size = round(carmen_planner_map->config.y_size / astar_config.state_map_resolution + 1);
	int last_theta = 365 / astar_config.state_map_theta_resolution;
	for (i = 0; i < x_size; i++)
		for (j = 0; j < y_size; j++)
			for (k = 0; k < last_theta; k++)
				astar_map[i][j][k] = NULL;
}


void
free_astar_map()
{
	int i, j, k;
	int theta_size = round(365 / astar_config.state_map_theta_resolution);
	int x_size = round(carmen_planner_map->config.x_size / astar_config.state_map_resolution + 1);
	int y_size = round(carmen_planner_map->config.y_size / astar_config.state_map_resolution + 1);

	for (i = 0; i < x_size; i++)
	{
		for (j = 0; j < y_size; j++)
		{
			for (k = 0; k < theta_size; k++)
			{
				if (astar_map[i][j][k] != NULL)
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
	int x_size = round(carmen_planner_map->config.x_size / astar_config.state_map_resolution + 1);
	int y_size = round(carmen_planner_map->config.y_size / astar_config.state_map_resolution + 1);

	astar_map = (carmen_astar_node_p ***)calloc(x_size, sizeof(carmen_astar_node_p**));
	carmen_test_alloc(astar_map);
	for (i = 0; i < x_size; i++)
	{
		astar_map[i] = (carmen_astar_node_p **)calloc(y_size, sizeof(carmen_astar_node_p*));
		carmen_test_alloc(astar_map[i]);
		for (j = 0;j < y_size; j++)
		{
			astar_map[i][j] = (carmen_astar_node_p*)calloc(theta_size, sizeof(carmen_astar_node_p));
			carmen_test_alloc(astar_map[i][j]);
		}
	}
}


void
alloc_precomputed_cost_map()
{
	int i, j;
	int theta_size = round(365 / 5);
	int xy_size = round(astar_config.precomputed_cost_size);

	precomputed_cost_map = (double ***)calloc(xy_size, sizeof(double**));
	carmen_test_alloc(precomputed_cost_map);
	for (i = 0; i < xy_size; i++)
	{
		precomputed_cost_map[i] = (double **)calloc(xy_size, sizeof(double*));
		carmen_test_alloc(precomputed_cost_map[i]);
		for (j = 0; j < xy_size; j++)
		{
			precomputed_cost_map[i][j] = (double*)calloc(theta_size, sizeof(double));
			carmen_test_alloc(precomputed_cost_map[i][j]);
		}
	}
}


int
open_precomputed_cost_map()
{
	FILE *fp;
	int i, j, k, result;
	int theta_size = round(365 / 5); //todo
	int xy_size = round(astar_config.precomputed_cost_size);
	char *carmen_home = getenv("CARMEN_HOME");
	char buf[256];
	sprintf(buf, "%s/data/ackerman_precomputed_path/%s",carmen_home, astar_config.precomputed_cost_file_name);
	fp = fopen (buf, "r");

	if (fp == NULL)
	{
		printf("Houve um erro ao abrir o arquivo.\n");
		return 1;
	}
	for (i = 0; i < xy_size; i++)
	{
		for (j = 0; j < xy_size; j++)
		{
			for (k = 0; k < theta_size; k++)
			{
				result = fscanf(fp, "%lf ", &precomputed_cost_map[i][j][k]);
				if (result == EOF)
				{
					break;
				}
			}
		}
	}
	fclose (fp);
	return 0;
}


void
smooth_path_astar(carmen_planner_path_p path)
{
	int i = 1;
	while (i < path->length - 1)
	{
		if (path->points[i - 1].phi == -path->points[i].phi && path->points[i - 1].v == path->points[i].v)
		{
			carmen_planner_util_delete_path_point(i - 1, path);
			carmen_planner_util_delete_path_point(i, path);
		}
		else
		{
			i++;
		}
	}
	i = 1;
	while (i < path->length - 1)
	{
		if (path->points[i - 1].phi == path->points[i].phi && path->points[i - 1].v == path->points[i].v)
		{
			carmen_planner_util_delete_path_point(i - 1, path);
		}
		else
		{
			i++;
		}
	}
}
