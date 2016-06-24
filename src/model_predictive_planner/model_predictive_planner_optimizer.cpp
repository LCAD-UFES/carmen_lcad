/*
 * model_predictive_planner_optimizer.cpp
 *
 *  Created on: Jun 23, 2016
 *      Author: lcad
 */


#include <stdio.h>
#include <iostream>
#include <math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_multimin.h>

#include <carmen/collision_detection.h>

#include "model/robot_state.h"
#include "model/global_state.h"
#include "util.h"
#include "model_predictive_planner_optimizer.h"


bool use_obstacles = true;

TrajectoryLookupTable::CarLatencyBuffer g_car_latency_buffer_op;

TrajectoryLookupTable::TrajectoryControlParameters
fill_in_tcp(const gsl_vector *x, ObjectiveFunctionParams *params)
{
	TrajectoryLookupTable::TrajectoryControlParameters tcp;

	if (x->size == 4)
	{
		tcp.has_k3 = true;

		tcp.k1 = gsl_vector_get(x, 0);
		tcp.k2 = gsl_vector_get(x, 1);
		tcp.k3 = gsl_vector_get(x, 2);
		tcp.tt = gsl_vector_get(x, 3);
//		printf("FILL: k1: %lf k2: %lf k3: %lf \n", tcp.k1, tcp.k2, tcp.k3);
	}
	else
	{
		tcp.has_k3 = false;

		tcp.k1 = gsl_vector_get(x, 0);
		tcp.k2 = gsl_vector_get(x, 1);
		tcp.k3 = 0.0;
		tcp.tt = gsl_vector_get(x, 2);
	}
	tcp.a = params->suitable_acceleration;
	tcp.vf = params->tcp_seed->vf;
	tcp.sf = params->tcp_seed->sf;

	tcp.valid = true;

	return (tcp);
}


void
move_path_to_current_robot_pose(vector<carmen_ackerman_path_point_t> &path, Pose *localizer_pose)
{
	for (std::vector<carmen_ackerman_path_point_t>::iterator it = path.begin(); it != path.end(); ++it)
	{
		double x = localizer_pose->x + it->x * cos(localizer_pose->theta) - it->y * sin(localizer_pose->theta);
		double y = localizer_pose->y + it->x * sin(localizer_pose->theta) + it->y * cos(localizer_pose->theta);
		it->x = x;
		it->y = y;
		it->theta += localizer_pose->theta;
	}
}


double
compute_abstacles_cost(vector<carmen_ackerman_path_point_t> path)
{
	double max_obstacles_cost = 0.0;

	move_path_to_current_robot_pose(path, GlobalState::localizer_pose);
	for (unsigned int i = 0; i < path.size(); i++)
	{
		carmen_point_t robot_pose = {path[i].x, path[i].y, path[i].theta};

		double current_cost = carmen_obstacle_avoider_get_maximum_occupancy_of_map_cells_hit_by_robot_border(
				&robot_pose, &GlobalState::cost_map,
				GlobalState::robot_config.length, GlobalState::robot_config.width,
				GlobalState::robot_config.distance_between_rear_car_and_rear_wheels);
		if (current_cost > max_obstacles_cost)
			max_obstacles_cost = current_cost;
	}

	// std::cout << "max_obstacles_cost = " << max_obstacles_cost << std::endl;
	return (max_obstacles_cost * 0.08);
}


inline
double
dist(carmen_ackerman_path_point_t v, carmen_ackerman_path_point_t w)
{
	return sqrt((carmen_square(v.x - w.x) + carmen_square(v.y - w.y)));
}


inline
double
dist2(carmen_ackerman_path_point_t v, carmen_ackerman_path_point_t w)
{
	return (carmen_square(v.x - w.x) + carmen_square(v.y - w.y));
}

carmen_ackerman_path_point_t
get_the_point_nearest_to_the_trajectory(int *point_in_trajectory_is,
		carmen_ackerman_path_point_t current_robot_position,
		carmen_ackerman_path_point_t waypoint,
		carmen_ackerman_path_point_t center_of_the_car_front_axel)
{

#define	WITHIN_THE_TRAJECTORY		0
#define	CURRENT_ROBOT_POSITION		1
#define	BEFORE_CURRENT_ROBOT_POSITION	2
#define	BEYOND_WAYPOINT			3

	// Return minimum distance between line segment vw and point p
	// http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
	carmen_ackerman_path_point_t v, w, p;
	double l2, t;

	p.x = center_of_the_car_front_axel.x;
	p.y = center_of_the_car_front_axel.y;
	p.v = 0;
	p.phi = 0;
	p.theta = 0;

	v.x = current_robot_position.x;
	v.y = current_robot_position.y;
	v.v = 0;
	v.phi = 0;
	v.theta = 0;

	w.x = waypoint.x;
	w.y = waypoint.y;
	w.v = 0;
	w.phi = 0;
	w.theta = 0;

	l2 = dist2(v, w); // i.e. |w-v|^2 // NAO TROQUE POR carmen_ackerman_traj_distance2(&v, &w) pois nao sei se ee a mesma coisa.
	if (l2 < 0.1)	  // v ~== w case // @@@ Alberto: Checar isso
	{
		*point_in_trajectory_is = CURRENT_ROBOT_POSITION;
		return (v);
	}

	// Consider the line extending the segment, parameterized as v + t (w - v).
	// We find the projection of point p onto the line.
	// It falls where t = [(p-v) . (w-v)] / |w-v|^2
	t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2;

	if (t < 0.0) 	// p beyond the v end of the segment
	{
		*point_in_trajectory_is = BEFORE_CURRENT_ROBOT_POSITION;
		return (v);
	}
	if (t > 1.0)	// p beyond the w end of the segment
	{
		*point_in_trajectory_is = BEYOND_WAYPOINT;
		return (w);
	}

	// Projection falls on the segment
	p.x = v.x + t * (w.x - v.x);
	p.y = v.y + t * (w.y - v.y);
	*point_in_trajectory_is = WITHIN_THE_TRAJECTORY;

	return (p);
}

double
get_distance_between_point_to_line(carmen_ackerman_path_point_t current_robot_position,
		carmen_ackerman_path_point_t waypoint,
		carmen_ackerman_path_point_t center_of_the_car_front_axel)
{
	//https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
	double delta_x = waypoint.x - current_robot_position.x;
	double delta_y = waypoint.y - current_robot_position.y;
	double d = sqrt(delta_x * delta_x + delta_y * delta_y);
	double x2y1 =  waypoint.x * current_robot_position.y;
	double y2x1 =  waypoint.y * current_robot_position.x;

	if (d < 0.0000001)
		return dist(waypoint, center_of_the_car_front_axel);

	return abs(delta_y * center_of_the_car_front_axel.x - delta_x * center_of_the_car_front_axel.y + x2y1 - y2x1) / d;

}


void
get_points(vector<carmen_ackerman_path_point_t> &detailed_goal_list, carmen_ackerman_path_point_t &path_point, int &index_p1, int &index_p2)
{
	double max1 = DBL_MAX;
	double max2 = DBL_MAX;
	unsigned int idx1 = 0 ,  idx2 = 0;


	for (unsigned int i = 0; i < detailed_goal_list.size(); i++)
	{
		double d = dist(detailed_goal_list.at(i), path_point);
		if (max1 > d)
		{
			max1 = d;
			idx1 = i;
		}
	}
	for (unsigned int i = 0; i < detailed_goal_list.size(); i++)
	{
		double d = dist(detailed_goal_list.at(i), path_point);
		if (max2 > d && i != idx1)
		{
			max2 = d;
			idx2 = i;
		}
	}
	index_p1 = idx1;
	index_p2 = idx2;
}

//Sigmoid para dar peso para distancia
double inline
sigmoid(double x, double z)
{
	return (1/(1+exp(-x * 0.9 + z)));
}


double inline
gaussian(double x, double z)
{
	return exp((-0.3/2)*((x - z) * (x - z)));
}


/*TODO
 * Seria necessario o primeiro ponto do path (x=0 e y=0) entrar no total_distance?
 * */
double
compute_interest_dist(ObjectiveFunctionParams *my_params, vector<carmen_ackerman_path_point_t> &path)
{
	double distance = 0.0;
	double total_distance = 0.0;
	double total_points = 0.0;

	for (unsigned int i = 0; i < my_params->detailed_goal_list.size(); i += 3)
	{
		//		printf("i: %u n point size: %ld\n", i, nearest_path_point.size());

		if (my_params->nearest_path_point.at(i) < path.size())
		{
			//			printf("i: %u vector size: %ld\n", i, detailed_goal_list.size());
			distance = dist(path.at(my_params->nearest_path_point.at(i)), my_params->detailed_goal_list.at(i));
		}
		else
			distance = 0.0;

		total_distance += distance;//(distance*distance);
		total_points += 1.0;

		//	printf("Path x: %lf y: %lf \n", path.at(i).x, path.back().y);
	}
	return (total_distance / total_points);
}


carmen_ackerman_path_point_t
front_axis_coordinates(carmen_ackerman_path_point_t rear_axis_coords)
{
	carmen_ackerman_path_point_t front_axis_coords;

	double L_2 = GlobalState::robot_config.distance_between_front_and_rear_axles;

	front_axis_coords = rear_axis_coords;
	front_axis_coords.x += L_2 * cos(rear_axis_coords.theta);
	front_axis_coords.y += L_2 * sin(rear_axis_coords.theta);

	return (front_axis_coords);
}


double
compute_reference_path(ObjectiveFunctionParams *param, vector<carmen_ackerman_path_point_t> &path)
{
	double distance = 0.0;
	double min_dist;
	int index = 0;
	param->nearest_path_point.clear();
	param->path_size = path.size();
	for (unsigned int i = 0; i < param->detailed_goal_list.size(); i++)
	{
		// consider the first point as the nearest one
		min_dist = dist(param->detailed_goal_list.at(i), path.at(0));

		for (unsigned int j = index; j < path.size(); j++)
		{
			distance = dist(param->detailed_goal_list.at(i), path.at(j));

			if (distance < min_dist)
			{
				min_dist = distance;
				index = j;
			}
		}
		param->nearest_path_point.push_back(index);
	}
	return (compute_interest_dist(param, path));

}


double
compute_proximity_to_obstacles(vector<carmen_ackerman_path_point_t> path)
{
	double proximity_to_obstacles = 0.0;
	double min_dist = 2.2 / 2.0; // metade da largura do carro
	int k = 1;
	for (unsigned int i = 0; i < path.size(); i += 2)
	{
		// Move path point to map coordinates
		carmen_ackerman_path_point_t path_point_in_map_coords;
		double x_gpos = GlobalState::localizer_pose->x - GlobalState::cost_map.config.x_origin;
		double y_gpos = GlobalState::localizer_pose->y - GlobalState::cost_map.config.y_origin;
		double coss = cos(GlobalState::localizer_pose->theta);
		double sine = sin(GlobalState::localizer_pose->theta);
		double L_2 = GlobalState::robot_config.distance_between_front_and_rear_axles / 2.0;
		path_point_in_map_coords.x = x_gpos + path[i].x * coss - path[i].y * sine + L_2 * coss; // no meio do carro
		path_point_in_map_coords.y = y_gpos + path[i].x * sine + path[i].y * coss + L_2 * sine; // no meio do carro

		// Search for nearest neighbors in the map
		vector<occupied_cell> returned_occupied_cells;
		occupied_cell sought = occupied_cell(path_point_in_map_coords.x, path_point_in_map_coords.y);
		// knn search
		GlobalState::obstacles_rtree.query(bgi::nearest(sought, k), std::back_inserter(returned_occupied_cells));

		carmen_ackerman_path_point_t nearest_obstacle;
		nearest_obstacle.x = returned_occupied_cells[0].get<0>();
		nearest_obstacle.y = returned_occupied_cells[0].get<1>();

		double distance = dist(path_point_in_map_coords, nearest_obstacle);
		double delta = distance - min_dist;
		if (delta < 0.0)
			proximity_to_obstacles += delta * delta;
	}
	return (proximity_to_obstacles);
}


double
my_f(const gsl_vector *x, void *params)
{
	ObjectiveFunctionParams *my_params = (ObjectiveFunctionParams *) params;

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(x, my_params);
	TrajectoryLookupTable::TrajectoryDimensions td;

	if (tcp.tt < 0.2) // o tempo nao pode ser pequeno demais
		tcp.tt = 0.2;

	vector<carmen_ackerman_path_point_t> path = simulate_car_from_parameters(td, tcp, my_params->target_td->v_i, my_params->target_td->phi_i, g_car_latency_buffer_op, false);
	//TCP_SEED nao eh modificado pelo CG?
	my_params->tcp_seed->vf = tcp.vf;
	my_params->tcp_seed->sf = tcp.sf;

	double result = sqrt((td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index +
			(carmen_normalize_theta(td.theta) - my_params->target_td->theta) * (carmen_normalize_theta(td.theta) - my_params->target_td->theta) / (my_params->theta_by_index * 0.2) +
			(carmen_normalize_theta(td.d_yaw) - my_params->target_td->d_yaw) * (carmen_normalize_theta(td.d_yaw) - my_params->target_td->d_yaw) / (my_params->d_yaw_by_index * 0.2));

	return (result);
}


/* The gradient of f, df = (df/dx, df/dy). */
void
my_df(const gsl_vector *v, void *params, gsl_vector *df)
{
	double f_x = my_f(v, params);

	double h = 0.00001;

	gsl_vector *x_h;
	x_h = gsl_vector_alloc(3);

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0) + h);
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1));
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2));
	double f_x_h = my_f(x_h, params);
	double d_f_x_h = (f_x_h - f_x) / h;

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0));
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1) + h);
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2));
	double f_y_h = my_f(x_h, params);
	double d_f_y_h = (f_y_h - f_x) / h;

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0));
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1));
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2) + h);
	double f_z_h = my_f(x_h, params);
	double d_f_z_h = (f_z_h - f_x) / h;

	gsl_vector_set(df, 0, d_f_x_h);
	gsl_vector_set(df, 1, d_f_y_h);
	gsl_vector_set(df, 2, d_f_z_h);

	gsl_vector_free(x_h);
}


/* Compute both f and df together. */
void
my_fdf(const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
	*f = my_f(x, params);
	my_df(x, params, df);
}



//Compute cost function to optimize lane
double
my_g(const gsl_vector *x, void *params)
{
	ObjectiveFunctionParams *my_params = (ObjectiveFunctionParams *) params;

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(x, my_params);
	TrajectoryLookupTable::TrajectoryDimensions td;

	if (tcp.tt < 0.2) // o tempo nao pode ser pequeno demais
		tcp.tt = 0.2;

	vector<carmen_ackerman_path_point_t> path = simulate_car_from_parameters(td, tcp, my_params->target_td->v_i, my_params->target_td->phi_i, g_car_latency_buffer_op, false);

	double total_interest_dist = 0.0;
	if (my_params->use_lane)
	{
		if (path.size() != my_params->path_size)
		{
			//			printf("01\n");
			total_interest_dist = compute_reference_path(my_params, path);
		}
		else
		{
			//			printf("02\n");
			total_interest_dist = compute_interest_dist(my_params, path);
		}
	}

	double proximity_to_obstacles = 0.0;
	if (use_obstacles && !GlobalState::obstacles_rtree.empty())
		proximity_to_obstacles = compute_proximity_to_obstacles(path);

	my_params->tcp_seed->vf = tcp.vf;
	my_params->tcp_seed->sf = tcp.sf;

	//
	//	FILE *path_file_dist = fopen("gnu_tests/gnuplot_path_dist.txt", "w");
	//	print_lane(path,path_file_dist);
	//	fclose(path_file_dist);

	//	printf("Distancia: %lf \n" , total_interest_dist);
	//	double goal_dist = dist(path.back(), my_params->detailed_goal_list.back());
	//	goal_dist *= goal_dist;
	//	double d_yaw = carmen_normalize_theta((path.back().phi - my_params->detailed_goal_list.back().phi));
	//	double dist_2 = total_interest_dist * total_interest_dist;

	//	double result = (goal_dist*0.8) + (total_interest_dist * 0.1); //goal_dist nao tava ao quadrado
	//	double result = (sqrt(((goal_dist*0.08) + (d_yaw*0.2))*1) + (total_interest_dist * 0.05));
	//	double result = (goal_dist*0.1) + (d_yaw*0.001) + (total_interest_dist*0.01);
	//	double result = (goal_dist * 0.5) + (total_interest_dist * 0.05) + (d_yaw);
	double result = sqrt(
			1.0 * (td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index +
			5.0 * (carmen_normalize_theta(td.theta - my_params->target_td->theta) * carmen_normalize_theta(td.theta - my_params->target_td->theta)) / my_params->theta_by_index +
			5.0 * (carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw) * carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw)) / my_params->d_yaw_by_index +
			1.8 * (total_interest_dist * total_interest_dist) +
			0.2 * proximity_to_obstacles); // já é quandrática
	//	printf("Goal dist: %lf \t sem peso: %lf \n", (goal_dist*0.1),  (goal_dist));
	//	printf("total_interest: %lf \t sem peso %lf \n", (total_interest_dist * 0.1), (total_interest_dist));
	return (result);

}


/* The gradient of f, df = (df/dx, df/dy). */
void
my_dg(const gsl_vector *v, void *params, gsl_vector *df)
{
	double g_x = my_g(v, params);

	double h = 0.00001;//<<< 0.00001

	gsl_vector *x_h;
	x_h = gsl_vector_alloc(4);

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0) + h);
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1));
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2));
	gsl_vector_set(x_h, 3, gsl_vector_get(v, 3));
	double g_x_h = my_g(x_h, params);
	double d_g_x_h = (g_x_h - g_x) / h;

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0));
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1) + h);
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2));
	gsl_vector_set(x_h, 3, gsl_vector_get(v, 3));
	double g_y_h = my_g(x_h, params);
	double d_g_y_h = (g_y_h - g_x) / h;

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0));
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1));
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2) + h);
	gsl_vector_set(x_h, 3, gsl_vector_get(v, 3));
	double g_z_h = my_g(x_h, params);
	double d_g_z_h = (g_z_h - g_x) / h;

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0));
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1));
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2));
	gsl_vector_set(x_h, 3, gsl_vector_get(v, 3) + h);
	double g_w_h = my_g(x_h, params);
	double d_g_w_h = (g_w_h - g_x) / h;

	gsl_vector_set(df, 0, d_g_x_h);
	gsl_vector_set(df, 1, d_g_y_h);
	gsl_vector_set(df, 2, d_g_z_h);
	gsl_vector_set(df, 3, d_g_w_h);

	gsl_vector_free(x_h);
}


/* Compute both g and df together. for while df equal to dg */
void
my_gdf(const gsl_vector *x, void *params, double *g, gsl_vector *dg)
{
	*g = my_g(x, params);
	my_dg(x, params, dg);
}


double
compute_suitable_acceleration(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v)
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

	double a = (target_v - target_td.v_i) / tcp_seed.tt;

	if (a >= 0.0)
	{
		if (a >= 1.2)
			a = 1.2;

		return (a);
	}

	if ((-0.5 * (target_td.v_i * target_td.v_i) / a) > target_td.dist * 1.1)
	{
		if (a >= 1.2)
			a = 1.2;

		return (a);
	}
	else
	{
		while ((-0.5 * (target_td.v_i * target_td.v_i) / a) <= target_td.dist * 1.1)
			a *= 0.95;

		if (a >= 1.2)
			a = 1.2;

		return (a);
	}
}


TrajectoryLookupTable::TrajectoryControlParameters
optimized_lane_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters &tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v, ObjectiveFunctionParams params)
{
	// A f(x) muntidimensional que queremos minimizar é:
	//   f(x) = ||(car_simulator(x) + lane(x) < max_dist_lane)||
	// e as dimensões de f(x) são (dist, theta, d_yaw, phi_i, v_i, v_f)
	// O resultado ideal é zero, isto é, a saida de car_simulator deve ser igual a distancia max da lane.
	// As variáveis que serão modificadas pela minimização são:
	//   k1, k2 e tt
	// E, durante a minimização:
	//   v_0, v_i e phi_i são constantes
	//   v_0 = v_i
	//   vt, a0, af, t0, tt e sf sao dependentes das demais segundo o TrajectoryVelocityProfile

	//	vector<carmen_ackerman_path_point_t> path = simulate_car_from_parameters(target_td, tcp_seed, target_td.v_i, target_td.phi_i, g_car_latency_buffer_op, false);

	//	FILE *lane_file = fopen("gnu_tests/gnuplot_lane.txt", "w");
	//	print_lane(params.detailed_goal_list, lane_file);
	//	fclose(lane_file);
	//	char path_name[20];
	//	sprintf(path_name, "path/%d.txt", 0);
	//	FILE *path_file = fopen("gnu_tests/gnuplot_traj.txt", "w");
	//	print_lane(path,path_file);
	//	fclose(path_file);
	//	getchar();


	const gsl_multimin_fdfminimizer_type *T;
	gsl_multimin_fdfminimizer *s;
	//Testava se tinha na lane, fiz o teste antes
	//	if (params.detailed_goal_list->size() > 0)
	//		params.detailed_goal_list = detail_goal_list;
	//	else
	//		return (tcp_seed);
	//	params.use_lane = use_lane; //Jah passei para a funcao antes

	params.distance_by_index = fabs(get_distance_by_index(N_DIST-1));
	params.theta_by_index = fabs(get_theta_by_index(N_THETA-1));
	params.d_yaw_by_index = fabs(get_d_yaw_by_index(N_D_YAW-1));
	params.target_td = &target_td;
	params.tcp_seed = &tcp_seed;
	params.target_v = target_v;
	params.path_size = 0;

	gsl_vector *x;
	gsl_multimin_function_fdf my_func;

	my_func.n = 4;
	my_func.f = my_g;
	my_func.df = my_dg;
	my_func.fdf = my_gdf;
	my_func.params = &params;

	double knots_x[3] = {0.0, tcp_seed.tt / 2.0, tcp_seed.tt};
	double knots_y[3];

	if (tcp_seed.has_k3)
	{
		knots_y[0] = target_td.phi_i;
		knots_y[1] = tcp_seed.k2;
		knots_y[2] = tcp_seed.k3;

//		printf("-----------------\n k1: %lf k2: %lf k3: %lf \n", tcp_seed.k1, tcp_seed.k2, tcp_seed.k3);
	}

	else
	{
		knots_y[0] = target_td.phi_i;
		knots_y[1] = tcp_seed.k1;
		knots_y[2] = tcp_seed.k2;
	}

	gsl_interp_accel *acc = gsl_interp_accel_alloc();
	const gsl_interp_type *type = gsl_interp_cspline;
	gsl_spline *phi_spline = gsl_spline_alloc(type, 3);
	gsl_spline_init(phi_spline, knots_x, knots_y, 3);

	//	print_phi_profile_temp(phi_spline, acc, tcp.tt, display_phi_profile);

		/* Starting point, x */
	x = gsl_vector_alloc(4);
		//	gsl_vector_set(x, 0, gsl_spline_eval(phi_spline, tcp_seed.tt / 3.0, acc));
		//	gsl_vector_set(x, 1, gsl_spline_eval(phi_spline, 2.0 * (tcp_seed.tt / 3.0), acc));
	gsl_vector_set(x, 0, gsl_spline_eval(phi_spline, tcp_seed.tt / 4.0, acc));
	gsl_vector_set(x, 1, gsl_spline_eval(phi_spline, tcp_seed.tt / 2.0, acc));
	gsl_vector_set(x, 2, gsl_spline_eval(phi_spline, tcp_seed.tt, acc));
	gsl_vector_set(x, 3, tcp_seed.tt);

	gsl_spline_free(phi_spline);
	gsl_interp_accel_free(acc);

	T = gsl_multimin_fdfminimizer_conjugate_fr;
	s = gsl_multimin_fdfminimizer_alloc(T, 4);

	// int gsl_multimin_fdfminimizer_set (gsl_multimin_fdfminimizer * s, gsl_multimin_function_fdf * fdf, const gsl_vector * x, double step_size, double tol)
	gsl_multimin_fdfminimizer_set(s, &my_func, x, 0.01, 0.001);

	size_t iter = 0;
	int status;
	do
	{
		iter++;

		status = gsl_multimin_fdfminimizer_iterate(s);

		if (status == GSL_ENOPROG) // minimizer is unable to improve on its current estimate, either due to numerical difficulty or a genuine local minimum
		{
			//			printf("@@@@@@@@@@@@@@ status = %d\n", status);
			break;
		}

		// int gsl_multimin_test_gradient (const gsl_vector * g, double epsabs)
		// |g| < epsabs	} while ((s->f > MAX_LANE_DIST) && (status == GSL_CONTINUE) && (iter < 300)); //alterado de 0.005

		status = gsl_multimin_test_gradient(s->gradient, 0.16); // esta funcao retorna GSL_CONTINUE ou zero

		//	--Debug with GNUPLOT

		//		TrajectoryLookupTable::TrajectoryControlParameters tcp_temp = fill_in_tcp(s->x, &params);
		//		char path_name[20];
		//		sprintf(path_name, "path/%lu.txt", iter);
		//		FILE *path_file = fopen("gnu_tests/gnuplot_traj.txt", "w");
		//		print_lane(simulate_car_from_parameters(target_td, tcp_temp, target_td.v_i, target_td.phi_i, g_car_latency_buffer_op, true), path_file);
		//		fclose(path_file);
		//		printf("Estou na: %lu iteracao, sf: %lf  \n", iter, s->f);
		//		getchar();
		//	--

	} while (/*(s->f > MAX_LANE_DIST) &&*/ (status == GSL_CONTINUE) && (iter < 300)); //alterado de 0.005

//	printf("Parei em: %lu iteracoes, sf: %lf  \n", iter, s->f);
	//	getchar();

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(s->x, &params);

	//	//TODO Verificar esse teste para a lane
	if ((tcp.tt < 0.2)/* || (s->f > 0.75)*/) // too short plan or bad minimum (s->f should be close to zero)
		tcp.valid = false;

	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

	return (tcp);
}


// TODO optimizer
TrajectoryLookupTable::TrajectoryControlParameters
get_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v, ObjectiveFunctionParams &params,
		bool has_previous_good_tcp)
{
	// A f(x) muntidimensional que queremos minimizar é:
	//   f(x) = ||(car_simulator(x) - target_td, vf - target_v)||
	// e as dimensões de f(x) são (dist, theta, d_yaw, phi_i, v_i, v_f)
	// O resultado ideal é zero, isto é, a saida de car_simulator deve ser igual a target_td e vf = target_v.
	// As variáveis que serão modificadas pela minimização são:
	//   k1, k2 e tt
	// E, durante a minimização:
	//   v_0, v_i e phi_i são constantes
	//   v_0 = v_i
	//   vt, a0, af, t0, tt e sf sao dependentes das demais segundo o TrajectoryVelocityProfile


	const gsl_multimin_fdfminimizer_type *T;
	gsl_multimin_fdfminimizer *s;

	double suitable_acceleration = compute_suitable_acceleration(tcp_seed, target_td, target_v);

	//	double par[17] = {0 target_td.v_i, 1 target_td.phi_i, 2 - target_td.dist, 3 - target_td.theta, 4 - target_td.d_yaw,
	//			5 - suitable_acceleration, 6 - tcp_seed.af, 7 - tcp_seed.t0, 8 - tcp_seed.tt, 9 - tcp_seed.vt, 10 - target_v,
	//			11 - (double) ((int) tcp_seed.velocity_profile), 12 - tcp_seed.vf, 13 - tcp_seed.sf,
	//			14 - fabs(get_distance_by_index(N_DIST-1)),
	//			15 - fabs(get_theta_by_index(N_THETA-1)), 16 - fabs(get_d_yaw_by_index(N_D_YAW-1))}

	params.distance_by_index = fabs(get_distance_by_index(N_DIST-1));
	params.theta_by_index = fabs(get_theta_by_index(N_THETA-1));
	params.d_yaw_by_index = fabs(get_d_yaw_by_index(N_D_YAW-1));
	params.suitable_acceleration = suitable_acceleration;
	params.target_td = &target_td;
	params.tcp_seed = &tcp_seed;
	params.target_v = target_v;

	if (has_previous_good_tcp)
		return (tcp_seed);

	gsl_vector *x;
	gsl_multimin_function_fdf my_func;

	my_func.n = 3;
	my_func.f = my_f;
	my_func.df = my_df;
	my_func.fdf = my_fdf;
	my_func.params = &params;

	/* Starting point, x */
	x = gsl_vector_alloc(3);
	gsl_vector_set(x, 0, tcp_seed.k1);
	gsl_vector_set(x, 1, tcp_seed.k2);
	gsl_vector_set(x, 2, tcp_seed.tt);

	T = gsl_multimin_fdfminimizer_conjugate_fr;
	s = gsl_multimin_fdfminimizer_alloc(T, 3);

	// int gsl_multimin_fdfminimizer_set (gsl_multimin_fdfminimizer * s, gsl_multimin_function_fdf * fdf, const gsl_vector * x, double step_size, double tol)
	gsl_multimin_fdfminimizer_set(s, &my_func, x, 0.0001, 0.001);

	size_t iter = 0;
	int status;
	do
	{
		iter++;

		status = gsl_multimin_fdfminimizer_iterate(s);


		if (status == GSL_ENOPROG) // minimizer is unable to improve on its current estimate, either due to numerical difficulty or a genuine local minimum
		{
			//printf("@@@@@@@@@@@@@@ status = %d\n", status);
			break;
		}

		// int gsl_multimin_test_gradient (const gsl_vector * g, double epsabs)
		// |g| < epsabs
		status = gsl_multimin_test_gradient(s->gradient, 0.16); // esta funcao retorna GSL_CONTINUE ou zero

	} while ((s->f > 0.005) && (status == GSL_CONTINUE) && (iter < 300)); //alterado de 0.005

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(s->x, &params);

	if ((tcp.tt < 0.2) || (s->f > 0.05)) // too short plan or bad minimum (s->f should be close to zero) mudei de 0.05 para outro
		tcp.valid = false;

	if (target_td.dist < 3.0 && tcp.valid == false) // para debugar
		tcp.valid = false;


	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

	//	if (tcp.valid)
	//	{
	//		print_path(simulate_car_from_parameters(target_td, tcp, target_td.v_i, target_td.phi_i, g_car_latency_buffer_op, false));
	//		print_path(params.detailed_goal_list);
	//		FILE *gnuplot_pipe = popen("gnuplot -persist", "w");
	//		fprintf(gnuplot_pipe, "set xrange [-15:45]\nset yrange [-15:15]\n"
	//				"plot './gnuplot_path.txt' using 1:2:3:4 w vec size  0.3, 10 filled\n");
	//		fflush(gnuplot_pipe);
	//		pclose(gnuplot_pipe);
	//		getchar();
	//		system("pkill gnuplot");
	//	}
	//	printf("Iteracoes: %lu \n", iter);
	return (tcp);
}

//TrajectoryLookupTable::TrajectoryControlParameters
//get_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd,
//        TrajectoryLookupTable::TrajectoryDimensions td)
//{
//    TrajectoryLookupTable::TrajectoryControlParameters tcp = td.control_parameters;
//
//    return (tcp);
//}


TrajectoryLookupTable::TrajectoryControlParameters
get_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
		TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd, double target_v)
{
	ObjectiveFunctionParams params;

	TrajectoryLookupTable::TrajectoryDimensions target_td = convert_to_trajectory_dimensions(tdd, tcp_seed);
	TrajectoryLookupTable::TrajectoryControlParameters tcp = get_optimized_trajectory_control_parameters(tcp_seed, target_td, target_v, params, false);

	return (tcp);
}

TrajectoryLookupTable::TrajectoryControlParameters
get_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
		TrajectoryLookupTable::TrajectoryDiscreteDimensions &tdd, double target_v, vector<carmen_ackerman_path_point_t> optimized_path)
{
	ObjectiveFunctionParams params;

	TrajectoryLookupTable::TrajectoryDimensions target_td = convert_to_trajectory_dimensions(tdd, tcp_seed);
	TrajectoryLookupTable::TrajectoryControlParameters tcp = get_optimized_trajectory_control_parameters(tcp_seed, target_td, target_v, params, false);
	if (tcp.valid)
	{
		TrajectoryLookupTable::TrajectoryDimensions td;
		optimized_path = simulate_car_from_parameters(td, tcp, target_td.v_i, target_td.phi_i, g_car_latency_buffer_op, false);
		tdd = get_discrete_dimensions(td);
	}

	return (tcp);
}

TrajectoryLookupTable::TrajectoryControlParameters
get_complete_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v, vector<carmen_ackerman_path_point_t> detailed_goal_list,
		bool use_lane, bool has_previous_good_tcp)
{
	TrajectoryLookupTable::TrajectoryControlParameters tcp_complete;
	ObjectiveFunctionParams params;
	params.detailed_goal_list = detailed_goal_list;
	params.use_lane = use_lane;

	tcp_complete = get_optimized_trajectory_control_parameters(tcp_seed, target_td, target_v, params, has_previous_good_tcp);

	if (tcp_complete.valid && params.detailed_goal_list.size() > 0)
		tcp_complete = optimized_lane_trajectory_control_parameters(tcp_complete, target_td, target_v, params);

	return (tcp_complete);
}

