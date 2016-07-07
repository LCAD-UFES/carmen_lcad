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
		tcp.has_k1 = true;

		tcp.k1 = gsl_vector_get(x, 0);
		tcp.k2 = gsl_vector_get(x, 1);
		tcp.k3 = gsl_vector_get(x, 2);
		tcp.tt = gsl_vector_get(x, 3);
	}
	else
	{
		tcp.has_k1 = false;

		tcp.k2 = gsl_vector_get(x, 0);
		tcp.k3 = gsl_vector_get(x, 1);
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
		it->theta = carmen_normalize_theta(it->theta + localizer_pose->theta);
	}
}


double
compute_distance_by_mask(carmen_ackerman_path_point_t path_point_in_map_coords, int theta)
{
	double min_distance = DBL_MAX;
	double distance;

	for(unsigned int i = 0; i < GlobalState::cell_mask.at(theta).size(); i++)
	{
		double mask_to_map_x = path_point_in_map_coords.x + GlobalState::cell_mask[theta].at(i).x;
		double mask_to_map_y = path_point_in_map_coords.y + GlobalState::cell_mask[theta].at(i).y;

		if ((round(mask_to_map_x) >= 0.0 && round(mask_to_map_y) < GlobalState::cost_map.config.x_size &&
				round(mask_to_map_y) >= 0.0 && round(mask_to_map_y) < GlobalState::cost_map.config.y_size))
		{
			double value = GlobalState::cost_map.map[(int) round(mask_to_map_x)][(int) round(mask_to_map_y)];

			if(value > 0.5)
			{
				distance = sqrt(pow(path_point_in_map_coords.x - mask_to_map_x, 2) + (pow(path_point_in_map_coords.y - mask_to_map_y, 2)));

				if(min_distance > distance)
					min_distance = distance;
			}

		}
	}

//	int x = carmen_grid_mapping_save_map("map.map", &GlobalState::cost_map);
//	if(x)
//		printf("Mapa criado");
	return min_distance;
}


double
compute_proximity_to_obstacles_mask(vector<carmen_ackerman_path_point_t> path)
{
	double proximity_to_obstacles = 0.0;
	double min_dist = 2.5 / 2.0; // metade da largura do carro
//    int k = 1;
    for (unsigned int i = 0; i < path.size(); i += 4)
    {
    	// Move path point to map coordinates
    	carmen_ackerman_path_point_t path_point_in_map_coords;
    	double x_gpos = GlobalState::localizer_pose->x - GlobalState::cost_map.config.x_origin;
    	double y_gpos = GlobalState::localizer_pose->y - GlobalState::cost_map.config.y_origin;
    	path_point_in_map_coords.x = x_gpos + path[i].x * cos(GlobalState::localizer_pose->theta) - path[i].y * sin(GlobalState::localizer_pose->theta);
    	path_point_in_map_coords.y = y_gpos + path[i].x * sin(GlobalState::localizer_pose->theta) + path[i].y * cos(GlobalState::localizer_pose->theta);

    	double distance = compute_distance_by_mask(path_point_in_map_coords, 0);
	    double delta = distance - min_dist;
	    if (delta < 0.0)
	    	proximity_to_obstacles += delta*delta;
    }
//    printf("FOI!");
//    getchar();
    return (proximity_to_obstacles);
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


void
get_points(vector<carmen_ackerman_path_point_t> &detailed_lane, carmen_ackerman_path_point_t &path_point, int &index_p1, int &index_p2)
{
	double max1 = DBL_MAX;
	double max2 = DBL_MAX;
	unsigned int idx1 = 0 ,  idx2 = 0;


	for (unsigned int i = 0; i < detailed_lane.size(); i++)
	{
		double d = dist(detailed_lane.at(i), path_point);
		if (max1 > d)
		{
			max1 = d;
			idx1 = i;
		}
	}
	for (unsigned int i = 0; i < detailed_lane.size(); i++)
	{
		double d = dist(detailed_lane.at(i), path_point);
		if (max2 > d && i != idx1)
		{
			max2 = d;
			idx2 = i;
		}
	}
	index_p1 = idx1;
	index_p2 = idx2;
}


double
compute_path_to_lane_distance(ObjectiveFunctionParams *my_params, vector<carmen_ackerman_path_point_t> &path)
{
	double distance = 0.0;
	double total_distance = 0.0;
	double total_points = 0.0;

	for (unsigned int i = 0; i < my_params->detailed_lane.size(); i += 3)
	{
		if (my_params->path_point_nearest_to_lane.at(i) < path.size())
			distance = dist(path.at(my_params->path_point_nearest_to_lane.at(i)), my_params->detailed_lane.at(i));
		else
			distance = 0.0;

		total_distance += distance * distance;
		total_points += 1.0;
	}
	return (total_distance / total_points);
}


void
compute_path_points_nearest_to_lane(ObjectiveFunctionParams *param, vector<carmen_ackerman_path_point_t> &path)
{
	double distance = 0.0;
	double min_dist;
	int index = 0;
	param->path_point_nearest_to_lane.clear();
	param->path_size = path.size();
	for (unsigned int i = 0; i < param->detailed_lane.size(); i++)
	{
		// consider the first point as the nearest one
		min_dist = dist(param->detailed_lane.at(i), path.at(0));

		for (unsigned int j = index; j < path.size(); j++)
		{
			distance = dist(param->detailed_lane.at(i), path.at(j));

			if (distance < min_dist)
			{
				min_dist = distance;
				index = j;
			}
		}
		param->path_point_nearest_to_lane.push_back(index);
	}
}


double
compute_proximity_to_obstacles(vector<carmen_ackerman_path_point_t> path)
{
	double proximity_to_obstacles = 0.0;
	double min_dist = 2.2 / 2.0; // metade da largura do carro
	int k = 1;
	for (unsigned int i = 0; i < path.size(); i += 1)
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
//		printf("Coordenada no do robô x: %lf y: %lf \n", path_point_in_map_coords.x, path_point_in_map_coords.y);
//		getchar();
		double distance = dist(path_point_in_map_coords, nearest_obstacle);
		double delta = distance - min_dist;
		if (delta < 0.0)
			proximity_to_obstacles += delta * delta;
	}
	return (proximity_to_obstacles);
}


//KD-TREE
double
compute_proximity_to_obstacles_kdtree(vector<carmen_ackerman_path_point_t> path)
{
	double proximity_to_obstacles = 0.0;
	double min_dist = 2.2 / 2.0; // metade da largura do carro
	unsigned int i;

	for (i = 0; i < path.size(); i += 2)
	{
//		double x = GlobalState::localizer_pose->x + path[i].x * cos(GlobalState::localizer_pose->theta) - path[i].y * sin(GlobalState::localizer_pose->theta);
//		double y = GlobalState::localizer_pose->y + path[i].x * sin(GlobalState::localizer_pose->theta) + path[i].y * cos(GlobalState::localizer_pose->theta);
//		Point2D path_point;
//		path_point.position[0] = x - GlobalState::cost_map.config.x_origin;
//		path_point.position[1] = y - GlobalState::cost_map.config.y_origin;

		// Move path point to map coordinates
		Point2D path_point_in_map_coords;
		double x_gpos = GlobalState::localizer_pose->x - GlobalState::cost_map.config.x_origin;
		double y_gpos = GlobalState::localizer_pose->y - GlobalState::cost_map.config.y_origin;
		double coss = cos(GlobalState::localizer_pose->theta);
		double sine = sin(GlobalState::localizer_pose->theta);
		double L_2 = GlobalState::robot_config.distance_between_front_and_rear_axles / 2.0;
		path_point_in_map_coords.position[0] = x_gpos + path[i].x * coss - path[i].y * sine + L_2 * coss; // no meio do carro
		path_point_in_map_coords.position[1] = y_gpos + path[i].x * sine + path[i].y * coss + L_2 * sine; // no meio do carro

		//Get the nearest point
		Point2D nearest_obstacle = GlobalState::obstacles_kdtree.nearest(path_point_in_map_coords);

		double distance = std::sqrt(pow(nearest_obstacle.position[0] - path_point_in_map_coords.position[0], 2) + pow(nearest_obstacle.position[1] - path_point_in_map_coords.position[1], 2));

		double delta = distance - min_dist;

		if (delta < 0.0)
			proximity_to_obstacles += delta * delta;
	}
	return (proximity_to_obstacles);

}


double
//distance_from_traj_point_to_obstacle(carmen_ackerman_path_point_t point, double x_gpos, double y_gpos, double displacement, FILE *plot)
distance_from_traj_point_to_obstacle(carmen_ackerman_path_point_t point, double x_gpos, double y_gpos, double displacement)
{
	// Move path point to map coordinates
	carmen_ackerman_path_point_t path_point_in_map_coords;
	double x = point.x;
	double y = point.y;

	double coss = cos(point.theta);
	double sine = sin(point.theta);
	double x_disp = x + displacement * coss;
	double y_disp = y + displacement * sine;

	coss = cos(GlobalState::localizer_pose->theta);
	sine = sin(GlobalState::localizer_pose->theta);
	path_point_in_map_coords.x = (x_gpos + x_disp * coss - y_disp * sine) / GlobalState::cost_map.config.resolution; // no meio do carro
	path_point_in_map_coords.y = (y_gpos + x_disp * sine + y_disp * coss) / GlobalState::cost_map.config.resolution; // no meio do carro

	int x_map_cell = (int) round(path_point_in_map_coords.x);
	int y_map_cell = (int) round(path_point_in_map_coords.y);

	// Os mapas de carmen sao orientados a colunas, logo a equacao eh como abaixo
	int index = y_map_cell + GlobalState::localize_map->config.y_size * x_map_cell;
	carmen_ackerman_path_point_t nearest_obstacle;
	nearest_obstacle.x = (double) GlobalState::localize_map->complete_x_offset[index] + (double) x_map_cell;
	nearest_obstacle.y = (double) GlobalState::localize_map->complete_y_offset[index] + (double) y_map_cell;

//	fprintf(plot, "%lf %lf red\n", path_point_in_map_coords.x, path_point_in_map_coords.y);
//	fprintf(plot, "%lf %lf green\n", nearest_obstacle.x, nearest_obstacle.y);

	double distance_in_map_coordinates = dist(path_point_in_map_coords, nearest_obstacle);
	double distance = distance_in_map_coordinates * GlobalState::cost_map.config.resolution;

	return (distance);
}


double
compute_proximity_to_obstacles_using_localize_map(vector<carmen_ackerman_path_point_t> path)
{
//	FILE *plot;
//
//	plot = fopen("Data.csv", "w");

	double proximity_to_obstacles = 0.0;
	double min_dist = (GlobalState::robot_config.width + 1.6) / 2.0; // metade da largura do carro + um espacco de guarda
	double x_gpos = GlobalState::localizer_pose->x - GlobalState::cost_map.config.x_origin;
	double y_gpos = GlobalState::localizer_pose->y - GlobalState::cost_map.config.y_origin;
	for (unsigned int i = 0; i < path.size(); i += 1)
	{
		double displacement = -GlobalState::robot_config.distance_between_rear_car_and_rear_wheels;
//		double distance = distance_from_traj_point_to_obstacle(path[i], x_gpos, y_gpos, displacement, plot);
		double distance = distance_from_traj_point_to_obstacle(path[i], x_gpos, y_gpos, displacement);
		double delta = distance - min_dist;
		if (delta < 0.0)
			proximity_to_obstacles += delta * delta;

		displacement = 0.0;//-GlobalState::robot_config.distance_between_rear_car_and_rear_wheels;
//		double distance = distance_from_traj_point_to_obstacle(path[i], x_gpos, y_gpos, displacement, plot);
		distance = distance_from_traj_point_to_obstacle(path[i], x_gpos, y_gpos, displacement);
		delta = distance - min_dist;
		if (delta < 0.0)
			proximity_to_obstacles += delta * delta;

		displacement = GlobalState::robot_config.distance_between_front_and_rear_axles / 2.0;
//		distance = distance_from_traj_point_to_obstacle(path[i], x_gpos, y_gpos, displacement, plot);
		distance = distance_from_traj_point_to_obstacle(path[i], x_gpos, y_gpos, displacement);
		delta = distance - min_dist;
		if (delta < 0.0)
			proximity_to_obstacles += delta * delta;

		displacement = GlobalState::robot_config.distance_between_front_and_rear_axles;// + GlobalState::robot_config.distance_between_front_car_and_front_wheels;
//		distance = distance_from_traj_point_to_obstacle(path[i], x_gpos, y_gpos, displacement, plot);
		distance = distance_from_traj_point_to_obstacle(path[i], x_gpos, y_gpos, displacement);
		delta = distance - min_dist;
		if (delta < 0.0)
			proximity_to_obstacles += delta * delta;

		displacement = GlobalState::robot_config.distance_between_front_and_rear_axles + GlobalState::robot_config.distance_between_front_car_and_front_wheels;
//		distance = distance_from_traj_point_to_obstacle(path[i], x_gpos, y_gpos, displacement, plot);
		distance = distance_from_traj_point_to_obstacle(path[i], x_gpos, y_gpos, displacement);
		delta = distance - min_dist;
		if (delta < 0.0)
			proximity_to_obstacles += delta * delta;
	}
//	fflush(plot);
//	fclose(plot);
//	printf("po %lf\n", proximity_to_obstacles);
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

	double path_to_lane_distance = 0.0;
	if (my_params->use_lane && (my_params->detailed_lane.size() > 0))
	{
		if (path.size() != my_params->path_size)
		{
			compute_path_points_nearest_to_lane(my_params, path);
			path_to_lane_distance = compute_path_to_lane_distance(my_params, path);
		}
		else
			path_to_lane_distance = compute_path_to_lane_distance(my_params, path);
	}

	double proximity_to_obstacles = 0.0;

//	if (use_obstacles && !GlobalState::obstacles_rtree.empty())
//		proximity_to_obstacles = compute_proximity_to_obstacles(path);
	if (use_obstacles && GlobalState::localize_map != NULL)
		proximity_to_obstacles = compute_proximity_to_obstacles_using_localize_map(path);
//	if (use_obstacles && !GlobalState::obstacles_kdtree.empty())
//		proximity_to_obstacles = compute_proximity_to_obstacles_kdtree(path);

	my_params->tcp_seed->vf = tcp.vf;
	my_params->tcp_seed->sf = tcp.sf;

	double result = sqrt(
			5.0 * (td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index +
			15.0 * (carmen_normalize_theta(td.theta - my_params->target_td->theta) * carmen_normalize_theta(td.theta - my_params->target_td->theta)) / my_params->theta_by_index +
			15.0 * (carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw) * carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw)) / my_params->d_yaw_by_index +
			1.5 * path_to_lane_distance + // já é quandrática
			10.0 * proximity_to_obstacles); // já é quandrática
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
compute_suitable_acceleration(double tt, TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v)
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

	if (target_v < 0.0)
		target_v = 0.0;

	double a = (target_v - target_td.v_i) / tt;

	if (a > 0.0)
	{
		if (a >= GlobalState::robot_config.maximum_acceleration_forward)
			a = GlobalState::robot_config.maximum_acceleration_forward;

		return (a);
	}
//	else if ((target_td.v_i * PROFILE_TIME) < 2.0 * target_td.dist)
//	{
//		a = (2.0 * (target_td.dist - target_td.v_i * PROFILE_TIME)) / (PROFILE_TIME * PROFILE_TIME);
//
//		if (a >= GlobalState::robot_config.maximum_acceleration_forward)
//			a = GlobalState::robot_config.maximum_acceleration_forward;
//
//		return (a);
//	}
//	else
//	{
//		a = -target_td.v_i / PROFILE_TIME;
//		if (a < -2.7)
//			a = -2.7;
//		return (a);
//	}

	if ((-0.5 * (target_td.v_i * target_td.v_i) / a) > target_td.dist * 1.1)
	{
		return (a);
	}
	else
	{
		while ((-0.5 * (target_td.v_i * target_td.v_i) / a) <= target_td.dist * 1.1)
			a *= 0.95;

		return (a);
	}
}


void
get_optimization_params(double target_v,
		TrajectoryLookupTable::TrajectoryControlParameters &tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions &target_td,
		ObjectiveFunctionParams &params)
{
	params.distance_by_index = fabs(get_distance_by_index(N_DIST - 1));
	params.theta_by_index = fabs(get_theta_by_index(N_THETA - 1));
	params.d_yaw_by_index = fabs(get_d_yaw_by_index(N_D_YAW - 1));
	params.target_td = &target_td;
	params.tcp_seed = &tcp_seed;
	params.target_v = target_v;
	params.path_size = 0;
}


void
get_missing_k1(const TrajectoryLookupTable::TrajectoryDimensions& target_td,
		TrajectoryLookupTable::TrajectoryControlParameters& tcp_seed)
{
	double knots_x[3] = { 0.0, tcp_seed.tt / 2.0, tcp_seed.tt };
	double knots_y[3];
	knots_y[0] = target_td.phi_i;
	knots_y[1] = tcp_seed.k2;
	knots_y[2] = tcp_seed.k3;
	gsl_interp_accel* acc = gsl_interp_accel_alloc();
	const gsl_interp_type* type = gsl_interp_cspline;
	gsl_spline* phi_spline = gsl_spline_alloc(type, 3);
	gsl_spline_init(phi_spline, knots_x, knots_y, 3);
	tcp_seed.k1 = gsl_spline_eval(phi_spline, tcp_seed.tt / 4.0, acc);
	tcp_seed.has_k1 = true;
	gsl_spline_free(phi_spline);
	gsl_interp_accel_free(acc);
}


TrajectoryLookupTable::TrajectoryControlParameters
optimized_lane_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters &tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v, ObjectiveFunctionParams params)
{
	//	vector<carmen_ackerman_path_point_t> path = simulate_car_from_parameters(target_td, tcp_seed, target_td.v_i, target_td.phi_i, g_car_latency_buffer_op, false);

	//	FILE *lane_file = fopen("gnu_tests/gnuplot_lane.txt", "w");
	//	print_lane(params.detailed_lane, lane_file);
	//	fclose(lane_file);
	//	char path_name[20];
	//	sprintf(path_name, "path/%d.txt", 0);
	//	FILE *path_file = fopen("gnu_tests/gnuplot_traj.txt", "w");
	//	print_lane(path,path_file);
	//	fclose(path_file);
	//	getchar();

	get_optimization_params(target_v, tcp_seed, target_td, params);

	gsl_multimin_function_fdf my_func;

	my_func.n = 4;
	my_func.f = my_g;
	my_func.df = my_dg;
	my_func.fdf = my_gdf;
	my_func.params = &params;

	if (!tcp_seed.has_k1)
		get_missing_k1(target_td, tcp_seed);

	/* Starting point, x */
	gsl_vector *x = gsl_vector_alloc(4);
	gsl_vector_set(x, 0, tcp_seed.k1);
	gsl_vector_set(x, 1, tcp_seed.k2);
	gsl_vector_set(x, 2, tcp_seed.k3);
	gsl_vector_set(x, 3, tcp_seed.tt);

	const gsl_multimin_fdfminimizer_type *T = gsl_multimin_fdfminimizer_vector_bfgs2;
	gsl_multimin_fdfminimizer *s = gsl_multimin_fdfminimizer_alloc(T, 4);

	// int gsl_multimin_fdfminimizer_set (gsl_multimin_fdfminimizer * s, gsl_multimin_function_fdf * fdf, const gsl_vector * x, double step_size, double tol)
	gsl_multimin_fdfminimizer_set(s, &my_func, x, 0.0001, 0.01);

	size_t iter = 0;
	int status;
	do
	{
		iter++;

		status = gsl_multimin_fdfminimizer_iterate(s);

		if (status == GSL_ENOPROG) // minimizer is unable to improve on its current estimate, either due to numerical difficulty or a genuine local minimum
			break;

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
//		params.suitable_acceleration = compute_suitable_acceleration(gsl_vector_get(x, 3), target_td, target_v);

	} while (/*(s->f > MAX_LANE_DIST) &&*/ (status == GSL_CONTINUE) && (iter < 25)); //alterado de 0.005

//	printf("iter = %ld\n", iter);

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(s->x, &params);

	if ((tcp.tt < 0.2)/* || (s->f > 0.75)*/) // too short plan or bad minimum (s->f should be close to zero)
		tcp.valid = false;

	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

	return (tcp);
}


TrajectoryLookupTable::TrajectoryControlParameters
get_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v, ObjectiveFunctionParams &params,
		bool has_previous_good_tcp)
{
	get_optimization_params(target_v, tcp_seed, target_td, params);
	params.suitable_acceleration = compute_suitable_acceleration(tcp_seed.tt, target_td, target_v);

	if (has_previous_good_tcp)
		return (tcp_seed);

//	printf("no previous_good_tcp\n");

	gsl_multimin_function_fdf my_func;

	my_func.n = 3;
	my_func.f = my_f;
	my_func.df = my_df;
	my_func.fdf = my_fdf;
	my_func.params = &params;

	/* Starting point, x */
	gsl_vector *x = gsl_vector_alloc(3);
	gsl_vector_set(x, 0, tcp_seed.k2);
	gsl_vector_set(x, 1, tcp_seed.k3);
	gsl_vector_set(x, 2, tcp_seed.tt);

	const gsl_multimin_fdfminimizer_type *T = gsl_multimin_fdfminimizer_vector_bfgs2;
	gsl_multimin_fdfminimizer *s = gsl_multimin_fdfminimizer_alloc(T, 3);

	gsl_multimin_fdfminimizer_set(s, &my_func, x, 0.0001, 0.01);

	size_t iter = 0;
	int status;
	do
	{
		iter++;

		status = gsl_multimin_fdfminimizer_iterate(s);
		if (status == GSL_ENOPROG) // minimizer is unable to improve on its current estimate, either due to numerical difficulty or a genuine local minimum
			break;

		status = gsl_multimin_test_gradient(s->gradient, 0.16); // esta funcao retorna GSL_CONTINUE ou zero
//		params.suitable_acceleration = compute_suitable_acceleration(gsl_vector_get(x, 2), target_td, target_v);

	} while ((s->f > 0.005) && (status == GSL_CONTINUE) && (iter < 30)); //alterado de 0.005

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
	//		print_path(params.detailed_lane);
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
get_complete_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v, vector<carmen_ackerman_path_point_t> detailed_lane,
		bool use_lane, bool has_previous_good_tcp)
{
	TrajectoryLookupTable::TrajectoryControlParameters tcp_complete;
	ObjectiveFunctionParams params;
	params.detailed_lane = detailed_lane;
	params.use_lane = use_lane;

	tcp_complete = get_optimized_trajectory_control_parameters(tcp_seed, target_td, target_v, params, has_previous_good_tcp);

	// Atencao: params.suitable_acceleration deve ser preenchido na funcao acima para que nao seja alterado no inicio da otimizacao abaixo
	if (tcp_complete.valid)
		tcp_complete = optimized_lane_trajectory_control_parameters(tcp_complete, target_td, target_v, params);

	return (tcp_complete);
}
