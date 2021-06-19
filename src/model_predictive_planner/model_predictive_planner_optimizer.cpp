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
#include <carmen/carmen.h>

#include "robot_state.h"
#include "model/global_state.h"
#include "util.h"
#include "model_predictive_planner_optimizer.h"

#define G_STEP_SIZE	0.001
#define F_STEP_SIZE	G_STEP_SIZE
#define H_STEP_SIZE	G_STEP_SIZE

#define G_TOL		0.01
#define F_TOL		G_TOL
#define H_TOL		G_TOL

#define G_EPSABS	0.016
#define F_EPSABS	G_EPSABS
#define H_EPSABS	G_EPSABS

bool use_obstacles = true;

//extern carmen_mapper_virtual_laser_message virtual_laser_message;

extern int use_unity_simulator;


bool
bad_tcp(TrajectoryLookupTable::TrajectoryControlParameters tcp)
{
	if (isnan(tcp.tt) || isnan(tcp.a) || isnan(tcp.s))
		return (true);
	else
		return (false);
}


void
compute_a_and_t_from_s_reverse(double s, double target_v,
		TrajectoryLookupTable::TrajectoryDimensions target_td,
		TrajectoryLookupTable::TrajectoryControlParameters &tcp_seed,
		ObjectiveFunctionParams *params)
{
	// https://www.wolframalpha.com/input/?i=solve+s%3Dv*x%2B0.5*a*x%5E2
	double a = (target_v * target_v - target_td.v_i * target_td.v_i) / (2.0 * s);
	if (a == 0.0)
		a = 0.000001;
	a = (-1)*a;
	tcp_seed.tt = (target_v - target_td.v_i) / a;
	if (a < -GlobalState::robot_config.maximum_acceleration_reverse)
	{
		a = GlobalState::robot_config.maximum_acceleration_reverse;
		double v = fabs(target_td.v_i);
		tcp_seed.tt = (sqrt(fabs(2.0 * a * s + v * v) + v)) / a;
		a = (-1)*a;

	}
	else if (a > GlobalState::robot_config.maximum_deceleration_reverse)
	{
		a = GlobalState::robot_config.maximum_deceleration_reverse;
		double v = fabs(target_td.v_i);
		tcp_seed.tt = (sqrt(fabs(2.0 * a * s + v * v)) + v) / a;
	}
//	else if (a == 0.0 && target_td.v_i != 0.0)
//		tcp_seed.tt = s/target_td.v_i;

	if (tcp_seed.tt > 200.0)
		tcp_seed.tt = 200.0;
	if (tcp_seed.tt < 0.05)
		tcp_seed.tt = 0.05;

//	if (isnan(tcp_seed.tt) || isnan(a))
//		printf("nan em compute_a_and_t_from_s_reverse()\n");

	//	printf("s %.1lf, a %.3lf, t %.1lf, tv %.1lf, vi %.1lf\n", s, a, tcp_seed.tt, target_v, target_td.v_i);
	params->suitable_tt = tcp_seed.tt;
	params->suitable_acceleration = tcp_seed.a = a;
}


void
compute_a_and_t_from_s_foward(double s, double target_v,
		TrajectoryLookupTable::TrajectoryDimensions target_td,
		TrajectoryLookupTable::TrajectoryControlParameters &tcp_seed,
		ObjectiveFunctionParams *params)
{
	// https://www.wolframalpha.com/input/?i=solve+s%3Dv*x%2B0.5*a*x%5E2
	double a = (target_v * target_v - target_td.v_i * target_td.v_i) / (2.0 * s);
	if (a == 0.0)
		a = 0.000001;
	tcp_seed.tt = (target_v - target_td.v_i) / a;
	if (a > GlobalState::robot_config.maximum_acceleration_forward)
	{
		a = GlobalState::robot_config.maximum_acceleration_forward;
		double v = target_td.v_i;
		tcp_seed.tt = (sqrt(2.0 * a * s + v * v) - v) / a;
	}
	else if (a < -GlobalState::robot_config.maximum_deceleration_forward)
	{
		a = -GlobalState::robot_config.maximum_deceleration_forward;
		double v = target_td.v_i;
		tcp_seed.tt = -(sqrt(2.0 * a * s + v * v) + v) / a;
	}
	if (tcp_seed.tt > 200.0)
		tcp_seed.tt = 200.0;
	if (tcp_seed.tt < 0.05)
		tcp_seed.tt = 0.05;

	//	printf("s %.1lf, a %.3lf, t %.1lf, tv %.1lf, vi %.1lf\n", s, a, tcp_seed.tt, target_v, target_td.v_i);

//	if (isnan(tcp_seed.tt) || isnan(a))
//		printf("nan em compute_a_and_t_from_s_foward()\n");

	params->suitable_tt = tcp_seed.tt;
	params->suitable_acceleration = tcp_seed.a = a;
}


void
compute_a_and_t_from_s(double s, double target_v,
		TrajectoryLookupTable::TrajectoryDimensions target_td,
		TrajectoryLookupTable::TrajectoryControlParameters &tcp_seed,
		ObjectiveFunctionParams *params)
{
	if (GlobalState::reverse_planning)
		compute_a_and_t_from_s_reverse(s, target_v, target_td, tcp_seed, params);
	else
		compute_a_and_t_from_s_foward(s, target_v, target_td, tcp_seed, params);
}


TrajectoryLookupTable::TrajectoryControlParameters
fill_in_tcp(const gsl_vector *x, ObjectiveFunctionParams *params)
{
	TrajectoryLookupTable::TrajectoryControlParameters tcp;

	tcp.shift_knots = params->tcp_seed->shift_knots;
	if (x->size == 4)
	{
		tcp.has_k1 = true;

		tcp.k1 = gsl_vector_get(x, 0);
		tcp.k2 = gsl_vector_get(x, 1);
		tcp.k3 = gsl_vector_get(x, 2);
		if (params->optimize_time == OPTIMIZE_TIME)
		{
			tcp.tt = gsl_vector_get(x, 3);
			tcp.a = params->suitable_acceleration;
		}
		if (params->optimize_time == OPTIMIZE_DISTANCE)
		{
			tcp.s = gsl_vector_get(x, 3);
			if (tcp.s < 0.01)
				tcp.s = 0.01;
			compute_a_and_t_from_s(tcp.s, params->target_v, *params->target_td, tcp, params);
			tcp.a = params->suitable_acceleration;
			tcp.tt = params->suitable_tt;
		}
		if (params->optimize_time == OPTIMIZE_ACCELERATION)
		{
			tcp.a = gsl_vector_get(x, 3);
			tcp.tt = params->suitable_tt;
		}
	}
	else if (x->size == 5)
	{
		tcp.has_k1 = true;

		tcp.k1 = gsl_vector_get(x, 0);
		tcp.k2 = gsl_vector_get(x, 1);
		tcp.k3 = gsl_vector_get(x, 2);
		tcp.a = gsl_vector_get(x, 4);
		if (params->optimize_time == OPTIMIZE_TIME)
		{
			tcp.tt = gsl_vector_get(x, 3);
		}
		if (params->optimize_time == OPTIMIZE_DISTANCE)
		{
			tcp.s = gsl_vector_get(x, 3);
			if (tcp.s < 0.01)
				tcp.s = 0.01;
			double v = params->target_td->v_i;
			if (tcp.a > 0.0001)
			{
				tcp.tt = (sqrt(2.0 * tcp.a * tcp.s + v * v) - v) / tcp.a;
			}
			else if (tcp.a < -0.0001)
			{
				tcp.tt = -(sqrt(2.0 * tcp.a * tcp.s + v * v) + v) / tcp.a;
			}
			else
				tcp.tt = tcp.s / v;

			params->suitable_tt = tcp.tt;
		}
	}
	else
	{
		tcp.has_k1 = false;

		tcp.k2 = gsl_vector_get(x, 0);
		tcp.k3 = gsl_vector_get(x, 1);
		if (params->optimize_time == OPTIMIZE_TIME)
		{
			tcp.tt = gsl_vector_get(x, 2);
			tcp.a = params->suitable_acceleration;
		}
		if (params->optimize_time == OPTIMIZE_DISTANCE)
		{
			tcp.s = gsl_vector_get(x, 2);
			if (tcp.s < 0.01)
				tcp.s = 0.01;
			compute_a_and_t_from_s(tcp.s, params->target_v, *params->target_td, tcp, params);
			tcp.a = params->suitable_acceleration;
			tcp.tt = params->suitable_tt;
		}
		if (params->optimize_time == OPTIMIZE_ACCELERATION)
		{
			tcp.a = gsl_vector_get(x, 2);
			tcp.tt = params->suitable_tt;
		}
	}

	if (tcp.tt < 0.05) // o tempo nao pode ser pequeno demais
		tcp.tt = 0.05;
	if (tcp.s < 0.01)
		tcp.s = 0.01;

//	if (tcp.a < -GlobalState::robot_config.maximum_deceleration_forward) // a aceleracao nao pode ser negativa demais//TODO
//		tcp.a = -GlobalState::robot_config.maximum_deceleration_forward;
//	if (tcp.a > GlobalState::robot_config.maximum_acceleration_forward) // a aceleracao nao pode ser positiva demais
//		tcp.a = GlobalState::robot_config.maximum_acceleration_forward;

	tcp.valid = true;

	return (tcp);
}


void
move_path_to_current_robot_pose(vector<carmen_robot_and_trailer_path_point_t> &path, carmen_robot_and_trailer_pose_t *localizer_pose)
{
	for (std::vector<carmen_robot_and_trailer_path_point_t>::iterator it = path.begin(); it != path.end(); ++it)
	{
		double x = localizer_pose->x + it->x * cos(localizer_pose->theta) - it->y * sin(localizer_pose->theta);
		double y = localizer_pose->y + it->x * sin(localizer_pose->theta) + it->y * cos(localizer_pose->theta);
		it->x = x;
		it->y = y;
		it->theta = carmen_normalize_theta(it->theta + localizer_pose->theta);
	}
}


carmen_robot_and_trailer_path_point_t
move_to_front_axle(carmen_robot_and_trailer_path_point_t pose)
{
	return (pose);

	double L = GlobalState::robot_config.distance_between_front_and_rear_axles;
	carmen_robot_and_trailer_path_point_t pose_moved = pose;
	pose_moved.x += L * cos(pose.theta);
	pose_moved.y += L * sin(pose.theta);

	return (pose_moved);
}


double
compute_path_to_lane_distance(ObjectiveFunctionParams *my_params, vector<carmen_robot_and_trailer_path_point_t> &path)
{
	double distance = 0.0;
	double total_distance = 0.0;
	double total_points = 0.0;

	int increment;
	if (use_unity_simulator)
		increment = 1;
	else
		increment = 3;
	for (unsigned int i = 0; i < path.size(); i += increment)
	{
		if ((i < my_params->path_point_nearest_to_lane.size()) &&
			(my_params->path_point_nearest_to_lane.at(i) < my_params->detailed_lane.size()))
		{
			if (GlobalState::reverse_planning)
				distance = DIST2D(path.at(i),
								my_params->detailed_lane.at(my_params->path_point_nearest_to_lane.at(i)));
			else
				distance = DIST2D(move_to_front_axle(path.at(i)),
								my_params->detailed_lane.at(my_params->path_point_nearest_to_lane.at(i)));
			total_points += 1.0;
		}
		else
			distance = 0.0;

		total_distance += distance * distance;
	}

	if (total_points > 0.0)
		return (((total_distance / total_points) > 7.0)? 7.0: total_distance / total_points);
	else
		return (0.0);
}


vector<carmen_robot_and_trailer_path_point_t>
compute_path_to_lane_distance_evaluation(ObjectiveFunctionParams *my_params, vector<carmen_robot_and_trailer_path_point_t> &path)
{
	int increment;
	if (use_unity_simulator)
		increment = 1;
	else
		increment = 3;

	vector<carmen_robot_and_trailer_path_point_t> modified_path;
	for (unsigned int i = 0; i < path.size(); i += increment)
	{
		if ((i < my_params->path_point_nearest_to_lane.size()) &&
			(my_params->path_point_nearest_to_lane.at(i) < my_params->detailed_lane.size()))
		{
			if (GlobalState::reverse_planning)
				modified_path.push_back(path.at(i));
			else
				modified_path.push_back(move_to_front_axle(path.at(i)));
		}
	}

	return (modified_path);
}


void
compute_path_points_nearest_to_lane(ObjectiveFunctionParams *param, vector<carmen_robot_and_trailer_path_point_t> &path)
{
	param->path_point_nearest_to_lane.clear();
	param->path_size = path.size();
	unsigned int index = 0;
	for (unsigned int j = 0; j < path.size(); j++)
	{
		carmen_robot_and_trailer_path_point_t axle;

		if (GlobalState::reverse_planning) //mantem o eixo traseiro
			axle = path.at(j);
		else
			axle = move_to_front_axle(path.at(j));

		double min_dist = DIST2D(axle, param->detailed_lane.at(index));
		for (unsigned int i = index; i < param->detailed_lane.size(); i++)
		{
			double distance = DIST2D(axle, param->detailed_lane.at(i));
			if (distance < min_dist)
			{
				min_dist = distance;
				index = i;
			}
		}
		param->path_point_nearest_to_lane.push_back(index);
	}
}


inline carmen_ackerman_path_point_t
move_path_point_to_map_coordinates(const carmen_ackerman_path_point_t point, double displacement)
{
	carmen_ackerman_path_point_t path_point_in_map_coords;
	double coss, sine;

	sincos(point.theta, &sine, &coss);
	double x_disp = point.x + displacement * coss;
	double y_disp = point.y + displacement * sine;

	sincos(GlobalState::localizer_pose->theta, &sine, &coss);
	path_point_in_map_coords.x = (GlobalState::localizer_pose->x - GlobalState::distance_map->config.x_origin + x_disp * coss - y_disp * sine) / GlobalState::distance_map->config.resolution;
	path_point_in_map_coords.y = (GlobalState::localizer_pose->y - GlobalState::distance_map->config.y_origin + x_disp * sine + y_disp * coss) / GlobalState::distance_map->config.resolution;

	return (path_point_in_map_coords);
}


double
compute_proximity_to_obstacles_using_distance_map(vector<carmen_robot_and_trailer_path_point_t> path)
{
	double proximity_to_obstacles_for_path = 0.0;
	double safety_distance = GlobalState::robot_config.model_predictive_planner_obstacles_safe_distance;

	for (unsigned int i = 0; i < path.size(); i += 1)
	{
		carmen_robot_and_trailer_pose_t point_to_check = {path[i].x, path[i].y, path[i].theta, path[i].beta};
		double proximity_point = carmen_obstacle_avoider_compute_car_distance_to_closest_obstacles(GlobalState::localizer_pose,
				point_to_check, GlobalState::distance_map, safety_distance);
		proximity_to_obstacles_for_path += proximity_point;
//		carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, carmen_get_time());
//		getchar();
	}

	return (proximity_to_obstacles_for_path);
}


double
my_f(const gsl_vector *x, void *params)
{
	ObjectiveFunctionParams *my_params = (ObjectiveFunctionParams *) params;

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(x, my_params);
	TrajectoryLookupTable::TrajectoryDimensions td;

	if (bad_tcp(tcp))
		return (1000000.0);

	vector<carmen_robot_and_trailer_path_point_t> path = simulate_car_from_parameters(td, tcp, my_params->target_td->v_i, my_params->target_td->phi_i, my_params->target_td->beta_i, false);

	my_params->plan_cost = ((td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index +
		(carmen_normalize_theta(td.theta - my_params->target_td->theta) * carmen_normalize_theta(td.theta - my_params->target_td->theta)) / (my_params->theta_by_index * 2.0) +
		(carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw) * carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw)) / (my_params->d_yaw_by_index * 2.0));

	double activate_factor = 1.0;
	if (my_params->target_td->dist < 4.0)
	{
		if (my_params->target_td->dist > 3.0)
			activate_factor = my_params->target_td->dist - 3.0;
		else
			activate_factor = 0.0;
	}

	double result = sqrt(
				GlobalState::w1 * (td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index +
				activate_factor * 4.0 * GlobalState::w2 * (carmen_normalize_theta(td.theta - my_params->target_td->theta) * carmen_normalize_theta(td.theta - my_params->target_td->theta)) / my_params->theta_by_index +
				GlobalState::w3 * (carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw) * carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw)) / my_params->d_yaw_by_index
				);

	return (result);
}


void
my_df(const gsl_vector *v, void *params, gsl_vector *df)
{
	double h;

	double f_x = my_f(v, params);

	h = 0.00005;

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


void
my_fdf(const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
	*f = my_f(x, params);
	my_df(x, params, df);
}


double
my_g(const gsl_vector *x, void *params)
{
	ObjectiveFunctionParams *my_params = (ObjectiveFunctionParams *) params;

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(x, my_params);
	TrajectoryLookupTable::TrajectoryDimensions td;

	if (bad_tcp(tcp))
		return (1000000.0);

	vector<carmen_robot_and_trailer_path_point_t> path = simulate_car_from_parameters(td, tcp, my_params->target_td->v_i, my_params->target_td->phi_i, my_params->target_td->beta_i, false);

	double path_to_lane_distance = 0.0;
	if (my_params->use_lane && (my_params->detailed_lane.size() > 0) && (path.size() > 0))
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
	if (use_obstacles && GlobalState::distance_map != NULL && path.size() > 0)
		proximity_to_obstacles = compute_proximity_to_obstacles_using_distance_map(path);

	my_params->plan_cost = sqrt((td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index +
			(carmen_normalize_theta(td.theta - my_params->target_td->theta) * carmen_normalize_theta(td.theta - my_params->target_td->theta)) / (my_params->theta_by_index * 0.2) +
			(carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw) * carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw)) / (my_params->d_yaw_by_index * 0.2));

	double activate_factor = 1.0;
	if (my_params->target_td->dist < 4.0)
	{
		if (my_params->target_td->dist > 3.0)
			activate_factor = my_params->target_td->dist - 3.0;
		else
			activate_factor = 0.0;
	}
	double result = sqrt(
				GlobalState::w1 * ((td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist)) / my_params->distance_by_index +
				activate_factor * GlobalState::w2 * (carmen_normalize_theta(td.theta - my_params->target_td->theta) * carmen_normalize_theta(td.theta - my_params->target_td->theta)) / my_params->theta_by_index +
				GlobalState::w3 * (carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw) * carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw)) / my_params->d_yaw_by_index +
				activate_factor * GlobalState::w4 * path_to_lane_distance + // já é quandrática
				GlobalState::w5 * proximity_to_obstacles);// +
//				GlobalState::w6 * (tcp.k1 * tcp.k1 + tcp.k2 * tcp.k2 + tcp.k3 * tcp.k3));

	return (result);
}


void
my_dg(const gsl_vector *v, void *params, gsl_vector *df)
{
	double g_x = my_g(v, params);

	double h = 0.00005;//<<< 0.00001

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


void
my_gdf(const gsl_vector *x, void *params, double *g, gsl_vector *dg)
{
	*g = my_g(x, params);
	my_dg(x, params, dg);
}


double
my_h(const gsl_vector *x, void *params)
{
	ObjectiveFunctionParams *my_params = (ObjectiveFunctionParams *) params;

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(x, my_params);
	TrajectoryLookupTable::TrajectoryDimensions td;

	if (bad_tcp(tcp))
		return (1000000.0);

	vector<carmen_robot_and_trailer_path_point_t> path = simulate_car_from_parameters(td, tcp, my_params->target_td->v_i, my_params->target_td->phi_i, my_params->target_td->beta_i, false);

	double path_to_lane_distance = 0.0;
	if (my_params->use_lane && (my_params->detailed_lane.size() > 0) && (path.size() > 0))
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

	if (use_obstacles && GlobalState::distance_map != NULL && (path.size() > 0))
		proximity_to_obstacles = compute_proximity_to_obstacles_using_distance_map(path);

	my_params->plan_cost = sqrt((td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index +
			(carmen_normalize_theta(td.theta - my_params->target_td->theta) * carmen_normalize_theta(td.theta - my_params->target_td->theta)) / (my_params->theta_by_index * 0.2) +
			(carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw) * carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw)) / (my_params->d_yaw_by_index * 0.2));

	double w1, w2, w3, w4, w5, w6, result;
	if (((ObjectiveFunctionParams *) (params))->optimize_time == OPTIMIZE_DISTANCE)
	{
		w1 = 30.0; w2 = 15.0; w3 = 15.0; w4 = 3.0; w5 = 3.0; w6 = 0.005;
//		if (td.dist < 7.0)
//			w2 *= exp(td.dist - 7.0);
		result = (
				w1 * (td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index +
				w2 * (carmen_normalize_theta(td.theta - my_params->target_td->theta) * carmen_normalize_theta(td.theta - my_params->target_td->theta)) / my_params->theta_by_index +
				w3 * (carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw) * carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw)) / my_params->d_yaw_by_index +
				w4 * path_to_lane_distance + // já é quandrática
				w5 * proximity_to_obstacles + // já é quandrática
				w6 * tcp.sf * tcp.sf);
	}
	else
	{
		w1 = 10.0; w2 = 15.0; w3 = 30.0; w4 = 5.0; w5 = 10.0; w6 = 0.0005;
//		double w7;
//		w7 = 0.1;//(my_params->target_td->v_i > 0.2 ? 2.0 : 0.0);
//		if (td.dist < 7.0)
//			w2 *= exp(td.dist - 7.0);
		result = sqrt(
				w1 * (td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index +
				w2 * (carmen_normalize_theta(td.theta - my_params->target_td->theta) * carmen_normalize_theta(td.theta - my_params->target_td->theta)) / my_params->theta_by_index +
				w3 * (carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw) * carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw)) / my_params->d_yaw_by_index +
				w4 * path_to_lane_distance + // já é quandrática
				w5 * proximity_to_obstacles + // já é quandrática
				w6 * tcp.sf * tcp.sf); //+
//				w7 * (my_params->target_v - tcp.vf) * (my_params->target_v - tcp.vf));
//		printf("--------\nW1: %0.2lf W2: %0.2lf W3: %0.2lf W4: %0.2lf W5: %0.2lf W6: %0.2lf W7: %0.2lf\n",
//				(w1 * (td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index),
//				(w2 * (carmen_normalize_theta(td.theta - my_params->target_td->theta) * carmen_normalize_theta(td.theta - my_params->target_td->theta)) / my_params->theta_by_index),
//				(w3 * (carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw) * carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw)) / my_params->d_yaw_by_index),
//				(w4 * path_to_lane_distance), // já é quandrática
//				(w5 * proximity_to_obstacles), // já é quandrática
//				(w6 * tcp.sf * tcp.sf),
//				(w7 * (my_params->target_v - tcp.vf) * (my_params->target_v - tcp.vf)));
	}

//	printf("NEW result %.2lf s %.2lf sf %.2lf, tdc %.2lf, tdd %.2f, a %lf v_i %.2lf v_f %.2lf vt %.2lf TT %lf\n--------\n", result, tcp.s, tcp.sf,
//			td.dist,my_params->target_td->dist, tcp.a, my_params->target_td->v_i, tcp.vf, my_params->target_v, tcp.tt);
	return (result);
}



void
my_dh(const gsl_vector *v, void *params, gsl_vector *df)
{
	double g_x = my_h(v, params);

	double h = 0.00005;//<<< 0.00001

	gsl_vector *x_h;
	x_h = gsl_vector_alloc(5);

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0) + h);
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1));
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2));
	gsl_vector_set(x_h, 3, gsl_vector_get(v, 3));
	gsl_vector_set(x_h, 4, gsl_vector_get(v, 4));
	double g_x_h = my_h(x_h, params);
	double d_g_x_h = (g_x_h - g_x) / h;

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0));
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1) + h);
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2));
	gsl_vector_set(x_h, 3, gsl_vector_get(v, 3));
	gsl_vector_set(x_h, 4, gsl_vector_get(v, 4));
	double g_y_h = my_h(x_h, params);
	double d_g_y_h = (g_y_h - g_x) / h;

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0));
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1));
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2) + h);
	gsl_vector_set(x_h, 3, gsl_vector_get(v, 3));
	gsl_vector_set(x_h, 4, gsl_vector_get(v, 4));
	double g_z_h = my_h(x_h, params);
	double d_g_z_h = (g_z_h - g_x) / h;

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0));
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1));
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2));
	gsl_vector_set(x_h, 3, gsl_vector_get(v, 3) + h);
	gsl_vector_set(x_h, 4, gsl_vector_get(v, 4));
	double g_w_h = my_h(x_h, params);
	double d_g_w_h = (g_w_h - g_x) / h;

	gsl_vector_set(x_h, 0, gsl_vector_get(v, 0));
	gsl_vector_set(x_h, 1, gsl_vector_get(v, 1));
	gsl_vector_set(x_h, 2, gsl_vector_get(v, 2));
	gsl_vector_set(x_h, 3, gsl_vector_get(v, 3));
	gsl_vector_set(x_h, 4, gsl_vector_get(v, 4) + h);
	double g_v_h = my_h(x_h, params);
	double d_g_v_h = (g_v_h - g_x) / h;

	gsl_vector_set(df, 0, d_g_x_h);
	gsl_vector_set(df, 1, d_g_y_h);
	gsl_vector_set(df, 2, d_g_z_h);
	gsl_vector_set(df, 3, d_g_w_h);
	gsl_vector_set(df, 4, d_g_v_h);

	gsl_vector_free(x_h);
}


void
my_hdf(const gsl_vector *x, void *params, double *g, gsl_vector *dh)
{
	*g = my_h(x, params);
	my_dh(x, params, dh);
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
		if (a > GlobalState::robot_config.maximum_acceleration_forward)
			a = GlobalState::robot_config.maximum_acceleration_forward;
		return (a);
	}

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
compute_suitable_acceleration_and_tt(ObjectiveFunctionParams &params,
		TrajectoryLookupTable::TrajectoryControlParameters &tcp_seed,
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
	//
	// Se estou co velocidade vi e quero chagar a vt, sendo que vt < vi, a eh negativo. O tempo, tt, para
	// ir de vi a vt pode ser derivado de dS/dt = Vo + a*t -> vt = vi + a*tt; a*tt = vt - vi; tt = (vt - vi) / a

	if (!GlobalState::reverse_planning && target_v < 0.0)
		target_v = 0.0;
	params.optimize_time = OPTIMIZE_DISTANCE;
//	params.optimize_time = OPTIMIZE_TIME;

	if (params.optimize_time == OPTIMIZE_DISTANCE)
	{
		tcp_seed.s = target_td.dist; // Pior caso: forcca otimizacao para o primeiro zero da distancia, evitando voltar de reh para atingir a distancia.
		compute_a_and_t_from_s(tcp_seed.s, target_v, target_td, tcp_seed, &params);
	}
	else
	{
//		double a = (target_v * target_v - target_td.v_i * target_td.v_i) / (2.0 * target_td.dist);
//		double tt = (target_v - target_td.v_i) / a;
		double a = (target_v - target_td.v_i) / tcp_seed.tt;
		double tt;

		if (a == 0.0) //avoid div by zero and plan v = 0 e vi = 0
		{
			tt = tcp_seed.tt;

			if (target_td.v_i == 0.0)
				params.optimize_time = OPTIMIZE_ACCELERATION;
			else
				params.optimize_time = OPTIMIZE_TIME;
		}
		else
			tt = tcp_seed.tt; // Isso vai ficar assim enquanto a < 0.0... Nao deveria haver um ajuste se "a" ficar com um valor indesejavel? So o limite maximo eh tratado abaixo...


		if (a > 0.0)
		{
			params.optimize_time = OPTIMIZE_TIME;
			if (a > GlobalState::robot_config.maximum_acceleration_forward)
				a = GlobalState::robot_config.maximum_acceleration_forward;
		}

		if (a < 0.0)
		{
			params.optimize_time = OPTIMIZE_ACCELERATION;
			if (a < -GlobalState::robot_config.maximum_deceleration_forward)
			{
				a = -GlobalState::robot_config.maximum_deceleration_forward;
				tt = (target_v - target_td.v_i) / a;
			}
		}

		if (tt > 15.0)
			tt = 15.0;
		else if (tt < 0.05)
			tt = 0.05;

		params.suitable_tt = tcp_seed.tt = tt;
		params.suitable_acceleration = tcp_seed.a = a;
		//printf("SUITABLE a %lf, tt %lf\n", a, tt);
	}
}


void
get_optimization_params(ObjectiveFunctionParams &params, double target_v,
		TrajectoryLookupTable::TrajectoryControlParameters *tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions *target_td)
{
	params.distance_by_index = fabs(get_distance_by_index(N_DIST - 1));
	params.theta_by_index = fabs(get_theta_by_index(N_THETA - 1));
	params.d_yaw_by_index = fabs(get_d_yaw_by_index(N_D_YAW - 1));
	params.target_td = target_td;
	params.tcp_seed = tcp_seed;
	params.target_v = target_v;
	params.path_size = 0;
}


void
get_missing_k1(TrajectoryLookupTable::TrajectoryControlParameters &tcp, TrajectoryLookupTable::TrajectoryDimensions target_td)
{
	double knots_x[3] = { 0.0, tcp.tt / 2.0, tcp.tt };
	double knots_y[3];
	knots_y[0] = target_td.phi_i;
	knots_y[1] = tcp.k2;
	knots_y[2] = tcp.k3;
	gsl_interp_accel* acc = gsl_interp_accel_alloc();
	const gsl_interp_type* type = gsl_interp_cspline;
	gsl_spline* phi_spline = gsl_spline_alloc(type, 3);
	gsl_spline_init(phi_spline, knots_x, knots_y, 3);
	if (tcp.shift_knots)
		tcp.k1 = gsl_spline_eval(phi_spline, 3.0 * (tcp.tt / 4.0), acc);
	else
		tcp.k1 = gsl_spline_eval(phi_spline, tcp.tt / 4.0, acc);
	tcp.has_k1 = true;
	gsl_spline_free(phi_spline);
	gsl_interp_accel_free(acc);
}


void
print_tcp(TrajectoryLookupTable::TrajectoryControlParameters tcp, double plan_cost)
{
	printf("pc %1.2lf, v %d, h_k1 %d, sk %d, tt %1.4lf, a %1.4lf, k1 %1.4lf, k2 %1.4lf, k3 %1.4lf, vf %2.4lf, sf %2.2lf, s %lf, ts %lf\n",
			plan_cost, tcp.valid, tcp.has_k1, tcp.shift_knots, tcp.tt, tcp.a, tcp.k1, tcp.k2, tcp.k3, tcp.vf, tcp.sf, tcp.s, carmen_get_time());
}


void
print_td(TrajectoryLookupTable::TrajectoryDimensions td)
{
	printf("dist %2.4lf, theta %1.4lf, d_yaw %1.4lf, phi_i %1.4lf, v_i %2.4lf %lf\n",
			td.dist, td.theta, td.d_yaw, td.phi_i, td.v_i, carmen_get_time());
}


void
print_td(TrajectoryLookupTable::TrajectoryDimensions td, double target_v)
{
	printf("dist %2.4lf, theta %1.4lf, d_yaw %1.4lf, phi_i %1.4lf, v_i %2.4lf, t_v %2.4lf %lf\n",
			td.dist, td.theta, td.d_yaw, td.phi_i, td.v_i, target_v, carmen_get_time());
}


TrajectoryLookupTable::TrajectoryControlParameters
optimized_lane_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters &tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v, ObjectiveFunctionParams params)
{
	get_optimization_params(params, target_v, &tcp_seed, &target_td);

	gsl_multimin_function_fdf my_func;

	my_func.n = 4;
	my_func.f = my_g;
	my_func.df = my_dg;
	my_func.fdf = my_gdf;
	my_func.params = &params;

	/* Starting point, x */
	gsl_vector *x = gsl_vector_alloc(4);
	gsl_vector_set(x, 0, tcp_seed.k1);
	gsl_vector_set(x, 1, tcp_seed.k2);
	gsl_vector_set(x, 2, tcp_seed.k3);
	if (params.optimize_time == OPTIMIZE_TIME)
		gsl_vector_set(x, 3, tcp_seed.tt);
	if (params.optimize_time == OPTIMIZE_DISTANCE)
		gsl_vector_set(x, 3, tcp_seed.s);
	if (params.optimize_time == OPTIMIZE_ACCELERATION)
		gsl_vector_set(x, 3, tcp_seed.a);

	const gsl_multimin_fdfminimizer_type *T = gsl_multimin_fdfminimizer_conjugate_fr;
	gsl_multimin_fdfminimizer *s = gsl_multimin_fdfminimizer_alloc(T, 4);

	// int gsl_multimin_fdfminimizer_set (gsl_multimin_fdfminimizer * s, gsl_multimin_function_fdf * fdf, const gsl_vector * x, double step_size, double tol)
	gsl_multimin_fdfminimizer_set(s, &my_func, x, G_STEP_SIZE, G_TOL);

	size_t iter = 0;
	int status;
	do
	{
		iter++;

		status = gsl_multimin_fdfminimizer_iterate(s);

		if (status)
			break;

		status = gsl_multimin_test_gradient(s->gradient, G_EPSABS); // esta funcao retorna GSL_CONTINUE ou zero
	} while ((status == GSL_CONTINUE) && (iter < 50));

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(s->x, &params);

	if (bad_tcp(tcp))
		tcp.valid = false;

	if (tcp.tt < 0.05)
		tcp.valid = false;

	if (params.plan_cost > 2.5) // 5.5) // 1.5)
		tcp.valid = false;

	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

	return (tcp);
}


TrajectoryLookupTable::TrajectoryControlParameters
optimized_lane_trajectory_control_parameters_new(TrajectoryLookupTable::TrajectoryControlParameters &tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v, ObjectiveFunctionParams params)
{
	//	vector<carmen_ackerman_path_point_t> path = simulate_car_from_parameters(target_td, tcp_seed, target_td.v_i, target_td.phi_i, target_td.beta_i, false);

	//	FILE *lane_file = fopen("gnu_tests/gnuplot_lane.txt", "w");
	//	print_lane(params.detailed_lane, lane_file);
	//	fclose(lane_file);
	//	char path_name[20];
	//	sprintf(path_name, "path/%d.txt", 0);
	//	FILE *path_file = fopen("gnu_tests/gnuplot_traj.txt", "w");
	//	print_lane(path,path_file);
	//	fclose(path_file);
	//	getchar();

	get_optimization_params(params, target_v, &tcp_seed, &target_td);
	//print_tcp(tcp_seed);
	//print_td(target_td);
	//printf("tv = %lf\n", target_v);

	gsl_multimin_function_fdf my_func;

	my_func.n = 5;
	my_func.f = my_h;
	my_func.df = my_dh;
	my_func.fdf = my_hdf;
	my_func.params = &params;

	/* Starting point, x */
	gsl_vector *x = gsl_vector_alloc(5);
	gsl_vector_set(x, 0, tcp_seed.k1);
	gsl_vector_set(x, 1, tcp_seed.k2);
	gsl_vector_set(x, 2, tcp_seed.k3);
	if (params.optimize_time == OPTIMIZE_TIME)
		gsl_vector_set(x, 3, tcp_seed.tt);
	if (params.optimize_time == OPTIMIZE_DISTANCE)
		gsl_vector_set(x, 3, tcp_seed.s);
	gsl_vector_set(x, 4, tcp_seed.a);

	const gsl_multimin_fdfminimizer_type *T = gsl_multimin_fdfminimizer_conjugate_fr;
	gsl_multimin_fdfminimizer *s = gsl_multimin_fdfminimizer_alloc(T, 5);

	// int gsl_multimin_fdfminimizer_set (gsl_multimin_fdfminimizer * s, gsl_multimin_function_fdf * fdf, const gsl_vector * x, double step_size, double tol)
	gsl_multimin_fdfminimizer_set(s, &my_func, x, H_STEP_SIZE, H_TOL);

	size_t iter = 0;
	int status;
	do
	{
		iter++;

		status = gsl_multimin_fdfminimizer_iterate(s);

		if (status)
		{
//			if (status != GSL_ENOPROG) // GSL_ENOPROG means that the minimizer is unable to improve on its current estimate, either due to numerical difficulty or a genuine local minimum
//			{
//				printf("GSL h minimizer error - %s\n", gsl_strerror(status));
//				fflush(stdout);
//			}

			break;
		}

		status = gsl_multimin_test_gradient(s->gradient, H_EPSABS); // esta funcao retorna GSL_CONTINUE ou zero

		//	--Debug with GNUPLOT

		//		TrajectoryLookupTable::TrajectoryControlParameters tcp_temp = fill_in_tcp(s->x, &params);
		//		char path_name[20];
		//		sprintf(path_name, "path/%lu.txt", iter);
		//		FILE *path_file = fopen("gnu_tests/gnuplot_traj.txt", "w");
		//		print_lane(simulate_car_from_parameters(target_td, tcp_temp, target_td.v_i, target_td.phi_i, target_td.beta_i, true), path_file);
		//		fclose(path_file);
		//		printf("Estou na: %lu iteracao, sf: %lf  \n", iter, s->f);
		//		getchar();
		//	--
//		params.suitable_acceleration = compute_suitable_acceleration(gsl_vector_get(x, 3), target_td, target_v);

	} while (/*(s->f > MAX_LANE_DIST) &&*/ (status == GSL_CONTINUE) && (iter < 50));

//	printf("iter = %ld\n", iter);

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(s->x, &params);

	if (bad_tcp(tcp))
		tcp.valid = false;

	if (tcp.tt < 0.05 || s->f > 20.0)
	{
//		printf("Plano invalido>>>>>>>>>>>>>> tt: %lf s->f %lf\n",tcp.tt, s->f);
		tcp.valid = false;
	}

//	printf("plan_cost = %lf\n", params.plan_cost);
	if (params.plan_cost > 2.0)
	{
//		printf(">>>>>>>>>>>>>> plan_cost > 3.6\n");
		tcp.valid = false;
	}

	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

	// print_tcp(tcp);
	return (tcp);
}


TrajectoryLookupTable::TrajectoryControlParameters
get_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions target_td, ObjectiveFunctionParams &params)
{
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
	if (params.optimize_time == OPTIMIZE_TIME)
		gsl_vector_set(x, 2, tcp_seed.tt);
	if (params.optimize_time == OPTIMIZE_DISTANCE)
		gsl_vector_set(x, 2, tcp_seed.s);
	if (params.optimize_time == OPTIMIZE_ACCELERATION)
		gsl_vector_set(x, 2, tcp_seed.a);

	const gsl_multimin_fdfminimizer_type *T = gsl_multimin_fdfminimizer_conjugate_fr;
	gsl_multimin_fdfminimizer *s = gsl_multimin_fdfminimizer_alloc(T, 3);

	gsl_multimin_fdfminimizer_set(s, &my_func, x, F_STEP_SIZE, F_TOL);

	size_t iter = 0;
	int status;
	do
	{
		iter++;

		status = gsl_multimin_fdfminimizer_iterate(s);

		if (status)
			break;

		status = gsl_multimin_test_gradient(s->gradient, F_EPSABS); // esta funcao retorna GSL_CONTINUE ou zero

	} while ((params.plan_cost > 0.005) && (status == GSL_CONTINUE) && (iter < 50));

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(s->x, &params);

	if (bad_tcp(tcp))
		tcp.valid = false;

	if ((tcp.tt < 0.05) || (params.plan_cost > 1.0))// 0.25)) // too short plan or bad minimum (s->f should be close to zero) mudei de 0.05 para outro
		tcp.valid = false;

	if (target_td.dist < 3.0 && tcp.valid == false) // para debugar
		tcp.valid = false;

	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

	return (tcp);
}


TrajectoryLookupTable::TrajectoryControlParameters
get_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
		TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd, double target_v)
{
	ObjectiveFunctionParams params;

	TrajectoryLookupTable::TrajectoryDimensions target_td = convert_to_trajectory_dimensions(tdd, tcp_seed);
	get_optimization_params(params, target_v, &tcp_seed, &target_td);
	compute_suitable_acceleration_and_tt(params, tcp_seed, target_td, target_v);

	TrajectoryLookupTable::TrajectoryControlParameters tcp = get_optimized_trajectory_control_parameters(tcp_seed, target_td, params);

	return (tcp);
}


void
shift_k1(TrajectoryLookupTable::TrajectoryControlParameters &tcp, TrajectoryLookupTable::TrajectoryDimensions target_td)
{
	double knots_x[4] = { 0.0, tcp.tt / 4.0, tcp.tt / 2.0, tcp.tt };
	double knots_y[4];
	knots_y[0] = target_td.phi_i;
	knots_y[1] = tcp.k1;
	knots_y[2] = tcp.k2;
	knots_y[3] = tcp.k3;
	gsl_interp_accel *acc = gsl_interp_accel_alloc();
	const gsl_interp_type *type = gsl_interp_cspline;
	gsl_spline *phi_spline = gsl_spline_alloc(type, 4);
	gsl_spline_init(phi_spline, knots_x, knots_y, 4);
	tcp.k1 = gsl_spline_eval(phi_spline, 3.0 * (tcp.tt / 4.0), acc);

	gsl_spline_free(phi_spline);
	gsl_interp_accel_free(acc);
}


void
un_shift_k1(TrajectoryLookupTable::TrajectoryControlParameters &tcp, TrajectoryLookupTable::TrajectoryDimensions target_td)
{
	double knots_x[4] = { 0.0, tcp.tt / 2.0, 3.0 * (tcp.tt / 4.0), tcp.tt };
	double knots_y[4];
	knots_y[0] = target_td.phi_i;
	knots_y[1] = tcp.k2;
	knots_y[2] = tcp.k1;
	knots_y[3] = tcp.k3;
	gsl_interp_accel *acc = gsl_interp_accel_alloc();
	const gsl_interp_type *type = gsl_interp_cspline;
	gsl_spline *phi_spline = gsl_spline_alloc(type, 4);
	gsl_spline_init(phi_spline, knots_x, knots_y, 4);
	tcp.k1 = gsl_spline_eval(phi_spline, tcp.tt / 4.0, acc);

	gsl_spline_free(phi_spline);
	gsl_interp_accel_free(acc);
}


void
verify_shift_option_for_k1(TrajectoryLookupTable::TrajectoryControlParameters &tcp,
		TrajectoryLookupTable::TrajectoryDimensions target_td)
{
	if (((target_td.v_i > (18.0 / 3.6)) && (target_td.dist < 20.0))	||
		((target_td.v_i < (5.0 / 3.6)) && (target_td.dist < 10.0)))
	{
		if (!tcp.shift_knots)
		{
			shift_k1(tcp, target_td);
			tcp.shift_knots = true;
		}
	}
	else
	{
		if (tcp.shift_knots)
		{
			un_shift_k1(tcp, target_td);
			tcp.shift_knots = false;
		}
	}
}


vector<carmen_robot_and_trailer_path_point_t>
move_detailed_lane_to_front_axle(vector<carmen_robot_and_trailer_path_point_t> &detailed_lane)
{
	for (unsigned int i = 0; i < detailed_lane.size(); i++)
		detailed_lane[i] = (move_to_front_axle(detailed_lane[i]));

	return (detailed_lane);
}


double
get_path_to_lane_distance(TrajectoryLookupTable::TrajectoryDimensions td,
		TrajectoryLookupTable::TrajectoryControlParameters tcp, ObjectiveFunctionParams *my_params)
{
	vector<carmen_robot_and_trailer_path_point_t> path = simulate_car_from_parameters(td, tcp, my_params->target_td->v_i, my_params->target_td->phi_i, my_params->target_td->beta_i, false);
	double path_to_lane_distance = 0.0;
	if (my_params->use_lane && (my_params->detailed_lane.size() > 0) && (path.size() > 0))
	{
		compute_path_points_nearest_to_lane(my_params, path);
		path_to_lane_distance = compute_path_to_lane_distance(my_params, path);
	}
	return (path_to_lane_distance);
}


void
limit_tcp_phi(TrajectoryLookupTable::TrajectoryControlParameters &tcp)
{
	double max_phi_during_planning = 1.0 * GlobalState::robot_config.max_phi;
	double max_phi_during_planning2 = 1.0 * GlobalState::robot_config.max_phi;

	if (tcp.has_k1)
	{
		if (tcp.k1 > max_phi_during_planning)
			tcp.k1 = max_phi_during_planning;
		else if (tcp.k1 < -max_phi_during_planning)
			tcp.k1 = -max_phi_during_planning;
	}

	if (tcp.k2 > max_phi_during_planning2)
		tcp.k2 = max_phi_during_planning2;
	else if (tcp.k2 < -max_phi_during_planning2)
		tcp.k2 = -max_phi_during_planning2;

	if (tcp.k3 > max_phi_during_planning2)
		tcp.k3 = max_phi_during_planning2;
	else if (tcp.k3 < -max_phi_during_planning2)
		tcp.k3 = -max_phi_during_planning2;
}


TrajectoryLookupTable::TrajectoryControlParameters
get_dummy_tcp(const TrajectoryLookupTable::TrajectoryDimensions& td)
{
	TrajectoryLookupTable::TrajectoryControlParameters dummy_tcp;
	//type = 1 reh reto/ 2- reh curva a direita 2- reh curva a esquerda
	int tcp_type;
	if (td.theta > 0.3)
		tcp_type = 2;
	else if (td.theta < -0.3)
		tcp_type = 3;
	else
		tcp_type = 1;

	if (tcp_type == 1)
	{
		dummy_tcp.valid = true;
		dummy_tcp.tt = 2.5;;
		dummy_tcp.k1 = 0.0;
		dummy_tcp.k2 = 0.01;
		dummy_tcp.k3 = 0.02;
		dummy_tcp.has_k1 = true;
		dummy_tcp.shift_knots = false;
		dummy_tcp.a = -0.7;
		dummy_tcp.vf = -2.0;
		dummy_tcp.sf = td.dist;
		dummy_tcp.s = td.dist;
	}
	else if (tcp_type == 2)
	{
		dummy_tcp.valid = true;
		dummy_tcp.tt = 2.5;//4.2252857989082688;
		dummy_tcp.k1 = -0.17769236781390979;
		dummy_tcp.k2 = -1.3001704239163896;
		dummy_tcp.k3 = -0.20967541493688466;
		dummy_tcp.has_k1 = true;
		dummy_tcp.shift_knots = false;
		dummy_tcp.a = -0.71001114309832969;
		dummy_tcp.vf = -2.9999999999999991;
		dummy_tcp.sf = 6.3379786983624049;
		dummy_tcp.s = 6.3379286983624015;
	}
	else if (tcp_type == 3)
	{
		dummy_tcp.valid = true;
		dummy_tcp.tt = 2.5;//4.2252857989082688;
		dummy_tcp.k1 = 0.17769236781390979;
		dummy_tcp.k2 = 1.3001704239163896;
		dummy_tcp.k3 = 0.20967541493688466;
		dummy_tcp.has_k1 = true;
		dummy_tcp.shift_knots = false;
		dummy_tcp.a = -0.71001114309832969;
		dummy_tcp.vf = -2.9999999999999991;
		dummy_tcp.sf = 6.3379786983624049;
		dummy_tcp.s = 6.3379286983624015;
	}

	return dummy_tcp;
}


bool
get_tcp_from_td(TrajectoryLookupTable::TrajectoryControlParameters &tcp,
		TrajectoryLookupTable::TrajectoryControlParameters previous_tcp,
		TrajectoryLookupTable::TrajectoryDimensions td)
{
	if (!previous_tcp.valid)
	{
		TrajectoryLookupTable::TrajectoryDiscreteDimensions tdd = get_discrete_dimensions(td);
		if (!has_valid_discretization(tdd))
		{
			printf("Invalid discretization!!!!\n");
			return (false);
		}

		tcp = search_lookup_table(tdd);
		if (!tcp.valid)
		{
//			printf(KMAG "@@@@@@@@@@@ Could not find a valid entry in the table!!!!\n\033[0m");
			if (0)//(GlobalState::reverse_planning)
			{
				printf(KMAG "@@@@@@@@@@@ Trying a dummy_TCP\n");
				tcp = get_dummy_tcp(td);
				return (true);
			}
			return (false);
		}
		tcp.s = td.dist;
	}
	else
		tcp = previous_tcp;

	return (true);
}


bool
get_tcp_from_detailed_lane(TrajectoryLookupTable::TrajectoryControlParameters &tcp,
		vector<carmen_robot_and_trailer_path_point_t> detailed_lane,
		TrajectoryLookupTable::TrajectoryDimensions target_td)
{
	if (detailed_lane.size() < 3)
		return (false);

	tcp.valid = true;
	tcp.has_k1 = true;
	tcp.shift_knots = false;
	tcp.vf = target_td.v_i + tcp.a * tcp.tt;

	double knots_x[4] = {0.0, tcp.tt / 4.0, tcp.tt / 2.0, tcp.tt};
	double *knots_y[4] = {NULL, &tcp.k1, &tcp.k2, &tcp.k3};

	unsigned int j = 1;
	double s_consumed = 0.0;
	for (int i = 1; i < 4; i++)
	{
		double t = knots_x[i];
		double s = target_td.v_i * t + 0.5 * tcp.a * t * t;
		for ( ; j < detailed_lane.size() - 1; j++)
		{
			s_consumed += DIST2D(detailed_lane[j], detailed_lane[j + 1]);
			if (s_consumed > s)
				break;
		}
//		double theta1 = ANGLE2D(detailed_lane[j - 1], detailed_lane[j]);
//		double theta2 = ANGLE2D(detailed_lane[j], detailed_lane[j + 1]);
//		double delta_theta = carmen_normalize_theta(theta2 - theta1);
		double delta_theta = carmen_normalize_theta(detailed_lane[j + 1].theta - detailed_lane[j].theta);
		double l = DIST2D(detailed_lane[j], detailed_lane[j + 1]);
		double L = GlobalState::robot_config.distance_between_front_and_rear_axles;
//		double v = target_td.v_i + tcp.a * t;
		double phi = atan((L * delta_theta) / l);
//		double phi = atan((L * delta_theta) / v);
		if (tcp.vf < 0.0)
			phi = -phi;
		*knots_y[i] = phi;
	}

	tcp.sf = s_consumed;

	return (true);
}


#include "model/tree.h"
#include "publisher_util.h"

void
copy_path_to_traj(carmen_robot_and_trailer_traj_point_t *traj, vector<carmen_robot_and_trailer_path_point_t> path);

TrajectoryLookupTable::TrajectoryControlParameters
get_complete_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters previous_tcp,
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v,
		vector<carmen_robot_and_trailer_path_point_t> detailed_lane, bool use_lane)
{
	//	virtual_laser_message.num_positions = 0;

	ObjectiveFunctionParams params;
	params.use_lane = use_lane;
	if (detailed_lane.size() > 1)
	{
		if (GlobalState::reverse_planning)
			params.detailed_lane = detailed_lane;
		else
			params.detailed_lane = move_detailed_lane_to_front_axle(detailed_lane);
	}
	else
		params.use_lane = false;

	TrajectoryLookupTable::TrajectoryControlParameters tcp_seed;
	get_optimization_params(params, target_v, &tcp_seed, &target_td);
	compute_suitable_acceleration_and_tt(params, tcp_seed, target_td, target_v);

	if (!previous_tcp.valid)
	{
		if (!get_tcp_from_detailed_lane(tcp_seed, detailed_lane, target_td))
		{
			if (!get_tcp_from_td(tcp_seed, previous_tcp, target_td))
			{
				tcp_seed.valid = false;
				return (tcp_seed);
			}
		}
	}
	else
		tcp_seed = previous_tcp;

	TrajectoryLookupTable::TrajectoryControlParameters tcp_complete, tcp_copy;
	tcp_copy = tcp_seed;
//	if (previous_tcp.valid)
//		tcp_copy = tcp_complete = previous_tcp;
//	else
	{
		tcp_complete = get_optimized_trajectory_control_parameters(tcp_seed, target_td, params);
		// A funcaoacima nao usa k1
		get_missing_k1(tcp_complete, target_td);
	}

//	verify_shift_option_for_k1(tcp_complete, target_td);

	limit_tcp_phi(tcp_complete);

	bool optmize_time_and_acc = false;
	if (optmize_time_and_acc)
		tcp_complete = optimized_lane_trajectory_control_parameters_new(tcp_complete, target_td, target_v, params);
	else
		tcp_complete = optimized_lane_trajectory_control_parameters(tcp_complete, target_td, target_v, params);

//	if (tcp_complete.valid)
//	{
//		double dist = get_path_to_lane_distance(target_td, tcp_complete, &params);
//		printf("dist %lf %d\n", dist, (int) params.detailed_lane.size());
//		if (dist > GlobalState::max_square_distance_to_lane)
//			tcp_complete.valid = false;
//	}

//	if (target_td.dist < 2.0)
//	{
//		tcp_complete.k1 = tcp_copy.k1;
//		tcp_complete.k2 = tcp_copy.k2;
//		tcp_complete.k3 = tcp_copy.k3;
//	}
//	print_tcp(tcp_complete, params.plan_cost);
//	print_td(target_td, target_v);
//	printf("\n");
//	if (tcp_complete.tt < 0.0)
//		printf("t %.3lf, v0 %.1lf, a %.3lf, vg %.2lf, dg %.1lf, tt %.3lf\n",
//			carmen_get_time(), target_td.v_i, tcp_complete.a, target_v, tcp_complete.s, tcp_complete.tt);

//	TrajectoryLookupTable::TrajectoryDimensions td = target_td;
//	TrajectoryLookupTable::TrajectoryControlParameters tcp = tcp_complete;
//	tcp.valid = true;
//	vector<carmen_robot_and_trailer_path_point_t> path = simulate_car_from_parameters(td, tcp, td.v_i, td.phi_i, td.beta_i, false);
//	print_lane(path, (char *) "caco.txt");
//	vector<carmen_robot_and_trailer_path_point_t> path_prev = simulate_car_from_parameters(td, tcp_copy, td.v_i, td.phi_i, td.beta_i, false);
//
//	ObjectiveFunctionParams params_copy = params;
//	if (params_copy.detailed_lane.size() > 0)
//	{
//		compute_path_points_nearest_to_lane(&params_copy, path);
//		vector<carmen_robot_and_trailer_path_point_t> modified_path = compute_path_to_lane_distance_evaluation(&params_copy, path);
//		Tree tree;
//		tree.num_paths = 3;
//		tree.num_edges = 0;
//		tree.p1 = NULL;
//		tree.p2 = NULL;
//		tree.paths = (carmen_robot_and_trailer_traj_point_t **) malloc(tree.num_paths * sizeof(carmen_robot_and_trailer_traj_point_t *));
//		tree.paths_sizes = (int *) malloc(tree.num_paths * sizeof(int));
//
//		move_path_to_current_robot_pose(modified_path, GlobalState::localizer_pose);
//		tree.paths[0] = (carmen_robot_and_trailer_traj_point_t *) malloc(modified_path.size() * sizeof(carmen_robot_and_trailer_traj_point_t));
//		copy_path_to_traj(tree.paths[0], modified_path);
//		tree.paths_sizes[0] = (modified_path.size() >= CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE)? CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE - 1: modified_path.size();
//
//		move_path_to_current_robot_pose(path_prev, GlobalState::localizer_pose);
//		tree.paths[1] = (carmen_robot_and_trailer_traj_point_t *) malloc(path_prev.size() * sizeof(carmen_robot_and_trailer_traj_point_t));
//		copy_path_to_traj(tree.paths[1], path_prev);
//		tree.paths_sizes[1] = (path_prev.size() >= CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE)? CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE - 1: path_prev.size();
//
//		move_path_to_current_robot_pose(params_copy.detailed_lane, GlobalState::localizer_pose);
//		tree.paths[2] = (carmen_robot_and_trailer_traj_point_t *) malloc(params_copy.detailed_lane.size() * sizeof(carmen_robot_and_trailer_traj_point_t));
//		copy_path_to_traj(tree.paths[2], params_copy.detailed_lane);
//		tree.paths_sizes[2] = (params_copy.detailed_lane.size() >= CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE)? CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE - 1: params_copy.detailed_lane.size();
//
//		Publisher_Util::publish_plan_tree_message(tree);
//	}

//	printf("w1 %lf, w2 %lf, w3 %lf, w4 %lf, w5 %lf, w6 %lf\n",
//			GlobalState::w1, GlobalState::w2, GlobalState::w3, GlobalState::w4, GlobalState::w5, GlobalState::w6);

	return (tcp_complete);
}
