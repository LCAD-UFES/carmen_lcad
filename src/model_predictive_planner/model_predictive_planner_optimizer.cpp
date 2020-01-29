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

#include "model/robot_state.h"
#include "model/global_state.h"
#include "util.h"
#include "model_predictive_planner_optimizer.h"

bool use_obstacles = true;

//extern carmen_mapper_virtual_laser_message virtual_laser_message;


void
compute_a_and_t_from_s(double s, double target_v,
		TrajectoryLookupTable::TrajectoryDimensions target_td,
		TrajectoryLookupTable::TrajectoryControlParameters &tcp_seed,
		ObjectiveFunctionParams *params)
{
	// https://www.wolframalpha.com/input/?i=solve+s%3Dv*x%2B0.5*a*x%5E2
	double a = (target_v * target_v - target_td.v_i * target_td.v_i) / (2.0 * s);
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

//	printf("s %.1lf, a %.3lf, t %.1lf, tv %.1lf, vi %.1lf\n", s, a, tcp_seed.tt, target_v, target_td.v_i);
	params->suitable_tt = tcp_seed.tt;
	params->suitable_acceleration = tcp_seed.a = a;
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
			double v = params->target_td->v_i;
			if (tcp.a > 0.0)
			{
				tcp.tt = (sqrt(2.0 * tcp.a * tcp.s + v * v) - v) / tcp.a;
			}
			else if (tcp.a < 0.0)
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
	tcp.vf = params->tcp_seed->vf;
	tcp.sf = params->tcp_seed->sf;

	if (tcp.tt < 0.2) // o tempo nao pode ser pequeno demais
		tcp.tt = 0.2;
	if (tcp.a < -GlobalState::robot_config.maximum_deceleration_forward) // a aceleracao nao pode ser negativa demais
		tcp.a = -GlobalState::robot_config.maximum_deceleration_forward;
	if (tcp.s < 0.2)
		tcp.s = 0.2;
//
//	if (tcp.a > GlobalState::robot_config.maximum_acceleration_forward) // a aceleracao nao pode ser positiva demais
//		tcp.a = GlobalState::robot_config.maximum_acceleration_forward;

//	double max_phi_during_planning = 1.8 * GlobalState::robot_config.max_phi;
//	if (tcp.has_k1)
//	{
//		if (tcp.k1 > max_phi_during_planning)
//			tcp.k1 = max_phi_during_planning;
//		else if (tcp.k1 < -max_phi_during_planning)
//			tcp.k1 = -max_phi_during_planning;
//	}
//
//	if (tcp.k2 > max_phi_during_planning)
//		tcp.k2 = max_phi_during_planning;
//	else if (tcp.k2 < -max_phi_during_planning)
//		tcp.k2 = -max_phi_during_planning;
//
//	if (tcp.k3 > max_phi_during_planning)
//		tcp.k3 = max_phi_during_planning;
//	else if (tcp.k3 < -max_phi_during_planning)
//		tcp.k3 = -max_phi_during_planning;

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


inline
double
dist(carmen_ackerman_path_point_t v, carmen_ackerman_path_point_t w)
{
	return sqrt((carmen_square(v.x - w.x) + carmen_square(v.y - w.y)));
}


inline
double
ortho_dist(carmen_ackerman_path_point_t v, carmen_ackerman_path_point_t w)
{
	double d_x = v.x - w.x;
	double d_y = v.y - w.y;
	double theta = atan2(d_y, d_x);
	double d = sqrt(d_x * d_x + d_y * d_y) * sin(fabs(v.theta - theta));
	return (d);
}


inline
double
dist2(carmen_ackerman_path_point_t v, carmen_ackerman_path_point_t w)
{
	return (carmen_square(v.x - w.x) + carmen_square(v.y - w.y));
}


carmen_ackerman_path_point_t
move_to_front_axle(carmen_ackerman_path_point_t pose)
{
	double L = GlobalState::robot_config.distance_between_front_and_rear_axles;
	carmen_ackerman_path_point_t pose_moved = pose;
	pose_moved.x += L * cos(pose.theta);
	pose_moved.y += L * sin(pose.theta);

	return (pose_moved);
}


double
compute_path_to_lane_distance(ObjectiveFunctionParams *my_params, vector<carmen_ackerman_path_point_t> &path)
{
	double distance = 0.0;
	double total_distance = 0.0;
	double total_points = 0.0;

	for (unsigned int i = 0; i < path.size(); i += 3)
	{
		if ((i < my_params->path_point_nearest_to_lane.size()) &&
			(my_params->path_point_nearest_to_lane.at(i) < my_params->detailed_lane.size()))
		{
			distance = dist(move_to_front_axle(path.at(i)),
					my_params->detailed_lane.at(my_params->path_point_nearest_to_lane.at(i)));
//			distance = dist(path.at(i),
//					my_params->detailed_lane.at(my_params->path_point_nearest_to_lane.at(i)));
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


void
compute_path_points_nearest_to_lane(ObjectiveFunctionParams *param, vector<carmen_ackerman_path_point_t> &path)
{
	param->path_point_nearest_to_lane.clear();
	param->path_size = path.size();
	for (unsigned int j = 0; j < path.size(); j++)
	{
		// TODO: Alberto @@@ tratar disso quando estivermos dando reh tambem
		if (path.at(j).v < 0.0)
			continue;

		carmen_ackerman_path_point_t front_axle = move_to_front_axle(path.at(j));

		// consider the first point as the nearest one
		unsigned int index = 0;
		double min_dist = dist(front_axle, param->detailed_lane.at(index));
//		double min_dist = dist(path.at(j), param->detailed_lane.at(index));

		for (unsigned int i = 1; i < param->detailed_lane.size(); i++)
		{
			double distance = dist(front_axle, param->detailed_lane.at(i));
//			double distance = dist(path.at(j), param->detailed_lane.at(i));

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
compute_proximity_to_obstacles_using_distance_map(vector<carmen_ackerman_path_point_t> path)
{
	double proximity_to_obstacles_for_path = 0.0;
	double safety_distance = GlobalState::robot_config.model_predictive_planner_obstacles_safe_distance;
	carmen_point_t localizer = {GlobalState::localizer_pose->x, GlobalState::localizer_pose->y, GlobalState::localizer_pose->theta};

	for (unsigned int i = 0; i < path.size(); i += 1)
	{
		carmen_point_t point_to_check = {path[i].x, path[i].y, path[i].theta};
		double proximity_point = carmen_obstacle_avoider_compute_car_distance_to_closest_obstacles(&localizer,
				point_to_check, GlobalState::robot_config, GlobalState::distance_map, safety_distance);
		proximity_to_obstacles_for_path += proximity_point;
//		carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, carmen_get_time());
//		getchar();
	}

	return (proximity_to_obstacles_for_path);
}

//double
//compute_collision_with_moving_obstacles(const vector<carmen_ackerman_path_point_t> &path, double car_velocity)
//{
//    double intersection_with_obstacles = 0.0;
//    int num_path_points = path.size();
//    int num_objects = GlobalState::moving_objects_trajectories.size();
//    carmen_oriented_bounding_box car;
//    car.length = GlobalState::robot_config.model_predictive_planner_obstacles_safe_length_distance;
//    car.width = 2 * GlobalState::robot_config.model_predictive_planner_obstacles_safe_distance;
//    car.linear_velocity = car_velocity;
//
//    for (int i = 0; i < num_path_points; i++) {
//        // Planning is done relative to car front axis, but we need the center point instead
//        double half_axis_distance = 0.5 * GlobalState::robot_config.distance_between_front_and_rear_axles;
//        car.object_pose.x = path[i].x - half_axis_distance * cos(path[i].theta);
//        car.object_pose.y = path[i].y - half_axis_distance * sin(path[i].theta);
//
//        car.orientation = path[i].theta;
//
//        for (int j = 0; j < num_objects; j++) {
//            // Broad phase
//            if ((carmen_square(GlobalState::moving_objects_trajectories[j][i].x - path[i].x) +
//                 carmen_square(GlobalState::moving_objects_trajectories[j][i].y - path[i].y)) > 25) {
//                continue;
//            }
//
//            // Narrow phase
//            carmen_oriented_bounding_box object;
//            object.object_pose.x = GlobalState::moving_objects_trajectories[j][i].x;
//            object.object_pose.y = GlobalState::moving_objects_trajectories[j][i].y;
//            object.orientation = GlobalState::moving_objects_trajectories[j][i].theta;
//            object.length = GlobalState::robot_config.length;
//            object.width = GlobalState::robot_config.width;
//            object.linear_velocity = GlobalState::moving_objects_trajectories[j][i].v;
//
//            intersection_with_obstacles += compute_collision_obb_obb(car, object);
//
//            //if (intersection_with_obstacles > 0)
//            //    return intersection_with_obstacles;
//        }
//    }
//
//    return intersection_with_obstacles;
//
//}

double
my_f(const gsl_vector *x, void *params)
{
	ObjectiveFunctionParams *my_params = (ObjectiveFunctionParams *) params;

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(x, my_params);
	TrajectoryLookupTable::TrajectoryDimensions td;

	vector<carmen_ackerman_path_point_t> path = simulate_car_from_parameters(td, tcp, my_params->target_td->v_i, my_params->target_td->phi_i, false);
	my_params->tcp_seed->vf = tcp.vf;
	my_params->tcp_seed->sf = tcp.sf;

	double result;
	if (((ObjectiveFunctionParams *) (params))->optimize_time == OPTIMIZE_DISTANCE)
		result = ((td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index +
			(carmen_normalize_theta(td.theta) - my_params->target_td->theta) * (carmen_normalize_theta(td.theta) - my_params->target_td->theta) / (my_params->theta_by_index * 2.0) +
			(carmen_normalize_theta(td.d_yaw) - my_params->target_td->d_yaw) * (carmen_normalize_theta(td.d_yaw) - my_params->target_td->d_yaw) / (my_params->d_yaw_by_index * 2.0));
	else
		result = sqrt((td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index +
			(carmen_normalize_theta(td.theta) - my_params->target_td->theta) * (carmen_normalize_theta(td.theta) - my_params->target_td->theta) / (my_params->theta_by_index * 2.0) +
			(carmen_normalize_theta(td.d_yaw) - my_params->target_td->d_yaw) * (carmen_normalize_theta(td.d_yaw) - my_params->target_td->d_yaw) / (my_params->d_yaw_by_index * 2.0));

	my_params->plan_cost = result;

	return (result);
}


void
my_df(const gsl_vector *v, void *params, gsl_vector *df)
{
	double h;

	double f_x = my_f(v, params);

	if (((ObjectiveFunctionParams *) (params))->optimize_time == OPTIMIZE_DISTANCE)
		h = 0.00005;
	else
		h = 0.0002;

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
	double d_f_z_h;
	if (((ObjectiveFunctionParams *) (params))->optimize_time == OPTIMIZE_DISTANCE)
		d_f_z_h = 150.0 * (f_z_h - f_x) / h;
	else
		d_f_z_h = (f_z_h - f_x) / h;

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

	vector<carmen_ackerman_path_point_t> path = simulate_car_from_parameters(td, tcp, my_params->target_td->v_i, my_params->target_td->phi_i, false);

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

    //TODO: essa parte do modulo esta em desenvolvimento ainda
//    double distance_to_moving_obstacles = 0.0;
//    if(!path.empty())
//    {
//        distance_to_moving_obstacles = compute_collision_with_moving_obstacles(path, my_params->target_td->v_i);
//    }

	my_params->tcp_seed->vf = tcp.vf;
	my_params->tcp_seed->sf = tcp.sf;

	my_params->plan_cost = sqrt((td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index +
			(carmen_normalize_theta(td.theta) - my_params->target_td->theta) * (carmen_normalize_theta(td.theta) - my_params->target_td->theta) / (my_params->theta_by_index * 0.2) +
			(carmen_normalize_theta(td.d_yaw) - my_params->target_td->d_yaw) * (carmen_normalize_theta(td.d_yaw) - my_params->target_td->d_yaw) / (my_params->d_yaw_by_index * 0.2));

	double result;
	if (((ObjectiveFunctionParams *) (params))->optimize_time == OPTIMIZE_DISTANCE)
	{
		if (td.dist < 7.0)
			GlobalState::w2 *= exp(td.dist - 7.0);  // Tries to smooth the steering wheel behavior near to stop
		result = (
				GlobalState::w1 * (td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index +   // Distance from the last pose of the path to the goal (in polar coordinates)
				GlobalState::w2 * (carmen_normalize_theta(td.theta - my_params->target_td->theta) * carmen_normalize_theta(td.theta - my_params->target_td->theta)) / my_params->theta_by_index +  // Angular distance from the last pose of the path to the goal (in polar coordinates)
				GlobalState::w3 * (carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw) * carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw)) / my_params->d_yaw_by_index +  // Angular distance from last pose of the path car orientation to the goal car orientation
				GlobalState::w4 * path_to_lane_distance + // já é quandrática
				GlobalState::w5 * proximity_to_obstacles + // já é quandrática
				GlobalState::w6 * tcp.sf * tcp.sf); // + path_size // traveled_distance
               // w7 * distance_to_moving_obstacles);
	}
	else
	{
		if (td.dist < 7.0)
			GlobalState::w2 *= exp(td.dist - 7.0);
		result = sqrt(
				GlobalState::w1 * (td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index +
				GlobalState::w2 * (carmen_normalize_theta(td.theta - my_params->target_td->theta) * carmen_normalize_theta(td.theta - my_params->target_td->theta)) / my_params->theta_by_index +
				GlobalState::w3 * (carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw) * carmen_normalize_theta(td.d_yaw - my_params->target_td->d_yaw)) / my_params->d_yaw_by_index +
				GlobalState::w4 * path_to_lane_distance + // já é quandrática
				GlobalState::w5 * proximity_to_obstacles + // já é quandrática
				GlobalState::w6 * tcp.sf * tcp.sf);
	}

//	printf("result %lf sf %.2lf, tdc %.2lf, tdd %.2f, a %.2lf delta_V %.2lf \n", result, tcp.sf, td.dist,
//			my_params->target_td->dist, tcp.a, (my_params->target_v - tcp.vf) * (my_params->target_v - tcp.vf));
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
	double d_g_w_h;
	if (((ObjectiveFunctionParams *) (params))->optimize_time == OPTIMIZE_DISTANCE)
		d_g_w_h = 200.0 * (g_w_h - g_x) / h;
	else
		d_g_w_h = (g_w_h - g_x) / h;

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
my_h(const gsl_vector *x, void *params)
{
	ObjectiveFunctionParams *my_params = (ObjectiveFunctionParams *) params;

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(x, my_params);
	TrajectoryLookupTable::TrajectoryDimensions td;

	vector<carmen_ackerman_path_point_t> path = simulate_car_from_parameters(td, tcp, my_params->target_td->v_i, my_params->target_td->phi_i, false);

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

	my_params->tcp_seed->vf = tcp.vf;
	my_params->tcp_seed->sf = tcp.sf;

	my_params->plan_cost = sqrt((td.dist - my_params->target_td->dist) * (td.dist - my_params->target_td->dist) / my_params->distance_by_index +
			(carmen_normalize_theta(td.theta) - my_params->target_td->theta) * (carmen_normalize_theta(td.theta) - my_params->target_td->theta) / (my_params->theta_by_index * 0.2) +
			(carmen_normalize_theta(td.d_yaw) - my_params->target_td->d_yaw) * (carmen_normalize_theta(td.d_yaw) - my_params->target_td->d_yaw) / (my_params->d_yaw_by_index * 0.2));

	double w1, w2, w3, w4, w5, w6, result;
	if (((ObjectiveFunctionParams *) (params))->optimize_time == OPTIMIZE_DISTANCE)
	{
		w1 = 30.0; w2 = 15.0; w3 = 15.0; w4 = 3.0; w5 = 3.0; w6 = 0.005;
		if (td.dist < 7.0)
			w2 *= exp(td.dist - 7.0);
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
		if (td.dist < 7.0)
			w2 *= exp(td.dist - 7.0);
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
	double d_g_w_h;
	if (((ObjectiveFunctionParams *) (params))->optimize_time == OPTIMIZE_DISTANCE)
		d_g_w_h = 200.0 * (g_w_h - g_x) / h;
	else
		d_g_w_h = (g_w_h - g_x) / h;

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


/* Compute both g and df together. for while df equal to dg */
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

	if (target_v < 0.0)
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
		else if (tt < 0.15)
			tt = 0.15;

		params.suitable_tt = tcp_seed.tt = tt;
		params.suitable_acceleration = tcp_seed.a = a;
		//printf("SUITABLE a %lf, tt %lf\n", a, tt);
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
	if (tcp_seed.shift_knots)
		tcp_seed.k1 = gsl_spline_eval(phi_spline, 3.0 * (tcp_seed.tt / 4.0), acc);
	else
		tcp_seed.k1 = gsl_spline_eval(phi_spline, tcp_seed.tt / 4.0, acc);
	tcp_seed.has_k1 = true;
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
	//	vector<carmen_ackerman_path_point_t> path = simulate_car_from_parameters(target_td, tcp_seed, target_td.v_i, target_td.phi_i, false);

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
	//print_tcp(tcp_seed);
	//print_td(target_td);
	//printf("tv = %lf\n", target_v);

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
	gsl_multimin_fdfminimizer_set(s, &my_func, x, 0.005, 0.1);

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
		//		print_lane(simulate_car_from_parameters(target_td, tcp_temp, target_td.v_i, target_td.phi_i, true), path_file);
		//		fclose(path_file);
		//		printf("Estou na: %lu iteracao, sf: %lf  \n", iter, s->f);
		//		getchar();
		//	--
//		params.suitable_acceleration = compute_suitable_acceleration(gsl_vector_get(x, 3), target_td, target_v);

	} while (/*(s->f > MAX_LANE_DIST) &&*/ (status == GSL_CONTINUE) && (iter < 50));

//	static int xx = 0;
//	printf("iter = %02ld, %d\n", iter, xx++);

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(s->x, &params);

	if (tcp.tt < 0.2)
	{
//		printf(">>>>>>>>>>>>>> tt < 0.2\n");
		tcp.valid = false;
	}

//	printf("lane plan_cost = %lf\n", params.plan_cost);
	if (params.plan_cost > 0.5)
	{
//		printf(">>>>>>>>>>>>>> lane plan_cost > 3.6\n");
		tcp.valid = false;
	}

//	if (tcp.valid == false) // Para achar bugs
//	{
//		printf("  ");
//	}

	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

	return (tcp);
}

TrajectoryLookupTable::TrajectoryControlParameters
optimized_lane_trajectory_control_parameters_new(TrajectoryLookupTable::TrajectoryControlParameters &tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v, ObjectiveFunctionParams params)
{
	//	vector<carmen_ackerman_path_point_t> path = simulate_car_from_parameters(target_td, tcp_seed, target_td.v_i, target_td.phi_i, false);

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
	gsl_multimin_fdfminimizer_set(s, &my_func, x, 0.005, 0.1);

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
		//		print_lane(simulate_car_from_parameters(target_td, tcp_temp, target_td.v_i, target_td.phi_i, true), path_file);
		//		fclose(path_file);
		//		printf("Estou na: %lu iteracao, sf: %lf  \n", iter, s->f);
		//		getchar();
		//	--
//		params.suitable_acceleration = compute_suitable_acceleration(gsl_vector_get(x, 3), target_td, target_v);

	} while (/*(s->f > MAX_LANE_DIST) &&*/ (status == GSL_CONTINUE) && (iter < 50));

//	printf("iter = %ld\n", iter);

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(s->x, &params);

	if (tcp.tt < 0.2 || s->f > 20.0)
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
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v, ObjectiveFunctionParams &params,
		bool has_previous_good_tcp)
{
	get_optimization_params(target_v, tcp_seed, target_td, params);
	compute_suitable_acceleration_and_tt(params, tcp_seed, target_td, target_v);

	if (has_previous_good_tcp)
		return (tcp_seed);

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

	gsl_multimin_fdfminimizer_set(s, &my_func, x, 0.005, 0.1);

	size_t iter = 0;
	int status;
	do
	{
		iter++;

		status = gsl_multimin_fdfminimizer_iterate(s);
		if (status == GSL_ENOPROG) // minimizer is unable to improve on its current estimate, either due to numerical difficulty or a genuine local minimum
			break;

		status = gsl_multimin_test_gradient(s->gradient, 0.16); // esta funcao retorna GSL_CONTINUE ou zero

	} while ((params.plan_cost > 0.005) && (status == GSL_CONTINUE) && (iter < 30));

	TrajectoryLookupTable::TrajectoryControlParameters tcp = fill_in_tcp(s->x, &params);

	if ((tcp.tt < 0.2) || (params.plan_cost > 0.1)) // too short plan or bad minimum (s->f should be close to zero) mudei de 0.05 para outro
	{
//		printf("################## plan_cost = %lf\n", params.plan_cost);
		tcp.valid = false;
	}

	if (target_td.dist < 3.0 && tcp.valid == false) // para debugar
		tcp.valid = false;

	gsl_multimin_fdfminimizer_free(s);
	gsl_vector_free(x);

	//	if (tcp.valid)
	//	{
	//		print_path(simulate_car_from_parameters(target_td, tcp, target_td.v_i, target_td.phi_i, false));
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


void
shift_k1(TrajectoryLookupTable::TrajectoryControlParameters &tcp_seed, const TrajectoryLookupTable::TrajectoryDimensions target_td)
{
	double knots_x[4] = { 0.0, tcp_seed.tt / 4.0, tcp_seed.tt / 2.0, tcp_seed.tt };
	double knots_y[4];
	knots_y[0] = target_td.phi_i;
	knots_y[1] = tcp_seed.k1;
	knots_y[2] = tcp_seed.k2;
	knots_y[3] = tcp_seed.k3;
	gsl_interp_accel* acc = gsl_interp_accel_alloc();
	const gsl_interp_type* type = gsl_interp_cspline;
	gsl_spline* phi_spline = gsl_spline_alloc(type, 4);
	gsl_spline_init(phi_spline, knots_x, knots_y, 4);
	tcp_seed.k1 = gsl_spline_eval(phi_spline, 3.0 * (tcp_seed.tt / 4.0), acc);

	gsl_spline_free(phi_spline);
	gsl_interp_accel_free(acc);
}


void
un_shift_k1(TrajectoryLookupTable::TrajectoryControlParameters &tcp_seed, const TrajectoryLookupTable::TrajectoryDimensions target_td)
{
	double knots_x[4] = { 0.0, tcp_seed.tt / 2.0, 3.0 * (tcp_seed.tt / 4.0), tcp_seed.tt };
	double knots_y[4];
	knots_y[0] = target_td.phi_i;
	knots_y[1] = tcp_seed.k2;
	knots_y[2] = tcp_seed.k1;
	knots_y[3] = tcp_seed.k3;
	gsl_interp_accel* acc = gsl_interp_accel_alloc();
	const gsl_interp_type* type = gsl_interp_cspline;
	gsl_spline* phi_spline = gsl_spline_alloc(type, 4);
	gsl_spline_init(phi_spline, knots_x, knots_y, 4);
	tcp_seed.k1 = gsl_spline_eval(phi_spline, tcp_seed.tt / 4.0, acc);

	gsl_spline_free(phi_spline);
	gsl_interp_accel_free(acc);
}


void
verify_shift_option_for_k1(TrajectoryLookupTable::TrajectoryControlParameters& tcp_seed,
		const TrajectoryLookupTable::TrajectoryDimensions& target_td,
		TrajectoryLookupTable::TrajectoryControlParameters& tcp_complete)
{
	if (!tcp_seed.has_k1)
		get_missing_k1(target_td, tcp_seed);

	if (((target_td.v_i > (18.0 / 3.6)) && (target_td.dist < 20.0))	||
		((target_td.v_i < (5.0 / 3.6)) && (target_td.dist < 10.0)))
	{
		if (!tcp_complete.shift_knots)
		{
			shift_k1(tcp_complete, target_td);
			tcp_complete.shift_knots = true;
		}
	}
	else
	{
		if (tcp_complete.shift_knots)
		{
			un_shift_k1(tcp_complete, target_td);
			tcp_complete.shift_knots = false;
		}
	}
}


vector<carmen_ackerman_path_point_t>
move_detailed_lane_to_front_axle(vector<carmen_ackerman_path_point_t> &detailed_lane)
{
	for (unsigned int i = 0; i < detailed_lane.size(); i++)
		detailed_lane[i] = (move_to_front_axle(detailed_lane[i]));

	return (detailed_lane);
}


double
get_path_to_lane_distance(TrajectoryLookupTable::TrajectoryDimensions td,
		TrajectoryLookupTable::TrajectoryControlParameters tcp, ObjectiveFunctionParams *my_params)
{
	vector<carmen_ackerman_path_point_t> path = simulate_car_from_parameters(td, tcp, my_params->target_td->v_i, my_params->target_td->phi_i, false);
	double path_to_lane_distance = 0.0;
	if (my_params->use_lane && (my_params->detailed_lane.size() > 0) && (path.size() > 0))
	{
		compute_path_points_nearest_to_lane(my_params, path);
		path_to_lane_distance = compute_path_to_lane_distance(my_params, path);
	}
	return (path_to_lane_distance);
}


TrajectoryLookupTable::TrajectoryControlParameters
get_complete_optimized_trajectory_control_parameters(TrajectoryLookupTable::TrajectoryControlParameters tcp_seed,
		TrajectoryLookupTable::TrajectoryDimensions target_td, double target_v, vector<carmen_ackerman_path_point_t> detailed_lane,
		bool use_lane, bool has_previous_good_tcp)
{
	TrajectoryLookupTable::TrajectoryControlParameters tcp_complete, tcp_copy;
	ObjectiveFunctionParams params;
	params.detailed_lane = move_detailed_lane_to_front_axle(detailed_lane);
//	params.detailed_lane = detailed_lane;
	params.use_lane = use_lane;

	bool optmize_time_and_acc = false;

//	virtual_laser_message.num_positions = 0;

	tcp_copy = tcp_complete = get_optimized_trajectory_control_parameters(tcp_seed, target_td, target_v, params, has_previous_good_tcp);

//	verify_shift_option_for_k1(tcp_seed, target_td, tcp_complete);
	// Atencao: params.suitable_acceleration deve ser preenchido na funcao acima para que nao seja alterado no inicio da otimizacao abaixo

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

	return (tcp_complete);
}
