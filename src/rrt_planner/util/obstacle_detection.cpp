/*
 * obstacle_detection.cpp
 *
 *  Created on: 09/04/2012
 *      Author: romulo
 */

#include <carmen/collision_detection.h>
#include "obstacle_detection.h"
#include "util.h"
#include "../model/global_state.h"
#include "ackerman.h"
#include <math.h>

bool Obstacle_Detection::is_lane(Pose &global_pose)
{
	if (GlobalState::lane_map.complete_map)
		return !Obstacle_Detection::is_obstacle(global_pose, GlobalState::lane_map, 0.2);
	else
		return false;
}

bool Obstacle_Detection::is_obstacle(Pose &global_pose, carmen_map_t &map, double threshold)
{
	int vertical_size   = ceil(Util::to_map_unit(GlobalState::robot_config.length));
	int horizontal_size = ceil(Util::to_map_unit(GlobalState::robot_config.width) / 2.0);
	int distance_between_rear_car_and_rear_wheels = ceil(Util::to_map_unit(GlobalState::robot_config.distance_between_rear_car_and_rear_wheels));

	Pose   vertical_pose, horizontal_pose[2], pose;
	double delta_vertical_x, delta_vertical_y, delta_horizontal_x, delta_horizontal_y;

	pose = Util::to_map_pose(global_pose);

	delta_vertical_x = cos(pose.theta);
	delta_vertical_y = sin(pose.theta);

	delta_horizontal_x = cos(M_PI / 2.0 - pose.theta);
	delta_horizontal_y = sin(M_PI / 2.0 - pose.theta);

	//a pose do robo originalmente está entre as rodas
	//por isso deve ser transladada para a traseira do carro
	pose.x -= distance_between_rear_car_and_rear_wheels * delta_vertical_x;
	pose.y -= distance_between_rear_car_and_rear_wheels * delta_vertical_y;

	vertical_pose = pose;

	for (int v = 0; v <= vertical_size; v++)
	{
		horizontal_pose[0] = vertical_pose;
		horizontal_pose[1] = vertical_pose;

		for (int h = 0; h <= horizontal_size; h++)
		{
			for (int i = 0; i < 2; i++)
			{
				if (horizontal_pose[i].x < 0 || horizontal_pose[i].x > map.config.x_size ||
						horizontal_pose[i].y < 0 || horizontal_pose[i].y > map.config.y_size)
					return false;

				if (map.complete_map[(int)horizontal_pose[i].x * map.config.y_size + (int)horizontal_pose[i].y] > threshold)
					return true;
			}

			horizontal_pose[0].x = horizontal_pose[0].x - delta_horizontal_x;
			horizontal_pose[0].y = horizontal_pose[0].y + delta_horizontal_y;

			horizontal_pose[1].x = horizontal_pose[1].x + delta_horizontal_x;
			horizontal_pose[1].y = horizontal_pose[1].y - delta_horizontal_y;
		}

		vertical_pose.x = vertical_pose.x + delta_vertical_x;
		vertical_pose.y = vertical_pose.y + delta_vertical_y;
	}

	return false;
}

bool Obstacle_Detection::is_obstacle(Pose &global_pose)
{
	int vertical_size   = ceil(Util::to_map_unit(GlobalState::robot_config.length));
	int horizontal_size = ceil(Util::to_map_unit(GlobalState::robot_config.width) / 2.0);
	int distance_between_rear_car_and_rear_wheels = ceil(Util::to_map_unit(GlobalState::robot_config.distance_between_rear_car_and_rear_wheels));

	Pose   vertical_pose, horizontal_pose[2], pose;
	double delta_vertical_x, delta_vertical_y, delta_horizontal_x, delta_horizontal_y;

	pose = Util::to_map_pose(global_pose);

	delta_vertical_x = cos(pose.theta);
	delta_vertical_y = sin(pose.theta);

	delta_horizontal_x = cos(M_PI / 2.0 - pose.theta);
	delta_horizontal_y = sin(M_PI / 2.0 - pose.theta);

	//a pose do robo originalmente está entre as rodas
	//por isso deve ser transladada para a traseira do carro
	pose.x -= distance_between_rear_car_and_rear_wheels * delta_vertical_x;
	pose.y -= distance_between_rear_car_and_rear_wheels * delta_vertical_y;

	vertical_pose = pose;

	for (int v = 0; v <= vertical_size; v++)
	{
		horizontal_pose[0] = vertical_pose;
		horizontal_pose[1] = vertical_pose;

		for (int h = 0; h <= horizontal_size; h++)
		{
			for (int i = 0; i < 2; i++)
			{
				if (is_obstacle_point(horizontal_pose[i]))
					return true;
			}

			horizontal_pose[0].x = horizontal_pose[0].x - delta_horizontal_x;
			horizontal_pose[0].y = horizontal_pose[0].y + delta_horizontal_y;

			horizontal_pose[1].x = horizontal_pose[1].x + delta_horizontal_x;
			horizontal_pose[1].y = horizontal_pose[1].y - delta_horizontal_y;
		}

		vertical_pose.x = vertical_pose.x + delta_vertical_x;
		vertical_pose.y = vertical_pose.y + delta_vertical_y;
	}

	return false;
}

bool Obstacle_Detection::is_obstacle_path(RRT_Node &node, Command &command, double interval_time)
{
	unsigned long int key;
	bool result;

	key = command.get_key(interval_time);
	map<unsigned long int, bool>::iterator it = node.obstacle_verification_cache.find(key);

	if (it == node.obstacle_verification_cache.end())
	{
		result = is_obstacle_path2(node.robot_state, command, interval_time);
		node.obstacle_verification_cache[key] = result;
	}
	else
	{
		result = it->second;
	}

	return result;
}

static bool
car_border_verification(Pose &global_pose)
{
	carmen_point_t robot_pose = Util::convert_to_carmen_point_t(global_pose);

	double max_occupancy = carmen_obstacle_avoider_get_maximum_occupancy_of_map_cells_hit_by_robot_border(
				&robot_pose, &GlobalState::cost_map,
				GlobalState::robot_config.length, GlobalState::robot_config.width,
				GlobalState::robot_config.distance_between_rear_car_and_rear_wheels);

	if (max_occupancy > 0.5)
		return (true);
	else
		return (false);
}


bool
Obstacle_Detection::is_obstacle_path2(Robot_State &robot_state, const Command &requested_command, double full_time_interval)
{
	double delta_t = 0.09;

	int n = floor(full_time_interval / delta_t);
	double remaining_time = full_time_interval - ((double) n * delta_t);
	Robot_State achieved_robot_state = robot_state; // achieved_robot_state eh computado iterativamente abaixo a partir do estado atual do robo

#ifdef OLD_STEERING_CONTROL
	double curvature = carmen_get_curvature_from_phi(
			requested_command.phi, requested_command.v,
			GlobalState::robot_config.understeer_coeficient,
			GlobalState::robot_config.distance_between_front_and_rear_axles);

	double new_curvature = carmen_get_curvature_from_phi(
			achieved_robot_state.v_and_phi.phi, achieved_robot_state.v_and_phi.v,
			GlobalState::robot_config.understeer_coeficient,
			GlobalState::robot_config.distance_between_front_and_rear_axles);

	double max_curvature_change = GlobalState::robot_config.desired_steering_command_rate * delta_t;
#else
	double max_phi_velocity = GlobalState::max_phi_velocity;
	double max_phi_acceleration = GlobalState::max_phi_acceleration;

	double phi_velocity = 0.0;
	// O codigo abaixo assume phi_velocity == 0.0 no início do full_time_interval
	double t_fim_descida;
	double t_fim_plato;
	double t_fim_subida;
	Ackerman::compute_intermediate_times(t_fim_subida, t_fim_plato, t_fim_descida, max_phi_acceleration, robot_state.v_and_phi.phi, requested_command.phi,
			full_time_interval,	max_phi_velocity);
#endif

	// Euler method
	for (int i = 0; i < n; i++)
	{
#ifdef OLD_STEERING_CONTROL
		Ackerman::predict_next_pose_during_main_rrt_planning_step(achieved_robot_state, requested_command, delta_t,
				new_curvature, curvature, max_curvature_change);
#else
		double t = (double) i * delta_t;
		Ackerman::predict_next_pose_during_main_rrt_planning_step(achieved_robot_state, requested_command, phi_velocity,
				t, delta_t,
				t_fim_descida, t_fim_plato, t_fim_subida,
				max_phi_velocity, max_phi_acceleration);
#endif

		if (car_border_verification(achieved_robot_state.pose))
			return true;
	}

	if (remaining_time > 0.0)
	{
#ifdef OLD_STEERING_CONTROL
		Ackerman::predict_next_pose_during_main_rrt_planning_step(achieved_robot_state, requested_command, delta_t,
				new_curvature, curvature, max_curvature_change);
#else
		double t = (double) n * delta_t;
		Ackerman::predict_next_pose_during_main_rrt_planning_step(achieved_robot_state, requested_command, phi_velocity,
				t, remaining_time,
				t_fim_descida, t_fim_plato, t_fim_subida,
				max_phi_velocity, max_phi_acceleration);
#endif

		if (car_border_verification(achieved_robot_state.pose))
			return true;
	}

	return false;
}

bool Obstacle_Detection::is_obstacle_path(Pose &global_pose, const Command &command, double interval_time)
{
	static double distance_interval = 0.8;
	double distance, distance_time;
	int	   size;
	Pose   p;

	p = Ackerman::predict_next_pose(global_pose, command, interval_time);

	//verify final pose
	if (is_obstacle(p))
	{
		return true;
	}

	distance = global_pose.distance(p);
	distance_time = min((distance_interval * interval_time) / distance, interval_time);
	size = ceil(interval_time / distance_time) - 1;

	p = global_pose;

	for (int i = 1; i < size; i++)
	{
		p = Ackerman::predict_next_pose(p, command, distance_time);

		if (is_obstacle(p))
		{
			return true;
		}
	}

	return false;
}

bool Obstacle_Detection::is_obstacle_point(Pose &pose)
{
	return is_obstacle_point((int)pose.x, (int)pose.y);
}

bool Obstacle_Detection::is_obstacle_point(const int &x, const int &y)
{
	if (!Util::is_valid_position(x, y))
		return true; //invalid position

	if (GlobalState::cost_map.map[x][y] > GlobalState::obstacle_threshold)
		return true; //obstacle

	return false; //free
}

