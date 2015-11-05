/*
 * cost_function.cpp
 *
 *  Created on: 22/02/2013
 *      Author: romulo
 */

#include <carmen/collision_detection.h>
#include "cost_function.h"
#include "ackerman.h"
#include "util.h"

double
Distance_Cost_Function::get_cost()
{
	return new_robot_state.distance(rand_pose);
}


double
Normalized_Distance_Cost_Function::get_cost()
{
	double cost = (new_robot_state.distance(rand_pose) - min_distance) / (max_distance - min_distance);
	return (cost);
}


void
Normalized_Distance_Cost_Function::set_distance_interval(const double& distance_interval)
{
	this->distance_interval = distance_interval;

	double distance_to_near = near.distance(rand_pose);

	//as vezes o robo anda um pouco mais que distance_interval
	//o que pode gerar um custo negativo posteriormente
	min_distance = fmax(0.0, distance_to_near - distance_interval - 0.5);
	max_distance = distance_to_near + distance_interval + 0.5;
}


double
Lane_Cost_Function::get_cost()
{
	return Lane_Cost_Function::get_cost(new_robot_state);
}


double
Lane_Cost_Function::get_cost(const Robot_State robot_state)
{
	Pose global_pose;
	Pose   front_pose, back_pose;
	double cost;

	global_pose = robot_state.pose;

	if (GlobalState::lane_map.complete_map == NULL)
		return 0.0;

	double vertical_size = Util::to_map_unit(GlobalState::robot_config.distance_between_front_and_rear_axles,
					GlobalState::lane_map.config);

	back_pose = Util::to_map_pose(global_pose, GlobalState::lane_map.config);
	front_pose = back_pose;
	front_pose.x += vertical_size * cos(global_pose.theta);
	front_pose.y += vertical_size * sin(global_pose.theta);
	if (!Util::is_valid_position(front_pose, GlobalState::lane_map.config) || !Util::is_valid_position(back_pose, GlobalState::lane_map.config))
		return 1.0;

	if (robot_state.v_and_phi.v < 0.0) // esta indo para tras?
		cost = GlobalState::lane_map.map[(int) round(back_pose.x)][(int) round(back_pose.y)];
	else
		cost = (GlobalState::lane_map.map[(int) round(front_pose.x)][(int) round(front_pose.y)] + GlobalState::lane_map.map[(int) round(back_pose.x)][(int) round(back_pose.y)]) / 2.0;

	return cost;
}


double
Align_Cost_Function::get_cost()
{
	return Util::heading2(new_robot_state.pose, rand_pose);
}


double
Obstacle_Cost_Function::get_cost()
{
	//as vezes retorna 1 quando nao deveria ... mesma coisa com a funcao de baixo
	int vertical_size   = ceil(Util::to_map_unit(GlobalState::robot_config.length));
	int horizontal_size = ceil(Util::to_map_unit(GlobalState::robot_config.width) / 2.0);
	int distance_between_rear_car_and_rear_wheels = ceil(Util::to_map_unit(GlobalState::robot_config.distance_between_rear_car_and_rear_wheels));

	Pose   car_edges[4], map_pose;
	double max, delta_vertical_x, delta_vertical_y, delta_horizontal_x, delta_horizontal_y;
	float  val;

	map_pose = Util::to_map_pose(new_robot_state.pose);

	delta_vertical_x = cos(map_pose.theta);
	delta_vertical_y = sin(map_pose.theta);

	//a pose do robo originalmente está entre as rodas
	//por isso deve ser transladada para a traseira do carro
	map_pose.x -= distance_between_rear_car_and_rear_wheels * delta_vertical_x;
	map_pose.y -= distance_between_rear_car_and_rear_wheels * delta_vertical_y;

	delta_horizontal_x = cos(M_PI_2 - map_pose.theta);
	delta_horizontal_y = sin(M_PI_2 - map_pose.theta);

	car_edges[0] = car_edges[1] = car_edges[2] = car_edges[3] = map_pose;

	car_edges[0].x = car_edges[0].x - delta_horizontal_x * horizontal_size;
	car_edges[0].y = car_edges[0].y + delta_horizontal_y * horizontal_size;

	car_edges[1].x = car_edges[1].x + delta_horizontal_x * horizontal_size;
	car_edges[1].y = car_edges[1].y - delta_horizontal_y * horizontal_size;

	car_edges[2].x = car_edges[2].x + delta_vertical_x * vertical_size;
	car_edges[2].y = car_edges[2].y + delta_vertical_y * vertical_size;
	car_edges[3]   = car_edges[2];

	car_edges[2].x = car_edges[2].x - delta_horizontal_x * horizontal_size;
	car_edges[2].y = car_edges[2].y + delta_horizontal_y * horizontal_size;

	car_edges[3].x = car_edges[3].x + delta_horizontal_x * horizontal_size;
	car_edges[3].y = car_edges[3].y - delta_horizontal_y * horizontal_size;


	max = 0.0;

	for (int i = 0; i < 4; i++)
	{
		if (!Util::is_valid_position(car_edges[i], GlobalState::cost_map.config))
			return 1.0;

		val = GlobalState::cost_map.complete_map[(int)car_edges[i].x * GlobalState::cost_map.config.y_size + (int)car_edges[i].y];

		if (val > max)
			max = val;
	}

	return max;
}


double
Obstacle_Cost_Function2::get_cost()
{
	return Obstacle_Cost_Function2::get_cost(new_robot_state.pose, GlobalState::cost_map);
}


double
Obstacle_Cost_Function2::get_cost(const Pose &global_pose, const carmen_map_t &map)
{
	carmen_point_t robot_pose = Util::convert_to_carmen_point_t(global_pose);
	carmen_map_t *occupancy_map = (carmen_map_t *) &map;

//	carmen_obstacle_avoider_get_maximum_occupancy_of_map_cells_hit_by_robot(
//				&robot_pose, occupancy_map,
//				GlobalState::robot_config.length, GlobalState::robot_config.width,
//				GlobalState::robot_config.distance_between_rear_car_and_rear_wheels);

	double cost = carmen_obstacle_avoider_get_maximum_occupancy_of_map_cells_hit_by_robot_border(
			&robot_pose, occupancy_map,
			GlobalState::robot_config.length, GlobalState::robot_config.width,
			GlobalState::robot_config.distance_between_rear_car_and_rear_wheels);

	return (carmen_clamp(0.0, cost * 5.0, 1.0));
}

/*
double
Obstacle_Cost_Function2::get_cost(const Pose &global_pose, const carmen_map_t &map)
{
	int vertical_size;
	Pose   front_pose, back_pose;
	double cost;

	if (map.complete_map == NULL)
		return 0.0;

	vertical_size = ceil(Util::to_map_unit(GlobalState::robot_config.length -
					GlobalState::robot_config.distance_between_rear_car_and_rear_wheels,
					map.config));

	back_pose = Util::to_map_pose(global_pose, map.config);
	front_pose = back_pose;
	front_pose.x += vertical_size * cos(global_pose.theta);
	front_pose.y += vertical_size * sin(global_pose.theta);

	if (!Util::is_valid_position(front_pose, map.config) || !Util::is_valid_position(back_pose, map.config))
		return 1.0;

//	cost = (map.complete_map[((int)front_pose.x) * map.config.y_size + ((int)front_pose.y)] +
//			map.complete_map[((int)back_pose.x) * map.config.y_size + ((int)back_pose.y)]) / 2.0;

	cost = fmax(map.complete_map[((int)front_pose.x) * map.config.y_size + ((int)front_pose.y)],
			map.complete_map[((int)back_pose.x) * map.config.y_size + ((int)back_pose.y)]);

	return cost;
}
*/

double
Phi_Change_Cost_Function::get_cost()
{
	return carmen_clamp(0.0, fabs(carmen_normalize_theta(near.v_and_phi.phi - new_robot_state.v_and_phi.phi)), 1.0);
}
