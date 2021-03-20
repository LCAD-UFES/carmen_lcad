/*
 * rrt_parking.cpp
 *
 *  Created on: 15/05/2014
 *      Author: romulo
 */

#include "rrt_parking.h"
#include <carmen/carmen.h>
#include <carmen/behavior_selector_messages.h>
#include "model/global_state.h"
#include "path_follower/path_follower_ackerman.h"
#include "path_follower/follower.h"
#include "util/obstacle_detection.h"
#include "util/publisher_util.h"
#include "util/dijkstra.h"
#include "util/util.h"
#include <carmen/navigator_gui_interface.h>

#include <queue>
#include <list>

RRT_Parking::RRT_Parking()
{
	max_dist_grad = 15; //max_dist_grad is the maximum value of dist_grad
	gradient_probability  = 0.2;

	num_parking_cost_function = 3;
	parking_cost_functions[0] = new Normalized_Distance_Cost_Function();
	parking_weights[0] 	   = 0.30;
	parking_cost_functions[1] = new Velocity_Cost_Function();
	parking_weights[1] 	   = 0.19;
	parking_cost_functions[2] = new Obstacle_Cost_Function2();
	parking_weights[2] 	   = 0.13;

	distance_near = 9.0;
}


void
print_path(list<RRT_Path_Edge> path, char *path_name)
{
	return;
	printf("%s\n", path_name);
	list<RRT_Path_Edge>::iterator it;
	for (it = path.begin(); it != path.end(); it++)
	{
		printf("v = %2.2lf, phi = %2.2lf, t = %2.3lf, p1.v = %2.2lf, p1.phi = %2.2lf, p2.v = %2.2lf, p2.phi = %2.2lf\n",
					it->command.v, carmen_radians_to_degrees(it->command.phi), it->time,
					it->p1.v_and_phi.v, carmen_radians_to_degrees(it->p1.v_and_phi.phi),
					it->p2.v_and_phi.v, carmen_radians_to_degrees(it->p2.v_and_phi.phi));
	}
}

void RRT_Parking::reuse_previous_path()
{
	if (path.size() > 0)
		print_path(path, (char *) "====== PATH ======");
	if (old_path.size() > 0)
		print_path(old_path, (char *) "====== OLD PATH ======");

	if ((old_path.size() == 0) && (path.size() > 0) && (reaches_goal_nodes.size() > 0))
		old_path = path;
	else
		path = old_path;
}

void RRT_Parking::build_rrt_path_from_robot_pose(Robot_State &robot_pose)
{
	double path_distance, theta_diff;

	path_distance = 0;
	theta_diff = 0;

	if (tree.root_node == NULL || !is_valid_path())
	{
		clear_the_search_tree_and_current_path();
		tree.add_root_node(robot_pose.pose, GlobalState::last_odometry);
		check_necessary_conditions_and_build_rrt_path();
//		reuse_previous_path();
		publish_status_message();
		return;
	}

	while(Ackerman::remove_front_node_of_the_path(robot_pose.pose, path, path_distance, theta_diff))
		;

	//verify if the robot is out of the path
	if (goal_pose && (path.empty() || path_distance > change_path_distance || theta_diff > change_path_theta))
	{
		clear_the_search_tree_and_current_path();
		tree.add_root_node(robot_pose.pose, GlobalState::last_odometry);
		check_necessary_conditions_and_build_rrt_path();
		publish_status_message();
	}

//	reuse_previous_path();
}


void RRT_Parking::build_rrt_path()
{
	double		p_grad;
	double		t1, current_time, build_time, cost_time;
	double		random_conf_radius;
	int			status, i;
	RRT_Node	*new_node, *near;
	Pose		rand_pose;
	bool		timeout = false;

	random_conf_radius = 8.0;
	current_time = 0.0;
	new_node = near = NULL;

	t1 = Util::get_time();
//	Cost_Map_Util::rebuild_cost_maps(); // rebuild cost maps if necessary
	cost_time = Util::get_time() - t1;

	t1 = Util::get_time();

	if (GlobalState::show_debug_info)
		printf("\n-------------BUILD CALLED-------------\n");

	i = 0;
	while (!timeout)
	{
		rand_pose = random_conf(goal_bias_probability, random_conf_radius);
		near = tree.nearest_node(rand_pose);

		if (near == NULL) // todos os nos foram explorados e nenhum caminho foi encontrado
			break;

		p_grad = Util::normalized_random();
		if (p_grad <= gradient_probability)
			new_node = connect_grad(near, &status);
		else
			new_node = extend(rand_pose, near, &status);

		if (status == REACHED_GOAL)
		{
			reached_verification(status, new_node);
			printf("#%ld time: %f %f\n", reaches_goal_nodes.size(),	(cost_time + (Util::get_time() - t1)),
					(*reaches_goal_nodes.rbegin())->cost);

			smooth_path(new_node);
		}

		build_time = Util::get_time() - t1;
		current_time = cost_time + build_time;

		if ((build_time > GlobalState::plan_time && reaches_goal_nodes.size()) || (current_time > GlobalState::timeout))
			timeout = true;

		i++;
	}

	if (GlobalState::goal_node)
		path = Dijkstra::build_path(GlobalState::goal_node);

	if (GlobalState::show_debug_info)
	{
		printf(	"Relatório:\n"
				"%ld nós gerados\n"
				"%ld paths encontrados\n"
				"Path size: %ld\n"
				"Cost map time %.4f s\n"
				"Build time %.4f s\n"
				"Total time %.4f s\n"
				"-------------BUILD FINISHED %d-------------\n",
				tree.nodes.size(), reaches_goal_nodes.size(), path.size(), cost_time, build_time, current_time, i);
	}
}

void RRT_Parking::set_max_distance_grad(double distance)
{
	this->max_dist_grad = distance;
}

void RRT_Parking::set_distance_near(double distance_near)
{
	this->distance_near = distance_near;
}

void RRT_Parking::compute_x_new_alternatives_and_associated_commands_and_times(vector<RRT_Node *> &new_nodes, vector<Command> &commands, vector<double> &commands_time, RRT_Node &near, Pose &rand)
{
	double		min_cost, command_time;
	RRT_Node   *new_node;
	Command		command;
	Robot_State robot_state, new_robot_state;

	if (near.distance(*goal_pose) < 10 && pose_reaches_goal(rand))
	{
		Ackerman::find_command_rs(near.robot_state, rand, commands, commands_time);

		if (!commands.empty())
		{
			robot_state = near.robot_state;

			for (unsigned int i = 0; i < commands.size(); i++)
			{
				new_robot_state = Ackerman::predict_next_pose_during_main_rrt_planning(robot_state, commands[i], commands_time[i]);

				new_robot_state.v_and_phi = commands[i];

				new_node = new RRT_Node();
				new_node->robot_state  = new_robot_state;

				new_nodes.push_back(new_node);

				robot_state = new_robot_state;
			}


			return;
		}
	}

	new_node = new RRT_Node();

	//		double t1 = Util::get_time();
	//		bool search_result = Ackerman::search_command(near, rand, new_node, &command, &command_time, front_weight_parking, rear_weight_parking, move_weight_parking, &min_cost);
	bool search_result = Ackerman::search_command_improved_parking(near, rand, new_node, &command, &command_time,
			parking_cost_functions, parking_weights, num_parking_cost_function, &min_cost, Ackerman::minimal_velocity_search_vector, tree);


	if (!search_result)
	{
		search_result = Ackerman::search_command_improved_parking(near, rand, new_node, &command, &command_time,
				parking_cost_functions, parking_weights, num_parking_cost_function, &min_cost, Ackerman::velocity_search_vector, tree);
	}

	//		printf("tempo gasto: %f s\n", Util::get_time() - t1);

	if (search_result)
	{
		new_nodes.push_back(new_node);
		commands.push_back(command);
		commands_time.push_back(command_time);
	}
	else
		delete new_node;

}


/**
 * initial_point on the map
 * max_dist in map units
 */
Pose get_gradient_point(Pose &initial_point, const double &max_dist)
{
	static int carmen_planner_x_offset[8] = {0, 1, 1, 1, 0, -1, -1, -1};
	static int carmen_planner_y_offset[8] = {-1, -1, 0, 1, 1, 1, 0, -1};
	Pose grad_pose, max_p;
	double max, new_max, distance, multiplier, cost;
	int x, y;

	grad_pose = initial_point;
	max = GlobalState::utility_map.costs[(int)grad_pose.x * GlobalState::utility_map.config.y_size + (int)grad_pose.y];
	distance   = 0;
	multiplier = 1;

	while (distance < max_dist)
	{
		new_max = max;

		for (int action = 0; action < 8; action++)
		{
			x = grad_pose.x + carmen_planner_x_offset[action];
			y = grad_pose.y + carmen_planner_y_offset[action];

			if (!Util::is_valid_position(x, y, GlobalState::utility_map.config))
				continue;

			cost = GlobalState::utility_map.costs[x * GlobalState::utility_map.config.y_size + y];

			if (cost > new_max)
			{
				new_max	   = cost;
				max_p.x	   = x;
				max_p.y	   = y;
				multiplier = (action % 2) == 1 ? M_SQRT2 : 1;
			}
		}

		if (new_max <= max)
			break;

		max = new_max;
		grad_pose = max_p;
		distance += 1 * multiplier;
	}

	return grad_pose;

}

Pose RRT_Parking::get_gradient_node(Pose &global_pose, double max_dist_meters)
{
	Pose grad_pose, next_grad_pose, pose, global_grad_pose;
	double distance_pose_to_goal, distance_grad_to_goal;

	distance_pose_to_goal = global_pose.distance(*goal_pose);

	if (distance_pose_to_goal < distance_near)
	{
		return *goal_pose;
	}

	pose = Util::to_map_pose(global_pose, GlobalState::utility_map.config);

	if (!Util::is_valid_position(pose, GlobalState::utility_map.config))
		return *goal_pose;

	grad_pose = get_gradient_point(pose, Util::to_map_unit(max_dist_meters, GlobalState::utility_map.config));

	next_grad_pose = get_gradient_point(grad_pose, Util::to_map_unit(1.5, GlobalState::utility_map.config));

	grad_pose.theta = carmen_normalize_theta(atan2(next_grad_pose.y - grad_pose.y, next_grad_pose.x - grad_pose.x));

	global_grad_pose = Util::to_global_pose(grad_pose, GlobalState::utility_map.config);

	distance_grad_to_goal = global_pose.distance(global_grad_pose);

	if (distance_pose_to_goal < distance_near && distance_grad_to_goal < distance_pose_to_goal)
	{
		return *goal_pose;
	}

	return global_grad_pose;
}


RRT_Node *RRT_Parking::connect_grad(RRT_Node *near, int *status, int k)
{
	double dist_to_goal, dist_grad;
	Pose   grad;
	RRT_Node *new_node;
	int	   i;

	*status = ADVANCED;

	i = 0;
	dist_grad	  = max_dist_grad; //dist that will be used in the get_grad_node function

	grad = get_gradient_node(near->robot_state.pose, dist_grad);


	do
	{
		new_node = extend(grad, near, status);

		//se o nó gerado for considerado um nó ruim
		//pegar o pai do nó anterior e tentar gerar algo melhor
		if (*status != REACHED_GOAL && (new_node == NULL || *status == TRAPPED || near->distance(grad) < new_node->distance(grad)))
		{
			near = near->parent;
			*status = ADVANCED;
		}
		else
		{
			near = new_node;
		}


		if (near != NULL && *status != REACHED_GOAL )
		{
			dist_to_goal = near->distance(*goal_pose);

			//if near_node is close to goal, use litle steps to search grad_node
			if (dist_to_goal < max_dist_grad / 2.0)
				dist_grad = dist_to_goal / 2.0;

			//if grad_node is an obstacle or near_node is too close
			//search another grad_node
			if (grad.distance(near->robot_state.pose) < (max_dist_grad * 0.5) || grad.distance(near->robot_state.pose) > max_dist_grad)
				grad = get_gradient_node(near->robot_state.pose, dist_grad);
		}



		i++;
	} while (*status == ADVANCED && near != NULL && i < k);

	return near;
}
