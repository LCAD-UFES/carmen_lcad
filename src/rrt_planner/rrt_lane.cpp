/*
 * rrt_lane.cpp
 *
 *  Created on: 15/05/2014
 *      Author: romulo
 */

#include <carmen/carmen.h>
#include <carmen/behavior_selector_messages.h>
#include <carmen/navigator_gui_interface.h>
#include "model/global_state.h"
#include "path_follower/path_follower_ackerman.h"
#include "path_follower/follower.h"
#include "util/obstacle_detection.h"
#include "util/publisher_util.h"
#include "util/dijkstra.h"
#include "util/util.h"
#include "rrt_lane.h"
#include "rrt.h"


// Distribuicao de custos
RRT_Lane::RRT_Lane() {
	num_lane_cost_function = 6;
	lane_cost_functions[0] = new Normalized_Distance_Cost_Function();
	lane_weights[0] 	   = 0.15;
	lane_cost_functions[1] = new Velocity_Cost_Function();
	lane_weights[1] 	   = 0.2;
	lane_cost_functions[2] = new Obstacle_Cost_Function2();
	lane_weights[2] 	   = 0.25;
	lane_cost_functions[3] = new Rear_Motion_Cost_Function();
	lane_weights[3] 	   = 0.08;
	lane_cost_functions[4] = new Change_Direction_Cost_Function();
	lane_weights[4] 	   = 0.12;
	lane_cost_functions[5] = new Lane_Cost_Function();
	lane_weights[5] 	   = 0.20;
	lane_cost_functions[6] = new Phi_Change_Cost_Function();
	lane_weights[6] 	   = 0.05;
}


void
RRT_Lane::build_rrt_path_from_robot_pose(Robot_State &robot_pose)
{

	double path_distance, theta_diff;
	Robot_State path_pose;

	if ((GlobalState::current_algorithm != CARMEN_BEHAVIOR_SELECTOR_RRT) ||
		!goal_pose || !GlobalState::cost_map_initialized || Obstacle_Detection::is_obstacle(*goal_pose))
		return;

	old_path = path;
	old_path_timestamp = path_timestamp;
	old_build_time = build_time;

	// Remove nos ja percorridos do path anterior. O path anterior eh incluido na arvore de busca em build_rrt_path().
	while(Ackerman::remove_front_node_of_the_path(robot_pose.pose, old_path, path_distance, theta_diff, &path_pose))
		;

	clear_the_search_tree_and_current_path();

	// if (!old_path.empty())
	//	robot_pose.command = path_pose.command;
	//tree.add_root_node(robot_pose.pose, robot_pose.command);
	tree.add_root_node(robot_pose.pose, GlobalState::last_odometry);

	if (!tree.root_node || node_reaches_goal(tree.root_node))
		return;

	build_rrt_path();
	if (!is_valid_path())
		clear_the_search_tree_and_current_path();
}


Pose
RRT_Lane::random_lane_pose(double radius)
{
	if (GlobalState::lane_points.empty())
		return random_conf_normal();

	Pose lane_pose = Util::to_map_pose(GlobalState::lane_points[(int)((GlobalState::lane_points.size() - 1) * Util::normalized_random())]);

	double radius_in_map_units = Util::to_map_unit(radius);

	Pose rand_lane_pose;
	int i = 0;
	do
	{
		rand_lane_pose.x = lane_pose.x + Util::normalized_random() * radius_in_map_units * 2.0 - radius_in_map_units;
		rand_lane_pose.y = lane_pose.y + Util::normalized_random() * radius_in_map_units * 2.0 - radius_in_map_units;

		if (i > 10)
		{
			rand_lane_pose = lane_pose;
			break;
		}

		i++;
	} while (Obstacle_Detection::is_obstacle_point(rand_lane_pose));

	return Util::to_global_pose(rand_lane_pose);
}


Pose
RRT_Lane::random_sub_lane_pose()
{
	Pose lane_pose;
	int i;

	if (sub_lane_points.empty())
		return random_conf_normal();

	i = 0;
	do
	{
		lane_pose = sub_lane_points[(int)((sub_lane_points.size() - 1) * Util::normalized_random())];
		i++;
	} while (Obstacle_Detection::is_obstacle_point(lane_pose) && (i < 10));

	return Util::to_global_pose(lane_pose);
}


Pose
RRT_Lane::random_pose_close_to_the_lane(double p_sub_lane, double radius)
{
	if (Util::normalized_random() < p_sub_lane)
		return random_sub_lane_pose();

	return random_lane_pose(radius);
}


void
RRT_Lane::build_rrt_path()
{
	list<RRT_Path_Edge>::iterator it;

	double		build_time, initial_time;
	double 		radius = GlobalState::robot_config.length;
	int			status, i;
	RRT_Node 	*x_new, *x_near;
	Pose		x_rand;
	bool		timeout = false;
	static int 	num_builds = 0;

	initial_time = Util::get_time();

	if (GlobalState::show_debug_info)
		printf("\n-------------BUILD CALLED-------------\n");

	create_sub_lane_points_vector();

	if (GlobalState::reuse_last_path)
		reuse_last_path(100, status);

	double max_plan_time = (fabs(GlobalState::last_odometry.v) > 0.5)? GlobalState::timeout: 50 * GlobalState::timeout;
	i = 0;
	while (!timeout)
	{
		if (i == 0) // first try
		{
			x_rand = *goal_pose;
			x_near = tree.root_node;
		}
		else
		{
			x_rand = random_pose_close_to_the_lane(0.7, radius);
			x_near = tree.nearest_node(x_rand);
		}

		if (x_near == NULL) // todos os nos foram explorados e nenhum caminho foi encontrado
			break;

		x_new = extend(x_rand, x_near, &status);

		if (status == ADVANCED)
			x_new = connect(*goal_pose, x_new, &status);

		if (status == REACHED_GOAL)
			reached_verification(status, x_new);

		build_time = Util::get_time() - initial_time;

		if (((build_time > GlobalState::plan_time) && (reaches_goal_nodes.size() != 0)) ||
			(build_time > max_plan_time))
			timeout = true;

		i++; num_builds++;
	}

	if (GlobalState::goal_node)
	{
		//RRT::smooth_principal_path_from_tree_using_conjugate_gradient(GlobalState::goal_node);
		path = Dijkstra::build_path(GlobalState::goal_node);
		path_timestamp = GlobalState::localizer_pose_timestamp;
	}

	if (GlobalState::show_debug_info)
	{
		system("clear");
		printf(	"-------------%d BUILD FINISHED AFTER %d TRIES-------------\n"
				"Relatório:\n"
				"%ld nós gerados\n"
				"%ld paths encontrados\n"
				"Path size: %ld nós, %lf m\n"
				"Build time %.4lf s\n"
				"Goal  pose x = %lf, y = %lf, theta = %lf\n"
				"Robot pose x = %lf, y = %lf, theta = %lf\n",
				num_builds, i, tree.nodes.size(), reaches_goal_nodes.size(), path.size(),
				GlobalState::goal_node != NULL ? GlobalState::goal_node->cost : 0.0, build_time,
				GlobalState::goal_pose->x, GlobalState::goal_pose->y, GlobalState::goal_pose->theta,
				tree.root_node->robot_state.pose.x, tree.root_node->robot_state.pose.y, tree.root_node->robot_state.pose.theta);
	}

/*	it = path.begin();

	RRT_Path_Edge p;
	p.time = 0.0;
	p.p1.pose.x = 0.0;
	p.p1.pose.y = 0.0;

	//it->p1.pose.x = 0.0;
	//printf ("aaaaaaaaaaaaaaaaaaaaaaaaaaaa%f %f\n", it->p1.pose.x, it->p1.pose.y);

	for (it = path.begin(); it != path.end(); it++, i++)
	{
		it->p1.pose.x = 0.0;
		//printf ("%f %f\n", it->p1.pose.x, it->p1.pose.y);
	}
	it--;
	printf ("%f %f\n\n", it->p2.pose.x, it->p2.pose.y);*/
}


void
RRT_Lane::compute_x_new_alternatives_and_associated_commands_and_times(vector<RRT_Node *> &x_new_alternatives_computed,
		vector<Command> &x_new_associated_commands, vector<double> &x_new_associated_commands_times,
		RRT_Node &x_near, Pose &x_rand)
{
	double		min_cost, x_new_associated_command_time;
	RRT_Node   *x_new_alternative;
	Command		x_new_associated_command;
	Robot_State robot_state, new_robot_state;

	x_new_alternative = new RRT_Node();

	bool found = Ackerman::search_for_x_new_alternative_and_associated_command_and_time(x_new_alternative, &x_new_associated_command, &x_new_associated_command_time,
			x_near, x_rand,
			lane_cost_functions, lane_weights, num_lane_cost_function,
			&min_cost, tree);

	if (found)
	{
		x_new_alternatives_computed.push_back(x_new_alternative);
		x_new_associated_commands.push_back(x_new_associated_command);
		x_new_associated_commands_times.push_back(x_new_associated_command_time);
	}
	else
		delete x_new_alternative;
}


void
RRT_Lane::create_sub_lane_points_vector()
{
	sub_lane_points.clear();

	double radius = Util::to_map_unit(tree.root_node->robot_state.pose.distance(*goal_pose));
	Pose center_point = Util::to_map_pose(tree.root_node->robot_state.pose); // O centro tem que ser a pose neste ponto para a reh funcionar

	for (unsigned int i = 0; i < GlobalState::lane_points_on_map.size(); i++)
	{
		if ((GlobalState::lane_points_on_map[i].distance(center_point) < radius))
			sub_lane_points.push_back(GlobalState::lane_points_on_map[i]);
	}
}

