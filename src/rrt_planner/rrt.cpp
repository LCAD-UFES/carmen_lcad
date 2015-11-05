/*
 * rrt.cpp
 *
 *  Created on: 16/12/2011
 *      Author: rradaelli
 */
#include "rrt.h"

double	  t1;

#include <carmen/carmen.h>
#include <carmen/behavior_selector_messages.h>
#include "model/global_state.h"
#include "path_follower/path_follower_ackerman.h"
#include "path_follower/follower.h"
#include "util/obstacle_detection.h"
#include "util/dijkstra.h"
#include "util/util.h"
#include <carmen/navigator_gui_interface.h>

#include <queue>
#include <list>


RRT::RRT()
{
	srand(time(NULL));

	goal_pose = NULL;

	goal_bias_probability = 0.1;

	goal_zoom = false;

	distance_threshold = 5.0;
	theta_threshold	   = carmen_degrees_to_radians(30);

	change_path_distance = 0.1;//lateral distance in meters that the path will be regenerated
	change_path_theta = carmen_degrees_to_radians(10);
}

RRT::~RRT()
{
	clear_the_search_tree_and_current_path();
}

Pose RRT::ramdom_conf_goal_zoom(double radius)
{
	Pose random_pose;
	int i, N;

	N = 30;

	i = 0;

	do
	{
		random_pose.x	  = goal_pose->x + (Util::normalized_random() * radius * 2) - radius;
		random_pose.y	  = goal_pose->y + (Util::normalized_random() * radius * 2) - radius;
		random_pose.theta = carmen_normalize_theta(carmen_degrees_to_radians((rand() % 360) - 180));
		i++;
	} while ((goal_pose->distance(random_pose) > radius || Obstacle_Detection::is_obstacle(random_pose)) && i < N );

	return random_pose;
}

Pose RRT::random_conf_normal()
{
	Pose random_pose;
	int i, N;

	N = 30;
	i = 0;

	do
	{
		random_pose = Util::random_pose();
	} while (Obstacle_Detection::is_obstacle(random_pose) && i < N);

	return random_pose;
}


Pose RRT::random_conf(double p_goal_bias, double radius)
{
	Pose random_pose;

	if (goal_pose && Util::normalized_random() < p_goal_bias)
	{
		if (goal_zoom)
			random_pose = ramdom_conf_goal_zoom(radius);
		else
			random_pose = *goal_pose;
	}
	else
	{
		random_pose = random_conf_normal();
	}

	return random_pose;
}


void RRT::set_stop_condition(double distance_threshold, double theta_threshold)
{
	this->distance_threshold = distance_threshold;
	this->theta_threshold	 = theta_threshold;
}

void RRT::set_change_path_condition(double distance, double theta)
{
	change_path_distance = distance;
	change_path_theta = theta;
}

void RRT::prune_nodes()
{
	for(unsigned i = 0; i < tree.nodes.size(); i++)
	{
		if (tree.nodes[i]->cvf < 1.0 && tree.nodes[i]->prune_node())
		{
			tree.nodes[i]->closed = true;
		}
	}
}

bool RRT::reached_verification_no_prune(const int &status, RRT_Node *new_node)
{
	if (status == REACHED_GOAL && (!GlobalState::goal_node || GlobalState::goal_node->cost > new_node->cost))
	{
		GlobalState::goal_node = new_node;

		return true;
	}

	return false;
}


bool
RRT::reached_verification(int &status, RRT_Node *x_new)
{
	if (status == REACHED_GOAL && (!GlobalState::goal_node || GlobalState::goal_node->cost > x_new->cost))
	{
		GlobalState::goal_node = x_new;

		prune_nodes();
		return true;
	}

	return false;
}

vector<RRT_Node*>
build_path(RRT_Node *goal_node)
{
	vector<RRT_Node*> path;

	while (goal_node)
	{
		path.insert(path.begin(), goal_node);

		goal_node = goal_node->parent;
	}

	return path;

}

void print_path_info(RRT_Node *new_node, double time, int path_number)
{
	double distance_spend = 0.0, time_spend = 0.0;
	int num_rear_command = 0;
	list<RRT_Path_Edge> path;

	path = Dijkstra::build_path(new_node);

	for (list<RRT_Path_Edge>::iterator it = path.begin(); it != path.end(); it++)
	{
		distance_spend += it->p1.distance(it->p2);
		time_spend += it->time;
		if (it->command.v < 0)
			num_rear_command++;
	}

	printf("#%d time=%f distance_spend=%f, time_spend=%f, cost=%f, num_rear_command=%d path_size=%ld\n",
			path_number, time, distance_spend, time_spend, new_node->cost, num_rear_command, path.size());

}


void
RRT::smooth_path(RRT_Node *goal_node)
{
	vector<RRT_Node*> path;

	RRT_Node* current_node;
	int status = -1;
	Pose target;
	bool last_index;
	unsigned int path_index;

	path = build_path(goal_node);
	for (int step = 5; step > 0; step--)
	{

		for (unsigned int current_node_index = 0; current_node_index < path.size(); )
		{
			current_node = path[current_node_index];
			path_index = current_node_index;
			status = -1;
			last_index = false;

			while (!last_index)
			{
				path_index += step;

				if (path_index >= path.size())
				{
					path_index = path.size() - 1;
					last_index = true;
				}

				target = path[path_index]->robot_state.pose;
				current_node = connect(target, current_node, &status);

				if (status == TRAPPED)
					break;

				if (status == REACHED_GOAL && current_node->cost < goal_node->cost)
				{

					reached_verification_no_prune(status, current_node);
					goal_node = current_node;
					path = build_path(goal_node);

					//					print_path_info(goal_node,
					//							Util::get_time() - t1,
					//							reaches_goal_nodes.size());


					break;
				}
			}

			current_node_index++;
		}

	}

	prune_nodes();
}


void RRT::check_necessary_conditions_and_build_rrt_path()
{
	if (GlobalState::current_algorithm != CARMEN_BEHAVIOR_SELECTOR_RRT)
		return;

	if (	!goal_pose ||
			!tree.root_node ||
			node_reaches_goal(tree.root_node) ||
			!GlobalState::cost_map_initialized ||
			Obstacle_Detection::is_obstacle(*goal_pose))
		return;

	build_rrt_path();
}


double get_intermediarie_time(Robot_State rs, Pose &p1, Pose &p2, Command &c)
{
	double last_distance, time, time_increment;
	double distance;

	time_increment = 0.001;
	last_distance = DBL_MAX;
	time = 0.0;

	if (p1 == p2)
	{
		return fabs((c.phi - rs.v_and_phi.phi) / GlobalState::robot_config.desired_steering_command_rate);
	}

	if (rs.v_and_phi == c && c.v == 0.0)
		return 0.0;

	if (c.v != 0.0)
	{
		last_distance = rs.distance(p2);
		time = fabs(last_distance / c.v);
		rs = Ackerman::predict_next_pose_during_main_rrt_planning(rs, c, time);
		last_distance = rs.distance(p2);
	}

	for (int i = 0; i < 10000; i++)
	{
		rs = Ackerman::predict_next_pose_during_main_rrt_planning(rs, c, time_increment);
		distance = rs.distance(p2);

		if ( distance > last_distance)
			break;

		last_distance = distance;
		time += time_increment;

	}
	return time;
}


RRT_Node* RRT::reuse_last_path(int num_reused_nodes, int &status)
{
	RRT_Node *node, *new_node;
	list<RRT_Path_Edge>::iterator it;
	RRT_Edge edge;
	double time;

	node = tree.root_node;
	it = old_path.begin();

	num_reused_nodes = fmin(num_reused_nodes, old_path.size());

	for (int i = 0; i < num_reused_nodes && it != old_path.end(); i++, it++)
	{
		time = get_intermediarie_time(node->robot_state, it->p1.pose, it->p2.pose, it->command);

		if (!Obstacle_Detection::is_obstacle_path(*node, it->command, time))
		{
			new_node = new RRT_Node();
			new_node->robot_state = Ackerman::predict_next_pose_during_main_rrt_planning(node->robot_state, it->command, time);

			node->add_command(it->command, time);
			node->update_cvf();

			edge.n1 = node;
			edge.n2 = new_node;
			edge.command = it->command;
			edge.time = time;
			edge.cost = edge.time + edge.time * Ackerman::get_env_cost(new_node->robot_state);

			new_node->parent = node;
			new_node->cost = node->cost + edge.cost;

			tree.add_node(new_node);
			tree.add_edge(edge);

			if(node_reaches_goal(new_node))
			{
				status = REACHED_GOAL;
				reaches_goal_nodes.push_back(new_node);
				reached_verification(status, new_node);
			}

			node = new_node;

			continue;
		}

		break;
	}

	//		printf("tree size: %ld %ld\n", tree.nodes.size(), old_path.size());

	status = ADVANCED;
	return node;
}


bool
RRT::set_goal(Pose &goal_pose)
{
	if ( !this->goal_pose ||
			(this->goal_pose->distance(goal_pose) >= distance_threshold) ||
			(this->goal_pose->get_theta_diff(goal_pose) >= theta_threshold))
	{
		delete this->goal_pose;
		clear_the_search_tree_and_current_path();

		if (GlobalState::localize_pose)
			tree.add_root_node(*GlobalState::localize_pose, GlobalState::last_odometry);

		this->goal_pose	   = new Pose();
		*this->goal_pose = goal_pose;

		return true;
	}

	return false;
}

static void
clear_unused_new_nodes(vector<RRT_Node*> &new_nodes, int i)
{
	for (unsigned int j = i; j < new_nodes.size(); j++)
		delete new_nodes[j];
}


void
RRT::update_adjacenty_nodes_cost(RRT_Node *node)
{
	list<RRT_Node*> nodes;
	list<RRT_Edge>::iterator it;
	nodes.push_back(node);

	double new_cost;

	RRT_Node* current;

	while (!nodes.empty())
	{
		current = nodes.front();
		nodes.pop_front();

		for (it = current->adjacency_nodes.begin(); it != current->adjacency_nodes.end(); it++)
		{
			new_cost = current->cost + it->cost;

			if (new_cost < it->n2->cost)
			{

				it->n2->cost = new_cost;
				it->n2->parent = current;
				it->n2->closed = GlobalState::goal_node && new_cost > GlobalState::goal_node->cost;

				if (node_reaches_goal(it->n2))
				{
					reached_verification_no_prune(REACHED_GOAL, it->n2);
				}

				nodes.push_back(it->n2);
			}
		}
	}
}


RRT_Node *
RRT::extend(Pose &x_rand, RRT_Node *x_near, int *status)
{
	vector<RRT_Node*> x_new_alternatives;
	vector<Command>	   commands;
	vector<double>	   commands_time;
	RRT_Node *prev;
	RRT_Edge edge;

	*status = ADVANCED;

	if (x_near == NULL)
	{
		*status = TRAPPED;
		return NULL;
	}

	compute_x_new_alternatives_and_associated_commands_and_times(x_new_alternatives, commands, commands_time, *x_near, x_rand);

	if (commands.empty())
	{
		*status = TRAPPED;
		x_near->closed = true;
		return NULL;
	}

	prev = x_near;

	// add nodes to tree
	for (unsigned int i = 0; i < x_new_alternatives.size(); i++)
	{
		prev->add_command(commands[i], commands_time[i]);

		edge.n1 = prev;
		edge.command = commands[i];
		edge.time = commands_time[i];
		edge.cost = edge.time + edge.time * Ackerman::get_env_cost(x_new_alternatives[i]->robot_state);
		x_new_alternatives[i]->cost = edge.cost + prev->cost;
		x_new_alternatives[i]->parent = prev;

		// descartar nós que não tem chance de gerar novos paths
		if (x_new_alternatives[i]->prune_node())
		{
			clear_unused_new_nodes(x_new_alternatives, i);
			*status = TRAPPED;
			return NULL;
		}

		if (!tree.add_node(x_new_alternatives[i]))
		{//caso existe um vertice repetido na árvore
			RRT_Node *tree_node = tree.nodes[tree.nodes_map[x_new_alternatives[i]->get_key()]];

			if (x_new_alternatives[i]->cost >= tree_node->cost)
			{
				// acho que mudei pra errado essa parte ... mas so vai entrar aqui se estiver no modo estacionamento ...
				delete x_new_alternatives[i];
				x_new_alternatives[i] = tree_node;
			}
			else
			{
				tree_node->cost = x_new_alternatives[i]->cost;
				tree_node->parent = x_new_alternatives[i]->parent;
				delete x_new_alternatives[i];
				x_new_alternatives[i] = tree_node;

				update_adjacenty_nodes_cost(tree_node);
			}
		}

		edge.n2 = x_new_alternatives[i];
		tree.add_edge(edge);

		if (node_reaches_goal(x_new_alternatives[i]))
		{
			reaches_goal_nodes.push_back(x_new_alternatives[i]);
			*status = REACHED_GOAL;
			return x_new_alternatives[i];
		}

		prev = x_new_alternatives[i];
	}

	return x_new_alternatives[x_new_alternatives.size() - 1];
}


RRT_Node *
RRT::connect(Pose &rand, RRT_Node *near, int *status, int max_extend)
{
	//	double theta, diff_theta;
	double near_distance, new_node_distance;
	int	   i;
	RRT_Node *new_node;

	*status = ADVANCED;

	i = 0;
	near_distance = near->distance(rand);

	do
	{
		new_node = extend(rand, near, status);

		if (new_node == NULL)
			return near;

		new_node_distance = new_node->distance(rand);

		if (new_node_distance < near_distance)
		{
			near_distance = new_node_distance;
			near = new_node;
		}
		else if (*status != REACHED_GOAL)
			return near;//new_node ultrapassou rand, retornar nó anterior

		i++;

	} while (*status == ADVANCED && i < max_extend);

	return near;
}

bool RRT::node_reaches_goal(RRT_Node *new_node)
{
	return 	pose_reaches_goal(new_node->robot_state.pose);
}

bool RRT::pose_reaches_goal(Pose &pose)
{
	return goal_pose &&
			goal_pose->distance(pose) < distance_threshold &&
			goal_pose->get_theta_diff(pose) < theta_threshold;
}


void RRT::clear_the_search_tree_and_current_path()
{
	path.clear();
	tree.clear();
	reaches_goal_nodes.clear();
	GlobalState::goal_node = NULL;
}


bool RRT::is_valid_path()
{
	list<RRT_Path_Edge>::iterator it;

	for (it = path.begin(); it != path.end(); it++)
		if (Obstacle_Detection::is_obstacle_path2(it->p1, it->command, it->time))
			return false;

	return true;
}

