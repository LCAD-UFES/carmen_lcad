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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_math.h>

#include <queue>
#include <list>

//FILE *plot = fopen("p.m", "w");
//FILE *normal = fopen("normal.m", "a");
//FILE *smooth = fopen("smooth.m", "a");
//int cont=0;


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


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Smooth Path Using Conjugate Gradient                                                      //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


Pose
calculate_point_in_curvature_arc (Pose &p1, Pose &p2)
{
	double delta_x=0, delta_y=0, p1x=p1.x, p1y=p1.y, p2x=p2.x, p2y=p2.y; //Copy values to variables avoids always acessing the structures
	Pose p0;

	if (p1x < p2x && p1y < p2y)
	{
		delta_x = p2x - p1x;
		delta_y = p2y - p1y;
		p0.x = p1x - delta_x;
		p0.y = p1y - delta_y;
		return p0;
	}
	if (p1x > p2x && p1y < p2y)
	{
		delta_x = p1x - p2x;
		delta_y = p2y - p1y;
		p0.x = p1x + delta_x;
		p0.y = p1y - delta_y;
		return p0;
	}
	if (p1x > p2x && p1y > p2y)
	{
		delta_x = p1x - p2x;
		delta_y = p1y - p2y;
		p0.x = p1x + delta_x;
		p0.y = p1y + delta_y;
		return p0;
	}

	delta_x = p2x - p1x;
	delta_y = p1y - p2y;
	p0.x = p1x - delta_x;
	p0.y = p1y + delta_y;
	return p0;

}


double
recalculate_theta (Pose &current, Pose &right)			            		//Recalculate cars orientantion
{
	Pose left = calculate_point_in_curvature_arc (current, right);

    double x1 = (right.x - left.x) + (right.x - current.x);
    double y1 = (right.y - left.y) + (right.y - current.y);

    return atan2(y1, x1);
}


double
recalculate_theta_old (Pose &left, Pose &current, Pose &right)					//Recalculate cars orientantio
{
    double x1 = 0.25*(right.x - left.x) + 0.75*(right.x - current.x);
    double y1 = 0.25*(right.y - left.y) + 0.75*(right.y - current.y);

    return atan2(y1, x1);
}


double
recalculate_delta_time (double x1, double x2, double v, double theta)      //Recalculate time duration of command
{                                                                          //Using first equation of ackerman model of movement
	return ((x2 - x1) / (v * cos(theta)));
}


double
recalculate_phi (double theta1, double theta2, double v, double time, double L)    //Recalculate wheel angle
{                                                                                  //Using last equation of ackerman model of movement
	return atan (((theta2 - theta1)* L) / (time * v));
}


void
recalculate_theta_time_phi(list<RRT_Path_Edge> &path)
{
    list<RRT_Path_Edge>::iterator it_ant = path.begin();
    list<RRT_Path_Edge>::iterator it = it_ant;
    double L = GlobalState::robot_config.distance_between_front_and_rear_axles;
    int first = 0;

    it->p1.pose.theta = recalculate_theta(it->p1.pose, it->p2.pose);
    it->time = recalculate_delta_time(it->p1.pose.x, it->p2.pose.x, it->command.v, it->p1.pose.theta);

    for (it++; it != path.end(); it_ant++, it++)
    {
    	it_ant->p2.pose.theta = it->p1.pose.theta = recalculate_theta(it->p1.pose, it->p2.pose);

    	if (first == 0)
    	{
    		first = 1;
    		it_ant->command.phi = recalculate_phi (it_ant->p1.pose.theta, it_ant->p2.pose.theta, it_ant->command.v, it_ant->time, L);
    	}

    	it->time = recalculate_delta_time(it->p1.pose.x, it->p2.pose.x, it->command.v, it->p1.pose.theta);

    	it_ant->p2.v_and_phi.phi = it->p1.v_and_phi.phi = it_ant->command.phi =
    			recalculate_phi (it->p1.pose.theta, it->p2.pose.theta, it->command.v, it->time, L);
    }

    it_ant->command.phi = it_ant->p2.v_and_phi.phi;
}


void
save_smoothed_path_back_to_tree (RRT_Node *goal, list<RRT_Path_Edge> path)
{
	list<RRT_Edge>::iterator it_tree;
	list<RRT_Path_Edge>::iterator it_path = path.end();
	RRT_Node *parent;

	it_path--;

	do
	{
		parent = goal->parent;
		it_tree = goal->prev_nodes.begin();

		for (; it_tree != goal->prev_nodes.end(); it_tree++)
		{
			if (it_tree->n1 == parent)
			{
				//printf ("a%f %f  %f %f\n", it_path->p1.pose.x, it_path->p1.pose.y, it_path->p2.pose.x, it_path->p2.pose.y);
				it_tree->n1->robot_state = it_path->p1;
				it_tree->n2->robot_state = it_path->p2;
				it_tree->command = it_path->command;
				it_tree->time = it_path->time;
				it_path--;
				//printf ("b%f %f  %f %f\n", it_tree->n1->robot_state.pose.x, it_tree->n1->robot_state.pose.y, it_tree->n2->robot_state.pose.x, it_tree->n2->robot_state.pose.y);
				break;
			}
		}
		goal = goal->parent;
	}
	while (goal);
}


//Function to be minimized summation[x(i+1)-2x(i)+x(i-1)]
double
my_f(const gsl_vector *v, void *params)
{
	list<RRT_Path_Edge> *p = (list<RRT_Path_Edge> *)params;
	int i, j, size =(p->size()-1);                                  //(p->size-1)Instead of -2 because the message size is 4 but the message has 5 points
	double a=0, b=0, sum=0;                                       //and we have to discount the first and last point that wont be optimized

	//printf("%d\n", size+1);
	double x_prev = p->front().p1.pose.x;			//x(i-1)
	double x      = gsl_vector_get(v, 0);		//x(i)
	double x_next = gsl_vector_get(v, 1);		//x(i+1)

	double y_prev = p->front().p1.pose.y;
	double y      = gsl_vector_get(v, size);
	double y_next = gsl_vector_get(v, size+1);
	//printf("---%f %f\n", x_prev, y_prev);
	//printf("---%f %f\n", x, y);
	///printf("---%f %f\n", x_next, y_next);
	for (i = 2, j = (size+2); i < size; i++, j++)
	{
		a = x_next - (2*x) + x_prev;
		b = y_next - (2*y) + y_prev;
		sum += (a*a + b*b);

		x_prev = x;
		x      = x_next;
		x_next = gsl_vector_get(v, i);

		y_prev = y;
		y      = y_next;
		y_next = gsl_vector_get(v, j);
		//printf("---%f %f\n", x_next, y_next);
	}

	x_prev = x;
	x      = x_next;
	x_next = p->back().p2.pose.x;

	y_prev = y;
	y      = y_next;
	y_next = p->back().p2.pose.y;

	//printf("---%f %f\n", x_next, y_next);
	a = x_next - (2*x) + x_prev;
	b = y_next - (2*y) + y_prev;
	sum += (a*a + b*b);

	//getchar();

	return (sum);
}


// The gradient of f, df = (df/dx, df/dy)
//derivative in each point [2x(i-2)-8x(i-1)+12x(i)-8x(i+1)+2x(i+2)]
void
my_df (const gsl_vector *v, void *params, gsl_vector *df)
{
	list<RRT_Path_Edge> *p = (list<RRT_Path_Edge> *)params;
	int i, j, size =(p->size()-1);

	double x_prev2= 0;
	double x_prev = p->front().p1.pose.x;
	double x      = gsl_vector_get(v, 0);
	double x_next = gsl_vector_get(v, 1);
	double x_next2= gsl_vector_get(v, 2);
	double sum_x  =  (10*x) - (8*x_next) + (2*x_next2) - (4*x_prev);
	gsl_vector_set(df, 0, sum_x);

	double y_prev2= 0;
	double y_prev = p->front().p1.pose.y;
	double y      = gsl_vector_get(v, size);
	double y_next = gsl_vector_get(v, size+1);
	double y_next2= gsl_vector_get(v, size+2);
	double sum_y  = (10*y) - (8*y_next) + (2*y_next2) - (4*y_prev);
	gsl_vector_set(df, size, sum_y);
//	printf("---%f %f\n", x_prev, y_prev);
//	printf("---%f %f\n", x, y);
//	printf("---%f %f\n", x_next, y_next);
//	printf("---%f %f\n", x_next2, y_next2);

	for (i = 3, j = (size+3); i < size; i++, j++)
	{
		x_prev2= x_prev;
		x_prev = x;
		x      = x_next;
		x_next = x_next2;
		x_next2= gsl_vector_get(v, i);
		sum_x = (2*x_prev2) - (8*x_prev) + (12*x) - (8*x_next) + (2*x_next2);
		gsl_vector_set(df, (i-2), sum_x);

		y_prev2= y_prev;
		y_prev = y;
		y      = y_next;
		y_next = y_next2;
		y_next2= gsl_vector_get(v, j);
		sum_y = (2*y_prev2) - (8*y_prev) + (12*y) - (8*y_next) + (2*y_next2);
		gsl_vector_set(df, (j-2), sum_y);
//		printf("---%f %f\n", x_next2, y_next2);
	}

	x_prev2= x_prev;
	x_prev = x;
	x      = x_next;
	x_next = x_next2;
	x_next2= p->back().p2.pose.x;
	sum_x  = (2*x_prev2) - (8*x_prev) + (12*x) - (8*x_next) + (2*x_next2);
	gsl_vector_set(df, size-2, sum_x);

	y_prev2= y_prev;
	y_prev = y;
	y      = y_next;
	y_next = y_next2;
	y_next2= p->back().p2.pose.y;
	sum_y  = (2*y_prev2) - (8*y_prev) + (12*y) - (8*y_next) + (2*y_next2);
	gsl_vector_set(df, (2*size)-2, sum_y);
//	printf("---%f %f\n", x_next2, y_next2);

	x_prev2= x_prev;
	x_prev = x;
	x      = x_next;
	x_next = x_next2;
	sum_x  = (2*x_prev2) - (8*x_prev) + (10*x) - (4*x_next);
	gsl_vector_set(df, size-1, sum_x);

	y_prev2= y_prev;
	y_prev = y;
	y      = y_next;
	y_next = y_next2;
	sum_y  = (2*y_prev2) - (8*y_prev) + (10*y) - (4*y_next);
	gsl_vector_set(df, (2*size)-1, sum_y);

//	getchar();
}


// Compute both f and df together
void
my_fdf (const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
	*f = my_f(x, params);
	my_df(x, params, df);
}


void RRT::smooth_principal_path_from_tree_using_conjugate_gradient (RRT_Node *goal)
{
	size_t iter = 0;
	int status, i=0, j=0, size;

	const gsl_multimin_fdfminimizer_type *T;
	gsl_multimin_fdfminimizer *s;

	gsl_vector *v;
	gsl_multimin_function_fdf my_func;

	list<RRT_Path_Edge>::iterator it;
	list<RRT_Path_Edge> path;
	path = Dijkstra::build_path(goal);

	if (path.size() < 4)
		return;

	size = path.size()+1;

	my_func.n = (2*size)-4;
	my_func.f = my_f;
	my_func.df = my_df;
	my_func.fdf = my_fdf;
	my_func.params = &path;

	v = gsl_vector_alloc ((2*size)-4);
	it = path.begin();

//	fprintf(plot, "a%d = [\n", cont);
//	printf ("size %d\n\n", size);
//	fprintf(plot, "%f %f\n", it->p1.pose.x, it->p1.pose.y);
//	fprintf(normal, "%f %f\n", it->p1.pose.x, it->p1.pose.y);
//	printf ("a%f %f\n", it->p1.pose.x, it->p1.pose.y);
	for (i=0, j=(size-2); i < (size-2); i++, j++, it++)
	{
//		fprintf(plot, "%f %f\n", it->p2.pose.x, it->p2.pose.y);
//		fprintf(normal, "%f %f\n", it->p2.pose.x, it->p2.pose.y);
//		printf ("a%f %f\n", it->p2.pose.x, it->p2.pose.y);
		gsl_vector_set (v, i, it->p2.pose.x);
		gsl_vector_set (v, j, it->p2.pose.y);
	}
//	fprintf(plot, "%f %f]\n\n", it->p2.pose.x, it->p2.pose.y);
//	fprintf(normal, "%f %f\n", it->p2.pose.x, it->p2.pose.y);
//	printf ("a%f %f\n", it->p2.pose.x, it->p2.pose.y);

	T = gsl_multimin_fdfminimizer_conjugate_fr;
	s = gsl_multimin_fdfminimizer_alloc (T, (2*size)-4);

	gsl_multimin_fdfminimizer_set (s, &my_func, v, 0.1, 0.01);  //(function_fdf, gsl_vector, step_size, tol)

	do
	{
		iter++;
		status = gsl_multimin_fdfminimizer_iterate (s);

		if (status){
			//printf("%%Saiu STATUS %d\n", status);
			break;
		}

		status = gsl_multimin_test_gradient (s->gradient, 0.01);        //(gsl_vector, epsabs) and  |g| < epsabs

		if (status == GSL_SUCCESS)
		{
			//printf ("%%Minimum found!!!\n");
		}
	}
	while (status == GSL_CONTINUE && iter < 999);

	it = path.begin();

//	fprintf(plot, "b%d = [   \n%f %f\n", cont, it->p1.pose.x, it->p1.pose.y);
//	fprintf(smooth, "%f %f\n", it->p1.pose.x, it->p1.pose.y);
//	printf ("z%f %f\n", it->p1.pose.x, it->p1.pose.y);
	for (i=0, j=(size-2); i < (size-2); i++, j++)
	{
		it->p2.pose.x = gsl_vector_get (s->x, i);
		it->p2.pose.y = gsl_vector_get (s->x, j);
//		fprintf(plot, "%f %f\n", it->p2.pose.x, it->p2.pose.y);
//		fprintf(smooth, "%f %f\n", it->p2.pose.x, it->p2.pose.y);
//		printf ("z%f %f\n", it->p2.pose.x, it->p2.pose.y);
		if (it != path.end())
		{
			it++;
			it->p1.pose.x = gsl_vector_get (s->x, i);
			it->p1.pose.y = gsl_vector_get (s->x, j);
			//fprintf(plot, "%f %f\n", it->p1.pose.x, it->p1.pose.y);
//			printf ("z%f %f\n", it->p1.pose.x, it->p1.pose.y);
		}
	}
//	fprintf(plot, "%f %f]\n\n", it->p2.pose.x, it->p2.pose.y);
//	fprintf(smooth, "%f %f\n", it->p2.pose.x, it->p2.pose.y);
//	fprintf(plot, "\nplot (a%d(:,1), a%d(:,2), b%d(:,1), b%d(:,2)); \nstr = input (\"a   :\");\n\n", cont, cont, cont, cont);
//	printf ("z%f %f\n", it->p2.pose.x, it->p2.pose.y);
//	printf ("\n");

	recalculate_theta_time_phi (path);

	save_smoothed_path_back_to_tree (goal, path);

	gsl_multimin_fdfminimizer_free (s);
	gsl_vector_free (v);
}
