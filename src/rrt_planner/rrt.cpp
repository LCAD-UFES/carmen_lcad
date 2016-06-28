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


// get the desired heading given tree points in forward drive mode
double
get_desired_heading(carmen_ackerman_traj_point_t &left, carmen_ackerman_traj_point_t &current, carmen_ackerman_traj_point_t &right)
{
    // get the displacement vector
    double x1 = 0.25*(right.x - left.x) + 0.75*(right.x - current.x);
    double y1 = 0.25*(right.y - left.y) + 0.75*(right.y - current.y);

    return atan2(y1, x1);
}

// the max allowed speed
double
get_max_speed(carmen_ackerman_traj_point_t &left, carmen_ackerman_traj_point_t &current, carmen_ackerman_traj_point_t &right)
{
    double x1, x2, y1, y2;

    // the max lateral acceleration, 0.5 m/s^2
    double max_lateral_acceleration = 0.5;

    // the max speed at the current curvature
    double curvature_constraint;

    // the current vector
    x1 = current.x - left.x;
    y1 = current.y - left.y;

    // the next vector
    x2 = right.x - current.x;
    y2 = right.y - current.y;

    // get the angle between the two vectors
    double angle = fabs(carmen_normalize_theta(atan2(y2, x2) - atan2(y1, x1)));

    // get the turn radius
	double radius = sqrt(x1*x1 + y1*y1) / angle;

    // get the curvature constraint
    curvature_constraint = sqrt(radius*max_lateral_acceleration);


    // max_forward_speed = sqrt(left.v * left.v + 2*maximum_acceleration_forward*displacement);

    // assign the max forward speed
    return min(GlobalState::robot_config.max_vel, curvature_constraint);
}

// get the max forward acceleration speed
double
get_accelerated_speed(carmen_ackerman_traj_point_t &left, carmen_ackerman_traj_point_t &right, double acceleration)
{
    // get the displacement between the currrent and the next point
    double displacement = sqrt(pow(right.y - left.y, 2) + pow(right.x - left.x, 2));

    // get the forward speed constrained by the max acceleration
    // Torricelli
    return sqrt(left.v*left.v + 2.0*acceleration*displacement);
}

double
get_turning_radius(carmen_ackerman_traj_point_t &a, carmen_ackerman_traj_point_t &b)
{
    // move the second position to the first position reference
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    //double radius;

    double angle = -a.theta;

    // rotate the goal point around the z axis
    double gx = dx*cos(angle) - dy*sin(angle);
    double gy = dx*sin(angle) + dy*cos(angle);

    // get the desired wheel angle
    return (gx*gx + gy*gy)/(2.0*gy);



}

// consolidate the entire path
// what we need:
// the car curvature along the path
// the car speed
// the car phi
// each command time
void
consolidate_path(rrt_path_message *msg)
{
    int i, last = msg->size;
    double radius;

    for (i = 1; i < last; i++)
    {
        // get the desired heading
    	msg->path[i-1].p2.theta = msg->path[i].p1.theta = get_desired_heading(msg->path[i-1].p1, msg->path[i].p1, msg->path[i].p2);

        // get the max speed
    	//msg->path[i-1].p2.v = msg->path[i].p1.v = get_max_speed(msg->path[i-1].p1, msg->path[i].p1, msg->path[i].p2);
        //printf ("-----%f\n", msg->path[i].p1.theta);

        radius = get_turning_radius(msg->path[i].p1, msg->path[i].p2);
        msg->path[i-1].p2.phi = msg->path[i].p1.phi = msg->path[i-1].phi = atan(GlobalState::robot_config.distance_between_front_and_rear_axles/(radius));


        //msg->path[i].time = msg->path[i].v radius;

    }

    // update the acceleration around the start and the end point
    /*double current_v;

    // from the start to the goal, update the forward acceleration
    for (i = 1; i < last; i++)
    {
        current_v = get_accelerated_speed(msg->path[i-1].p1, msg->path[i].p1, GlobalState::robot_config.desired_acceleration);

        if (current_v < msg->path[i].v)
            msg->path[i].v = current_v;
        else
            break;
    }

    // from the goal to the start, update the forward deceleration
    for (i = last-1; i > 0; i--)
    {
        current_v = get_accelerated_speed(msg->path[i].p2, msg->path[i].p1, GlobalState::robot_config.desired_decelaration_forward);

        if (current_v < msg->path[i].v)
        	msg->path[i].v = current_v;
        else
            break;
    }
	*/
}


//Function to be minimized
//summation[x(i+1)-2x(i)+x(i-1)]
double
my_f(const gsl_vector *v, void *params)
{
	rrt_path_message *p = (rrt_path_message *)params;
	int i, j, size =(p->size-1);                                  //(p->size-1)Instead of -2 because the message size is 4 but the message has 5 points
	double a=0, b=0, sum=0;                                       //and we have to discount the first and last point that wont be optimized

	//printf("%d\n", size+1);
	double x_prev = p->path[0].p1.x;			//x(i-1)
	double x      = gsl_vector_get(v, 0);		//x(i)
	double x_next = gsl_vector_get(v, 1);		//x(i+1)

	double y_prev = p->path[0].p1.y;
	double y      = gsl_vector_get(v, size);
	double y_next = gsl_vector_get(v, size+1);
	//printf("---%f %f\n", x_prev, y_prev);
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
	}

	x_prev = x;
	x      = x_next;
	x_next = p->path[size].p2.x;

	y_prev = y;
	y      = y_next;
	y_next = p->path[size].p2.y;

	//printf("---%f %f\n", x_next, y_next);
	a = x_next - (2*x) + x_prev;
	b = y_next - (2*y) + y_prev;
	sum += (a*a + b*b);

	return (sum);
}


// The gradient of f, df = (df/dx, df/dy)
//derivative in each point [2x(i-2)-8x(i-1)+12x(i)-8x(i+1)+2x(i+2)]
void
my_df (const gsl_vector *v, void *params, gsl_vector *df)
{
	rrt_path_message *p = (rrt_path_message *)params;
	int i, j, size =(p->size-1);

	double x_prev2= 0;
	double x_prev = p->path[0].p1.x;
	double x      = gsl_vector_get(v, 0);
	double x_next = gsl_vector_get(v, 1);
	double x_next2= gsl_vector_get(v, 2);
	double sum_x  =  (10*x) - (8*x_next) + (2*x_next2) - (4*x_prev);
	gsl_vector_set(df, 0, sum_x);
	//printf ("%f\n", sum_x);

	double y_prev2= 0;
	double y_prev = p->path[0].p1.y;
	double y      = gsl_vector_get(v, size);
	double y_next = gsl_vector_get(v, size+1);
	double y_next2= gsl_vector_get(v, size+2);
	double sum_y  = (10*y) - (8*y_next) + (2*y_next2) - (4*y_prev);
	gsl_vector_set(df, size, sum_y);

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
	}

	x_prev2= x_prev;
	x_prev = x;
	x      = x_next;
	x_next = x_next2;
	x_next2= p->path[size].p2.x;
	sum_x  = (2*x_prev2) - (8*x_prev) + (12*x) - (8*x_next) + (2*x_next2);
	gsl_vector_set(df, size-2, sum_x);

	y_prev2= y_prev;
	y_prev = y;
	y      = y_next;
	y_next = y_next2;
	y_next2= p->path[size].p2.y;
	sum_y  = (2*y_prev2) - (8*y_prev) + (12*y) - (8*y_next) + (2*y_next2);
	gsl_vector_set(df, (2*size)-2, sum_y);

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
}


// Compute both f and df together
void
my_fdf (const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
	*f = my_f(x, params);
	my_df(x, params, df);
}


void RRT::smooth_path_using_conjugate_gradient (rrt_path_message *msg)
{
		size_t iter = 0;
		int status, i, j, size =(msg->size+1);

		//printf("size %d\n", size);
		//printf ("%aaaaaaaaaaaaaalf\n", msg->path[0].p1.x);

		const gsl_multimin_fdfminimizer_type *T;
		gsl_multimin_fdfminimizer *s;

		gsl_vector *v;
		gsl_multimin_function_fdf my_func;

		my_func.n = (2*size)-4;
		my_func.f = my_f;
		my_func.df = my_df;
		my_func.fdf = my_fdf;
		my_func.params = msg;

		v = gsl_vector_alloc ((2*size)-4);

		for (i=0, j=(size-2); i < (size-2); i++, j++)
		{
			//printf ("%f %f\n", msg->path[i].p2.x, msg->path[i].p2.y);
			gsl_vector_set (v, i, msg->path[i].p2.x);
			gsl_vector_set (v, j, msg->path[i].p2.y);
		}

		T = gsl_multimin_fdfminimizer_conjugate_fr;
		s = gsl_multimin_fdfminimizer_alloc (T, (2*size)-4);
		//printf("FOOIIIIIIII\n");
		gsl_multimin_fdfminimizer_set (s, &my_func, v, 0.1, 0.01);  //(function_fdf, gsl_vector, step_size, tol)
		//printf("-----------\n");
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

		for (i=0, j=(size-2); i < (size-2); i++, j++)
		{
			msg->path[i].p2.x = msg->path[i+1].p1.x = gsl_vector_get (s->x, i);
			msg->path[i].p2.y = msg->path[i+1].p1.y = gsl_vector_get (s->x, j);
		}

		consolidate_path(msg);
		/*printf ("%d  -------------------------------\n%f %f\n", (int)iter, msg->path[0].p1.x, msg->path[0].p1.y);
		for (i=0, j=(size-2); i < (size-2); i++, j++)
		{
			printf ("%f %f\n", gsl_vector_get (s->x, i), gsl_vector_get (s->x, j));
		}
		printf ("%f %f\n", msg->path[size-2].p2.x, msg->path[size-2].p2.y);
		*/

		gsl_multimin_fdfminimizer_free (s);
		gsl_vector_free (v);
}
