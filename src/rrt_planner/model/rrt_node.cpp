/*
 * rrt_node.cpp
 *
 *  Created on: 16/12/2011
 *      Author: rradaelli
 */

#include "rrt_node.h"
#include "math.h"
#include "../util/util.h"
#include "../model/global_state.h"

double RRT_Node::n = 50;
int	   RRT_Node::k = 3;

RRT_Node::RRT_Node()
{
	parent = 0;
	cvf	   = 0;
	cost   = DBL_MAX;
	closed = false;
}

bool RRT_Node::prune_node()
{
	return GlobalState::goal_node &&
			(cost + distance(GlobalState::goal_node->robot_state.pose) / GlobalState::robot_config.max_vel) > GlobalState::goal_node->cost;
}

RRT_Node::~RRT_Node()
{
	followedCommands.clear();
}

bool RRT_Node::add_command(Command &c, double &time)
{
	bool inserted = followedCommands.insert(pair<unsigned long int, Command>(c.get_key(time), c)).second;

	return inserted;
}

bool RRT_Node::is_command_followed(Command &c, double &time)
{
	return followedCommands.find(c.get_key(time)) != followedCommands.end();
}


list<RRT_Edge>::iterator find(RRT_Node *node, list<RRT_Edge> &edge_list)
{
	list<RRT_Edge>::iterator it;

	if(edge_list.size() > 0)
	{
		for(it = edge_list.begin(); it != edge_list.end(); it++)
		{
			if(it->n1 == node || it->n2 == node)
				return it;
		}
	}

	return edge_list.end();
}

void RRT_Node::remove_edge(RRT_Node *node)
{
	list<RRT_Edge>::iterator it;

	while(true)
	{
		it = find(node, adjacency_nodes);

		if(it == adjacency_nodes.end())
			break;

		adjacency_nodes.erase(it);
	}

	while(true)
	{
		it = find(node, prev_nodes);

		if(it == prev_nodes.end())
			break;

		prev_nodes.erase(it);
	}
}

void RRT_Node::update_cvf(int i)
{
	if (i > k)
		return;

	cvf += 1.0 / pow(n, i);

	std::list<RRT_Edge>::iterator it = adjacency_nodes.begin();

	for (; it != adjacency_nodes.end(); it++)
	{
		if (it->n1 == this)
		{
			continue;
		}

		it->n1->update_cvf(i + 1);
	}
}

static unsigned long int
get_pose_key(Pose p)
{
	static double theta_precision = 0.25; //pula a cada 4 graus
	static double position_precision = 5; //pula a cada 0.2 m

	unsigned long int theta_size, theta_key, position_key, y_size;

	p = Util::to_map_pose(p);

	//converter novamente para metros
	p.x *= GlobalState::cost_map.config.resolution;
	p.y *= GlobalState::cost_map.config.resolution;
	y_size = GlobalState::cost_map.config.y_size * GlobalState::cost_map.config.resolution;

	//converte para precisao especificada e arredonda
	p.x = ceil(p.x * position_precision);
	p.y = ceil(p.y * position_precision);
	y_size = ceil(y_size * position_precision);


	theta_size = ceil(360 * theta_precision);

	position_key = p.x * y_size + p.y;
	theta_key = ceil(carmen_radians_to_degrees(p.theta < 0 ? (-p.theta + M_PI + 0.0174532925) : p.theta) * theta_precision);

	return position_key * theta_size + theta_key;
}

unsigned long int RRT_Node::get_key(Robot_State& robot_state)
{
	static Command max_c(-GlobalState::robot_config.max_vel, -GlobalState::robot_config.max_phi);
	static unsigned long int max_command_key = max_c.get_key();

	return get_pose_key(robot_state.pose) * max_command_key + robot_state.v_and_phi.get_key();
}

unsigned long int RRT_Node::get_key()
{
	return RRT_Node::get_key(RRT_Node::robot_state);
}


double RRT_Node::distance(const RRT_Node &n)
{
	return robot_state.pose.distance(n.robot_state.pose);
}

double RRT_Node::distance(const Pose &p)
{
	return robot_state.pose.distance(p);
}

bool RRT_Node::operator==(RRT_Node &node)
{
	return get_key() == node.get_key();
}

bool RRT_Node::operator!=(RRT_Node &node)
{
	return !operator==(node);
}

bool RRT_Edge::operator==(const RRT_Edge &node)
{
	return node.n1 == n1 && node.n2 == n2;
}

bool RRT_Edge::operator!=(const RRT_Edge &node)
{
	return !operator==(node);
}

RRT_Edge::RRT_Edge(RRT_Node *n1, RRT_Node *n2, double cost)
{
	this->n1   = n1;
	this->n2   = n2;
	this->cost = cost;
	time = 0.0;
}

