/*
 * tree.cpp
 *
 *  Created on: 13/03/2012
 *      Author: romulo
 */

#include "tree.h"
#include <stdlib.h>
#include <float.h>
#include "../util/util.h"
#include <sys/time.h>
#include <omp.h>
#include "global_state.h"

Tree::Tree()
{
	root_node = NULL;
}

Tree::~Tree()
{
	clear();
}

void Tree::add_root_node(const Pose &pose, const Command &command)
{
	RRT_Node *root = new RRT_Node();

	root->robot_state.pose = pose;
	root->robot_state.v_and_phi = command;
	root->cost	 = 0;
	root->parent = NULL;
	add_node(root);

	root_node = root;
}

void Tree::add_edge(RRT_Edge e)
{
	e.n1->adjacency_nodes.push_back(e);
	e.n2->prev_nodes.push_back(e);
}


bool Tree::add_node(RRT_Node *n)
{
	Map_Index::iterator it;
	unsigned long int key = n->get_key();
	it = nodes_map.find(key);

	if (it != nodes_map.end())
	{
		return false;
	}

	nodes_map[key] = nodes.size();

	nodes.push_back(n);

	return true;
}

bool Tree::add_node(Pose &pose)
{
	RRT_Node *root = new RRT_Node();

	root->robot_state.pose = pose;
	root->robot_state.v_and_phi.v	  = 0;
	root->robot_state.v_and_phi.phi = 0;

	return add_node(root);
}


RRT_Node *Tree::nearest_node(Pose &n)
{
	RRT_Node *nearest_node, *nearest_node2, *node;
	double	  min_dist, min_dist2, dist;

	nearest_node = NULL;
	min_dist	 = DBL_MAX;
	nearest_node2 = NULL;
	min_dist2	 = DBL_MAX;

	for (unsigned int i = 0; i < nodes.size(); i++)
	{
		node = nodes[i];

		if (node->closed)//talvez seja melhor criar um vetor sem esses nós
			continue;

		dist = node->distance(n);

		if (dist < min_dist2)
		{
			min_dist2	 = dist;
			nearest_node2 = node;
		}


		if (Util::normalized_random() < node->cvf)
			continue;


		if (dist < min_dist)
		{
			min_dist	 = dist;
			nearest_node = node;
		}
	}

	return nearest_node != NULL ? nearest_node : nearest_node2;
}

void Tree::initialize_dijkstra()
{
	for (unsigned int i = 0; i < nodes.size(); i++)
	{
		nodes[i]->cost	  = DBL_MAX;
		nodes[i]->parent = NULL;
	}
}

void Tree::clear()
{

	for(unsigned int i = 0; i < nodes.size(); i++)
	{
		delete nodes[i];
	}

	nodes.clear();
	nodes_map.clear();
	root_node = NULL;
}

