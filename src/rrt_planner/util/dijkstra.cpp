/*
 * dijkstra.cpp
 *
 *  Created on: 14/03/2012
 *      Author: romulo
 */

#include "dijkstra.h"
#include <float.h>
#include <stdlib.h>
#include <stdio.h>


RRT_Node *Dijkstra::find_min(vector<RRT_Node *> &open_list)
{
	double	  min = DBL_MAX;
	RRT_Node *min_node = NULL;
	int min_index = -1;

	for (register unsigned int i = 0; i < open_list.size(); i++)
	{
		if (open_list[i]->cost < min)
		{
			min_node = open_list[i];
			min = min_node->cost;
			min_index = i;
		}
	}

	open_list.erase(open_list.begin() + min_index);

	return min_node;
}

void Dijkstra::find_path(Tree &tree)
{
	vector<RRT_Node *>	 open_list;
	map<double, RRT_Node *>::iterator it;
	RRT_Node *root_node = tree.root_node;
	double	  value;

	tree.initialize_dijkstra();

	root_node->cost = 0;
	open_list.push_back(root_node);

	while (open_list.size() > 0)
	{
		RRT_Node *actual = find_min(open_list);

		for (list<RRT_Edge>::iterator it = actual->adjacency_nodes.begin(); it != actual->adjacency_nodes.end(); it++)
		{
			RRT_Node *n2 = it->n2;

			value = actual->cost + it->cost;

			if (n2->cost > value)
			{
				if (n2->cost == DBL_MAX)
				{
					open_list.push_back(n2);
				}

				n2->cost   = value;
				n2->parent = actual;
			}
		}
	}
}

list<RRT_Path_Edge> Dijkstra::build_path(RRT_Node *goal)
{
	RRT_Path_Edge path_edge;
	list<RRT_Path_Edge> path;

	list<RRT_Edge>::iterator it;

	do
	{
		RRT_Node *parent = goal->parent;

		it = goal->prev_nodes.begin();

		for (; it != goal->prev_nodes.end(); it++)
		{
			if (it->n1 == parent)
			{
				path_edge.p1 = it->n1->robot_state;
				path_edge.p2 = it->n2->robot_state;
				path_edge.command = it->command;
				path_edge.time = it->time;
				path.push_front(path_edge);
				break;
			}
		}

		goal = goal->parent;
	} while (goal);

	return path;
}

