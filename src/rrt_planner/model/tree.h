/*
 * tree.h
 *
 *  Created on: 13/03/2012
 *      Author: romulo
 */

#ifndef TREE_H_
#define TREE_H_

using namespace std;
#include <vector>
#include <map>
#include "rrt_node.h"

typedef map<unsigned long int, unsigned int> Map_Index;
class Tree
{
public:
	Tree();
	virtual ~Tree();
	void clear();
	void add_root_node(const Pose &pose, const Command &command);
	bool add_node(RRT_Node *n);
	bool add_node(Pose &pose);

	/*
	 * Remove the node and its edges from the tree
	 */
	void	  add_edge(RRT_Edge e);
//	RRT_Node *find(RRT_Node *n);
	RRT_Node *nearest_node(Pose &n);
	void	  initialize_dijkstra();


public:
	vector<RRT_Node *>	 nodes;
	RRT_Node *root_node;

	Map_Index nodes_map;
};

#endif /* TREE_H_ */
