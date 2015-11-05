/*
 * dijkstra.h
 *
 *  Created on: 14/03/2012
 *      Author: romulo
 */

#ifndef DIJKSTRA_H_
#define DIJKSTRA_H_

using namespace std;

#include <vector>
#include "../model/tree.h"
#include <carmen/carmen.h>

class Dijkstra
{
public:
	static void find_path(Tree &tree);

	static list<RRT_Path_Edge> build_path(RRT_Node *goal);

private:
	static RRT_Node *find_min(vector<RRT_Node *> &open_list);
};

#endif /* DIJKSTRA_H_ */
