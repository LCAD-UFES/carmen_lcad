/*
 * publisher_util.h
 *
 *  Created on: 08/03/2012
 *      Author: romulo
 */

#ifndef PUBLISHER_UTIL_H_
#define PUBLISHER_UTIL_H_

using namespace std;

#include <list>
#include "../message/rrt_planner_message.h"
#include "../model/tree.h"
#include "../rrt.h"
#include "util.h"
#include "../model/cost_map.h"

class Publisher_Util
{
public:
	static void publish_plan_tree_message(Tree &t, const vector<RRT_Node *> reaches_goal_nodes);

	static void publish_navigator_ackerman_plan_message(list<RRT_Edge> &path);

	static void publish_navigator_ackerman_plan_message(list<RRT_Path_Edge> &path);

	static void publish_navigator_ackerman_plan_message(carmen_navigator_ackerman_plan_message msg);

	static void publish_navigator_ackerman_status_message();

	static void publish_plan_message(list<RRT_Edge> &path);

	static void publish_plan_message(carmen_navigator_ackerman_plan_message msg);

	static void publish_utility_map(Gradient_Cost_Map u);

	static void publish_obstacle_cost_map(carmen_map_t c);

	//for path follower
	static void publish_rrt_path_message(rrt_path_message *msg);

	static void publish_path(list<RRT_Path_Edge> &path);

	static void publish_principal_path_message(list<RRT_Path_Edge> &path);

private:
	static carmen_navigator_ackerman_plan_message	get_path(list<RRT_Edge> &path);
	static carmen_navigator_ackerman_plan_message	get_path(list<RRT_Path_Edge> &path);
	static carmen_rrt_planner_tree_message *get_tree_message(Tree &t);
	static int get_path_or_old_path(const vector<RRT_Node *> reaches_goal_nodes, carmen_navigator_ackerman_plan_tree_message &plan_tree_msg);
};


#endif /* PUBLISHER_UTIL_H_ */
