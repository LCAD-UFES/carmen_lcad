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
#include "rrt_planner_message.h"
#include "model/tree.h"
//#include "../rrt.h"
//#include "util.h"
#include "model/cost_map.h"

class Publisher_Util
{
public:
	static void publish_plan_tree_message(Tree tree);

	static void publish_navigator_ackerman_plan_message(carmen_navigator_ackerman_plan_message msg);

	static void publish_navigator_ackerman_status_message();

	static void publish_plan_message(carmen_navigator_ackerman_plan_message msg);

	static void publish_utility_map(Gradient_Cost_Map u);
	static void publish_obstacle_cost_map(carmen_map_t c);
};


#endif /* PUBLISHER_UTIL_H_ */
