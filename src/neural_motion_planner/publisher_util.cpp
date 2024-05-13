/*
 * publisher_util.cpp
 *
 *  Created on: 08/03/2012
 *      Author: romulo
 */

#include "publisher_util.h"
#include "rrt_planner_interface.h"
#include <map>
#include "model/global_state.h"
//#include "dijkstra.h"

void
define_plan_tree_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME);
}


void
Publisher_Util::publish_plan_tree_message(Tree tree)
{
	static carmen_navigator_ackerman_plan_tree_message plan_tree_msg;
	IPC_RETURN_TYPE err = IPC_OK;
	static bool first_time = true;

	if (first_time)
	{
		define_plan_tree_message();
		plan_tree_msg.host = carmen_get_host();
		first_time = false;
	}

	plan_tree_msg.timestamp = GlobalState::localizer_pose_timestamp;//carmen_get_time();
	plan_tree_msg.num_edges = tree.num_edges;

	plan_tree_msg.p1 = tree.p1;
	plan_tree_msg.p2 = tree.p2;
	plan_tree_msg.mask = tree.mask;

	plan_tree_msg.num_path = tree.num_paths;
	if (plan_tree_msg.num_path > CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_NUM_PATHS)
	{	// Ver tipo carmen_navigator_ackerman_plan_tree_message
		printf("Error: plan_tree_msg.num_path > %d in Publisher_Util::publish_plan_tree_message()\n", CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_NUM_PATHS);
		exit(1);
	}
	for (int i = 0; i < tree.num_paths; i++)
	{
		if (tree.paths_sizes[i] > CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE)
		{	// Ver tipo carmen_navigator_ackerman_plan_tree_message
			printf("Error: paths_sizes[%d] > %d in Publisher_Util::publish_plan_tree_message()\n", i, CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_MAX_PATH_SIZE);
			exit(1);
		}
		memcpy(plan_tree_msg.paths[i], tree.paths[i], sizeof(carmen_robot_and_trailers_traj_point_t) * tree.paths_sizes[i]);
		plan_tree_msg.path_size[i] = tree.paths_sizes[i];
	}

	err = IPC_publishData(CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME, &plan_tree_msg);

	carmen_test_ipc(err, "Could not publish", CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME);
}


void
Publisher_Util::publish_navigator_ackerman_plan_message(carmen_navigator_ackerman_plan_message msg)
{
	static bool first_time = true;
	IPC_RETURN_TYPE err;

	if (first_time)
	{
		err = IPC_defineMsg(CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME,
				IPC_VARIABLE_LENGTH,
				CARMEN_NAVIGATOR_ACKERMAN_PLAN_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME);
		first_time = false;
	}

	err = IPC_publishData(CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME, &msg);
	carmen_test_ipc(err, "Could not publish",
			CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME);
}


void
Publisher_Util::publish_plan_message(carmen_navigator_ackerman_plan_message msg)
{
	static bool first_time = true;
	IPC_RETURN_TYPE err;

	if (first_time)
	{
		carmen_rrt_planner_define_plan_message();
	}

	err = IPC_publishData(CARMEN_RRT_PLANNER_PLAN_NAME, &msg);
	carmen_test_ipc(err, "Could not publish",
			CARMEN_RRT_PLANNER_PLAN_NAME);
}


static void
define_path_message()
{
	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(RRT_PATH_NAME, IPC_VARIABLE_LENGTH,
			RRT_PATH_FMT);
	carmen_test_ipc_exit(err, "Could not define", RRT_PATH_NAME);
}


void
Publisher_Util::publish_rrt_path_message(rrt_path_message *msg)
{
	static int firsttime = 1;
	IPC_RETURN_TYPE err;

	if (firsttime)
	{
		define_path_message();
		firsttime = 0;
	}

	err = IPC_publishData(RRT_PATH_NAME, msg);
	carmen_test_ipc(err, "Could not publish",
			RRT_PATH_NAME);
}
