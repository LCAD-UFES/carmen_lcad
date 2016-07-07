/*
 * publisher_util.cpp
 *
 *  Created on: 08/03/2012
 *      Author: romulo
 */

#include "publisher_util.h"
#include "../message/rrt_planner_interface.h"
#include <map>
#include "../model/global_state.h"
#include <carmen/grid_mapping_interface.h>
#include "dijkstra.h"

void
define_plan_tree_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME);
}


void
Publisher_Util::publish_plan_tree_message(Tree &t, const vector<RRT_Node *> &reaches_goal_nodes)
{
	static carmen_navigator_ackerman_plan_tree_message plan_tree_msg;
	IPC_RETURN_TYPE err = IPC_OK;
	static bool		first_time = true;

	if (first_time)
	{

		define_plan_tree_message();
		plan_tree_msg.host = carmen_get_host();
		first_time = false;
	}

	plan_tree_msg.timestamp = carmen_get_time();
	plan_tree_msg.num_edges = 0;

	for(unsigned int i = 0; i < t.nodes.size(); i++)
	{
		plan_tree_msg.num_edges += t.nodes[i]->adjacency_nodes.size();
	}

	plan_tree_msg.p1 = (carmen_ackerman_traj_point_t *) malloc (sizeof(carmen_ackerman_traj_point_t) * plan_tree_msg.num_edges);
	plan_tree_msg.p2 = (carmen_ackerman_traj_point_t *) malloc (sizeof(carmen_ackerman_traj_point_t) * plan_tree_msg.num_edges);
	plan_tree_msg.mask = (int *) malloc(sizeof(int) * plan_tree_msg.num_edges);

	int aux = 0;

	//	printf("------------- %ld\n", nodes.size());
	for(unsigned int i = 0; i < t.nodes.size(); i++)
	{
		list<RRT_Edge> *adjacency_list = &t.nodes[i]->adjacency_nodes;

		list<RRT_Edge>::iterator adj_it = adjacency_list->begin();

		for (; adj_it != adjacency_list->end(); adj_it++)
		{
			plan_tree_msg.p1[aux].x	 = adj_it->n1->robot_state.pose.x;
			plan_tree_msg.p1[aux].y	 = adj_it->n1->robot_state.pose.y;
			plan_tree_msg.p1[aux].theta = adj_it->n1->robot_state.pose.theta;
			plan_tree_msg.p1[aux].v = adj_it->n1->robot_state.v_and_phi.v;
			plan_tree_msg.p1[aux].phi = adj_it->n1->robot_state.v_and_phi.phi;

			plan_tree_msg.p2[aux].x	 = adj_it->n2->robot_state.pose.x;
			plan_tree_msg.p2[aux].y	 = adj_it->n2->robot_state.pose.y;
			plan_tree_msg.p2[aux].theta = adj_it->n2->robot_state.pose.theta;
			plan_tree_msg.p2[aux].v = adj_it->n2->robot_state.v_and_phi.v;
			plan_tree_msg.p2[aux].phi = adj_it->n2->robot_state.v_and_phi.phi;

			plan_tree_msg.mask[aux] = fmax(adj_it->n1->cvf, adj_it->n2->cvf) >= 1.0 ? 0 : 1;

			if (adj_it->n1->closed || adj_it->n2->closed)
				plan_tree_msg.mask[aux] = -1;

			aux++;
		}
	}

	carmen_navigator_ackerman_plan_message msg;
	list<RRT_Path_Edge> path;
	plan_tree_msg.num_path = reaches_goal_nodes.size();
	if (plan_tree_msg.num_path > 500)
	{	// Ver tipo carmen_navigator_ackerman_plan_tree_message
		printf("Error: plan_tree_msg.num_path > 500 in Publisher_Util::publish_plan_tree_message()\n");
		exit(1);
	}
	for (unsigned int i = 0; i < reaches_goal_nodes.size(); i++)
	{
		path = Dijkstra::build_path(reaches_goal_nodes[i]);
		msg = get_path(path);

		if (msg.path_length > 100)
		{	// Ver tipo carmen_navigator_ackerman_plan_tree_message
			printf("Error: msg.path_length > 100 in Publisher_Util::publish_plan_tree_message()\n");
			exit(1);
		}
		memcpy(plan_tree_msg.paths[i], msg.path, sizeof(carmen_ackerman_traj_point_t) * msg.path_length);
		free(msg.path);
		plan_tree_msg.path_size[i] = msg.path_length;
	}

	err = IPC_publishData(CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME, &plan_tree_msg);

	carmen_test_ipc(err, "Could not publish", CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME);

	free(plan_tree_msg.p1);
	free(plan_tree_msg.p2);
	free(plan_tree_msg.mask);
}

void
Publisher_Util::publish_principal_path_message(list<RRT_Path_Edge> &path)
{
	static carmen_navigator_ackerman_plan_tree_message plan_tree_msg;
	IPC_RETURN_TYPE err = IPC_OK;
	static bool		first_time = true;

	if (first_time)
	{
		define_plan_tree_message();
		plan_tree_msg.host = carmen_get_host();
		first_time = false;
	}

	plan_tree_msg.timestamp = carmen_get_time();
	plan_tree_msg.num_edges = 0;

	plan_tree_msg.p1 = (carmen_ackerman_traj_point_t *) malloc(sizeof(carmen_ackerman_traj_point_t) * plan_tree_msg.num_edges);
	plan_tree_msg.p2 = (carmen_ackerman_traj_point_t *) malloc(sizeof(carmen_ackerman_traj_point_t) * plan_tree_msg.num_edges);
	plan_tree_msg.mask = (int *) malloc(sizeof(int) * plan_tree_msg.num_edges);

	carmen_navigator_ackerman_plan_message msg;

	plan_tree_msg.num_path = 1;

	msg = get_path(path);

	if (msg.path_length > 100)
	{	// Ver tipo carmen_navigator_ackerman_plan_tree_message
		printf("Error: msg.path_length > 100 in Publisher_Util::publish_plan_tree_message()\n");
		exit(1);
	}

	memcpy(plan_tree_msg.paths[0], msg.path, sizeof(carmen_ackerman_traj_point_t) * msg.path_length);
	free(msg.path);
	plan_tree_msg.path_size[0] = msg.path_length;

	err = IPC_publishData(CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME, &plan_tree_msg);

	carmen_test_ipc(err, "Could not publish", CARMEN_NAVIGATOR_ACKERMAN_PLAN_TREE_NAME);

	free(plan_tree_msg.p1);
	free(plan_tree_msg.p2);
	free(plan_tree_msg.mask);
}


void
Publisher_Util::publish_navigator_ackerman_plan_message(carmen_navigator_ackerman_plan_message msg)
{
	static bool		first_time = true;
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
	static bool		first_time = true;
	IPC_RETURN_TYPE err;
	list<RRT_Edge>::iterator it;

	if (first_time)
	{
		carmen_rrt_planner_define_plan_message();
	}

	err = IPC_publishData(CARMEN_RRT_PLANNER_PLAN_NAME, &msg);
	carmen_test_ipc(err, "Could not publish",
			CARMEN_RRT_PLANNER_PLAN_NAME);
}


carmen_navigator_ackerman_plan_message
Publisher_Util::get_path(list<RRT_Path_Edge> &path)
{
	carmen_navigator_ackerman_plan_message msg;
	list<RRT_Path_Edge>::iterator it;
	list<RRT_Path_Edge>::reverse_iterator rit;
	int i;

	msg.host = carmen_get_host();
	msg.timestamp = GlobalState::rrt_planner_timestamp;

	if (path.empty())
	{
		msg.path = NULL;
		msg.path_length = 0;
		return msg;
	}

	msg.path_length = path.size() + 1;
	msg.path = (carmen_ackerman_traj_point_t *) malloc(sizeof(carmen_ackerman_traj_point_t) * (msg.path_length));

	it = path.begin();

	for (i = 0; it != path.end(); it++)
	{
		msg.path[i].x	  = it->p1.pose.x;
		msg.path[i].y	  = it->p1.pose.y;
		msg.path[i].theta = it->p1.pose.theta;
		msg.path[i].v	  = it->command.v;
		msg.path[i].phi	  = it->command.phi;
		i++;
	}

	rit = path.rbegin();

	msg.path[i].x	  = rit->p2.pose.x;
	msg.path[i].y	  = rit->p2.pose.y;
	msg.path[i].theta = rit->p2.pose.theta;
	msg.path[i].v	  = rit->command.v;
	msg.path[i].phi	  = rit->command.phi;

	return msg;
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
