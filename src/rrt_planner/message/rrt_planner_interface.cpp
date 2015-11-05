/*
 * rrt_planner_interface.cpp
 *
 *  Created on: 16/12/2011
 *      Author: rradaelli
 */
#include "rrt_planner_interface.h"

void carmen_rrt_planner_define_robot_tree_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_RRT_PLANNER_ROBOT_TREE_NAME, IPC_VARIABLE_LENGTH,
						CARMEN_RRT_PLANNER_ROBOT_TREE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_RRT_PLANNER_ROBOT_TREE_NAME);
}

void carmen_rrt_planner_define_goal_tree_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_RRT_PLANNER_GOAL_TREE_NAME, IPC_VARIABLE_LENGTH,
						CARMEN_RRT_PLANNER_GOAL_TREE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_RRT_PLANNER_GOAL_TREE_NAME);
}

void carmen_rrt_planner_subscribe_robot_tree_message(
	carmen_rrt_planner_tree_message *message,
	carmen_handler_t				 handler,
	carmen_subscribe_t				 subscribe_how)
{
	carmen_subscribe_message((char *)CARMEN_RRT_PLANNER_ROBOT_TREE_NAME, (char *)CARMEN_RRT_PLANNER_ROBOT_TREE_FMT,
							 message, sizeof(carmen_rrt_planner_tree_message),
							 handler, subscribe_how);
}

void carmen_rrt_planner_subscribe_goal_tree_message(
	carmen_rrt_planner_tree_message *message,
	carmen_handler_t				 handler,
	carmen_subscribe_t				 subscribe_how)
{
	carmen_subscribe_message((char *)CARMEN_RRT_PLANNER_GOAL_TREE_NAME, (char *)CARMEN_RRT_PLANNER_GOAL_TREE_FMT,
							 message, sizeof(carmen_rrt_planner_tree_message),
							 handler, subscribe_how);
}

void carmen_rrt_planner_set_goal(carmen_point_t goal)
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_rrt_planner_set_goal_message msg;
	static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_RRT_PLANNER_SET_GOAL_NAME,
							IPC_VARIABLE_LENGTH,
							CARMEN_RRT_PLANNER_SET_GOAL_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
							 CARMEN_RRT_PLANNER_SET_GOAL_NAME);
		initialized = 1;
	}

	msg.x		  = goal.x;
	msg.y		  = goal.y;
	msg.theta	  = goal.theta;
	msg.timestamp = carmen_get_time();
	msg.host	  = carmen_get_host();

	err = IPC_publishData(CARMEN_RRT_PLANNER_SET_GOAL_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_RRT_PLANNER_SET_GOAL_NAME);
}

void carmen_rrt_planner_subscribe_set_goal_message(
	carmen_rrt_planner_set_goal_message *message,
	carmen_handler_t					 handler,
	carmen_subscribe_t					 subscribe_how)
{
	carmen_subscribe_message(
		(char *)CARMEN_RRT_PLANNER_SET_GOAL_NAME,
		(char *)CARMEN_RRT_PLANNER_SET_GOAL_FMT,
		message,
		sizeof(carmen_rrt_planner_set_goal_message),
		handler,
		subscribe_how);
}

void carmen_rrt_planner_go()
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_rrt_planner_go_message msg;
	static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_RRT_PLANNER_GO,
							IPC_VARIABLE_LENGTH,
							CARMEN_DEFAULT_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
							 CARMEN_RRT_PLANNER_GO);
		initialized = 1;
	}

	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_RRT_PLANNER_GO, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_RRT_PLANNER_GO);
}

void carmen_rrt_planner_stop()
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_rrt_planner_stop_message msg;
	static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_RRT_PLANNER_STOP,
							IPC_VARIABLE_LENGTH,
							CARMEN_DEFAULT_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
							 CARMEN_RRT_PLANNER_STOP);
		initialized = 1;
	}

	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_RRT_PLANNER_STOP, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_RRT_PLANNER_STOP);
}

void carmen_rrt_planner_define_status_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_RRT_PLANNER_STATUS_NAME, IPC_VARIABLE_LENGTH,
						CARMEN_RRT_PLANNER_STATUS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_RRT_PLANNER_STATUS_NAME);
}

void carmen_rrt_planner_subscribe_status_message(
	carmen_rrt_planner_status_message *message,
	carmen_handler_t				   handler,
	carmen_subscribe_t				   subscribe_how)
{
	carmen_subscribe_message((char *)CARMEN_RRT_PLANNER_STATUS_NAME, (char *)CARMEN_RRT_PLANNER_STATUS_FMT,
							 message, sizeof(carmen_rrt_planner_status_message),
							 handler, subscribe_how);
}

void carmen_rrt_planner_subscribe_plan_message(
	carmen_rrt_planner_plan_message *message,
	carmen_handler_t				 handler,
	carmen_subscribe_t				 subscribe_how)
{
	carmen_subscribe_message((char *)CARMEN_RRT_PLANNER_PLAN_NAME, (char *)CARMEN_RRT_PLANNER_PLAN_FMT,
							 message, sizeof(carmen_rrt_planner_plan_message),
							 handler, subscribe_how);
}

void carmen_rrt_planner_define_plan_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_RRT_PLANNER_PLAN_NAME, IPC_VARIABLE_LENGTH,
						CARMEN_RRT_PLANNER_PLAN_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_RRT_PLANNER_PLAN_NAME);
}

void carmen_rrt_planner_subscribe_go_message(
	carmen_handler_t   handler,
	carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(
		(char *)CARMEN_RRT_PLANNER_GO,
		(char *)CARMEN_DEFAULT_MESSAGE_FMT,
		NULL, sizeof(carmen_rrt_planner_go_message),
		handler, subscribe_how);
}

void carmen_rrt_planner_subscribe_stop_message(
	carmen_handler_t   handler,
	carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char *)CARMEN_RRT_PLANNER_STOP,
							 (char *)CARMEN_DEFAULT_MESSAGE_FMT,
							 NULL, sizeof(carmen_rrt_planner_stop_message),
							 handler, subscribe_how);
}

