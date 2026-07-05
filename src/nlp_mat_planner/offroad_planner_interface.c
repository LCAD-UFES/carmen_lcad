/*********************************************************
 *
 ********************************************************/

#include "offroad_planner_interface.h"

void carmen_offroad_planner_subscribe_plan_message(carmen_offroad_planner_plan_message *plan,
												   carmen_handler_t handler,
												   carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_OFFROAD_PLANNER_PLAN_MESSAGE_NAME,
							 CARMEN_OFFROAD_PLANNER_PLAN_MESSAGE_FMT,
							 plan, sizeof(carmen_offroad_planner_plan_message),
							 handler, subscribe_how);
}

void carmen_offroad_planner_unsubscribe_plan_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_OFFROAD_PLANNER_PLAN_MESSAGE_NAME, handler);
}

void carmen_offroad_planner_publish_plan(carmen_offroad_planner_plan_message *plan)
{
	IPC_RETURN_TYPE err = IPC_OK;

	static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_OFFROAD_PLANNER_PLAN_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_OFFROAD_PLANNER_PLAN_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_OFFROAD_PLANNER_PLAN_MESSAGE_NAME);
		initialized = 1;
	}

	plan->timestamp = carmen_get_time();
	plan->host = carmen_get_host();

	err = IPC_publishData(CARMEN_OFFROAD_PLANNER_PLAN_MESSAGE_NAME, plan);
	carmen_test_ipc(err, "Could not publish", CARMEN_OFFROAD_PLANNER_PLAN_MESSAGE_NAME);
}
