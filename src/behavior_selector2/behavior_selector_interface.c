/*
 * behavior_selector_interface.c
 *
 *  Created on: 28/09/2012
 *      Author: romulo
 */
#include "behavior_selector_interface.h"


void carmen_behavior_selector_subscribe_current_state_message(
		carmen_behavior_selector_state_message *msg,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_NAME,
			CARMEN_BEHAVIOR_SELECTOR_CURRENT_STATE_FMT,
			msg, sizeof(carmen_behavior_selector_state_message),
			handler, subscribe_how);
}

void
carmen_behavior_selector_subscribe_goal_list_message(carmen_behavior_selector_goal_list_message *msg,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_NAME,
			CARMEN_BEHAVIOR_SELECTOR_GOAL_LIST_FMT,
			msg, sizeof(carmen_behavior_selector_goal_list_message),
			handler, subscribe_how);
}

void
carmen_behavior_selector_set_goal_source(carmen_behavior_selector_goal_source_t goal_source)
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_behavior_selector_set_goal_source_message msg;

	static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_SET_GOAL_SOURCE_NAME,
				IPC_VARIABLE_LENGTH,
				CARMEN_BEHAVIOR_SELECTOR_SET_GOAL_SOURCE_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_BEHAVIOR_SELECTOR_SET_GOAL_SOURCE_NAME);
		initialized = 1;
	}

	msg.goal_source = goal_source;
	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_SET_GOAL_SOURCE_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_SET_GOAL_SOURCE_NAME);

}

void carmen_behavior_selector_set_state(carmen_behavior_selector_state_t state)
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_behavior_selector_set_state_message msg;

	static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_SET_STATE_NAME,
				IPC_VARIABLE_LENGTH,
				CARMEN_BEHAVIOR_SELECTOR_SET_STATE_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_BEHAVIOR_SELECTOR_SET_STATE_NAME);
		initialized = 1;
	}

	msg.state = state;
	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_SET_STATE_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_SET_STATE_NAME);
}

void carmen_behavior_selector_add_goal(carmen_point_t goal)
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_behavior_selector_add_goal_message msg;

	static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_ADD_GOAL_NAME,
				IPC_VARIABLE_LENGTH,
				CARMEN_BEHAVIOR_SELECTOR_ADD_GOAL_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_BEHAVIOR_SELECTOR_SET_STATE_NAME);
		initialized = 1;
	}

	msg.goal = goal;
	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_ADD_GOAL_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_ADD_GOAL_NAME);
}

void carmen_behavior_selector_clear_goal_list()
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_behavior_selector_clear_goal_list_message msg;

	static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_CLEAR_GOAL_LIST_NAME,
				IPC_VARIABLE_LENGTH,
				CARMEN_BEHAVIOR_SELECTOR_CLEAR_GOAL_LIST_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_BEHAVIOR_SELECTOR_SET_STATE_NAME);
		initialized = 1;
	}

	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_CLEAR_GOAL_LIST_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_CLEAR_GOAL_LIST_NAME);
}

void carmen_behavior_selector_remove_goal()
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_behavior_selector_remove_goal_message msg;

	static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_REMOVE_GOAL_NAME,
				IPC_VARIABLE_LENGTH,
				CARMEN_BEHAVIOR_SELECTOR_REMOVE_GOAL_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_BEHAVIOR_SELECTOR_SET_STATE_NAME);
		initialized = 1;
	}

	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_REMOVE_GOAL_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_REMOVE_GOAL_NAME);
}

void
carmen_behavior_selector_set_algorithm(carmen_behavior_selector_algorithm_t algorithm,  carmen_behavior_selector_state_t state)
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_behavior_selector_set_algorithm_message msg;

	static int initialized = 0;

	if (!initialized)
	{
		err = IPC_defineMsg(CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_NAME,
				IPC_VARIABLE_LENGTH,
				CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_FMT);
		carmen_test_ipc_exit(err, "Could not define message",
				CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_NAME);
		initialized = 1;
	}

	msg.algorithm = algorithm;
	msg.state = state;
	msg.timestamp = carmen_get_time();
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_BEHAVIOR_SELECTOR_SET_ALGOTITHM_NAME);
}


char *
get_low_level_state_name(carmen_behavior_selector_low_level_state_t state)
{
	if (state == Initializing) 		return ((char *) "Initializing");
	if (state == Stopped) 			return ((char *) "Stopped");
	if (state == Free_Running) 		return ((char *) "Free_Running");
	if (state == Following_Moving_Object) 			return ((char *) "Following_Moving_Object");
	if (state == Stopping_Behind_Moving_Object) 	return ((char *) "Stopping_Behind_Moving_Object");
	if (state == Stopped_Behind_Moving_Object_S0) 	return ((char *) "Stopped_Behind_Moving_Object_S0");
	if (state == Stopped_Behind_Moving_Object_S1) 	return ((char *) "Stopped_Behind_Moving_Object_S1");
	if (state == Stopped_Behind_Moving_Object_S2) 	return ((char *) "Stopped_Behind_Moving_Object_S2");
	if (state == Stopping_At_Red_Traffic_Light)     return ((char *) "Stopping_At_Red_Traffic_Light");
	if (state == Stopped_At_Red_Traffic_Light_S0) 	return ((char *) "Stopped_At_Red_Traffic_Light_S0");
	if (state == Stopped_At_Red_Traffic_Light_S1) 	return ((char *) "Stopped_At_Red_Traffic_Light_S1");
	if (state == Stopped_At_Red_Traffic_Light_S2) 	return ((char *) "Stopped_At_Red_Traffic_Light_S2");
	if (state == Stopping_At_Busy_Pedestrian_Track)     return ((char *) "Stopping_At_Busy_Pedestrian_Track");
	if (state == Stopped_At_Busy_Pedestrian_Track_S0) 	return ((char *) "Stopped_At_Busy_Pedestrian_Track_S0");
	if (state == Stopped_At_Busy_Pedestrian_Track_S1) 	return ((char *) "Stopped_At_Busy_Pedestrian_Track_S1");
	if (state == Stopped_At_Busy_Pedestrian_Track_S2) 	return ((char *) "Stopped_At_Busy_Pedestrian_Track_S2");
	if (state == Stopping_At_Stop_Sign) 			return ((char *) "Stopping_At_Stop_Sign");
	if (state == Stopped_At_Stop_Sign_S0) 			return ((char *) "Stopped_At_Stop_Sign_S0");
	if (state == Stopped_At_Stop_Sign_S1) 			return ((char *) "Stopped_At_Stop_Sign_S1");

	return ((char *) " ");
}
