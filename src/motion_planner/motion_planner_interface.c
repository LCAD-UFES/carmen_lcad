/*
 * motion_planner_interface.c
 *
 *  Created on: 20/09/2012
 *      Author: romulo
 */

#include "motion_planner_interface.h"

void
carmen_motion_planner_subscribe_path_message(
		carmen_motion_planner_path_message *path_msg,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_MOTION_PLANNER_PATH_NAME,
			CARMEN_MOTION_PLANNER_PATH_FMT,
			path_msg, sizeof(carmen_motion_planner_path_message),
			handler, subscribe_how);

}


void carmen_motion_planner_publish_path_message(
		carmen_robot_and_trailer_traj_point_t *path, int size, int algorithm)
{
	IPC_RETURN_TYPE err;
	static int firsttime = 1;
	static carmen_motion_planner_path_message msg;

	if (firsttime)
	{
		carmen_motion_planner_define_path_message();
		msg.host = carmen_get_host();
		firsttime = 0;
	}

	msg.timestamp = carmen_get_time();
	msg.path = path;
	msg.path_size = size;
	msg.algorithm = algorithm;

	err = IPC_publishData(CARMEN_MOTION_PLANNER_PATH_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_MOTION_PLANNER_PATH_NAME);
}


void carmen_motion_planner_define_path_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_MOTION_PLANNER_PATH_NAME,
			IPC_VARIABLE_LENGTH,
			CARMEN_MOTION_PLANNER_PATH_FMT);
	carmen_test_ipc_exit(err, "Could not define message",
			CARMEN_MOTION_PLANNER_PATH_NAME);
}




