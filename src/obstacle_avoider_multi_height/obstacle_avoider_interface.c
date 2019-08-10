#include "obstacle_avoider_interface.h"
#include "obstacle_avoider_messages.h"


void 
carmen_obstacle_avoider_publish_base_ackerman_motion_command(carmen_ackerman_motion_command_p motion_commands,
		int num_motion_commands, double timestamp)
{
	IPC_RETURN_TYPE err;
	carmen_base_ackerman_motion_command_message motion_command_message;

	motion_command_message.host = carmen_get_host();

	motion_command_message.num_motion_commands = num_motion_commands;
	motion_command_message.motion_command = motion_commands;

	motion_command_message.timestamp = timestamp;

	err = IPC_publishData(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, &motion_command_message);
	carmen_test_ipc(err, "Could not publish", CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME);
}


void
carmen_obstacle_avoider_publish_motion_planner_path(carmen_navigator_ackerman_plan_message msg)
{
	static int first_time = TRUE;
	IPC_RETURN_TYPE err;

	if (first_time)
	{
		err = IPC_defineMsg(CARMEN_OBSTACLE_AVOIDER_MOTION_PLANNER_PATH_NAME, IPC_VARIABLE_LENGTH,
				CARMEN_OBSTACLE_AVOIDER_MOTION_PLANNER_PATH_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_OBSTACLE_AVOIDER_MOTION_PLANNER_PATH_NAME);

		first_time = FALSE;
	}

	err = IPC_publishData(CARMEN_OBSTACLE_AVOIDER_MOTION_PLANNER_PATH_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_OBSTACLE_AVOIDER_MOTION_PLANNER_PATH_NAME);
}


void
carmen_obstacle_avoider_publish_path(carmen_navigator_ackerman_plan_message msg)
{
	static int first_time = TRUE;
	IPC_RETURN_TYPE err;

	if (first_time)
	{
		err = IPC_defineMsg(CARMEN_OBSTACLE_AVOIDER_PATH_NAME, IPC_VARIABLE_LENGTH,
				CARMEN_OBSTACLE_AVOIDER_PATH_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_OBSTACLE_AVOIDER_PATH_NAME);

		first_time = FALSE;
	}

	err = IPC_publishData(CARMEN_OBSTACLE_AVOIDER_PATH_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_OBSTACLE_AVOIDER_PATH_NAME);
}


void
carmen_obstacle_avoider_publish_robot_hit_obstacle_message(int robot_will_hit_obstacle)
{
	static int first_time = TRUE;
	IPC_RETURN_TYPE err;
	carmen_obstacle_avoider_robot_will_hit_obstacle_message msg;

	if (first_time)
	{
		err = IPC_defineMsg(CARMEN_OBSTACLE_AVOIDER_ROBOT_HIT_OBSTACLE_NAME, IPC_VARIABLE_LENGTH,
				CARMEN_OBSTACLE_AVOIDER_ROBOT_HIT_OBSTACLE_FMT);
		carmen_test_ipc_exit(err, "Could not define message", CARMEN_OBSTACLE_AVOIDER_ROBOT_HIT_OBSTACLE_NAME);

		first_time = FALSE;
	}

	msg.host = carmen_get_host();
	msg.robot_will_hit_obstacle = robot_will_hit_obstacle;
	msg.timestamp = carmen_get_time();

	err = IPC_publishData(CARMEN_OBSTACLE_AVOIDER_ROBOT_HIT_OBSTACLE_NAME, &msg);
	carmen_test_ipc(err, "Could not publish", CARMEN_OBSTACLE_AVOIDER_ROBOT_HIT_OBSTACLE_NAME);
}


void
carmen_obstacle_avoider_subscribe_robot_hit_obstacle_message(carmen_obstacle_avoider_robot_will_hit_obstacle_message *msg,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_OBSTACLE_AVOIDER_ROBOT_HIT_OBSTACLE_NAME,
			CARMEN_OBSTACLE_AVOIDER_ROBOT_HIT_OBSTACLE_FMT,
			msg, sizeof(carmen_obstacle_avoider_robot_will_hit_obstacle_message),
			handler, subscribe_how);
}


void
carmen_obstacle_avoider_subscribe_motion_planner_path_message(carmen_navigator_ackerman_plan_message *msg, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_OBSTACLE_AVOIDER_MOTION_PLANNER_PATH_NAME,
			CARMEN_OBSTACLE_AVOIDER_MOTION_PLANNER_PATH_FMT,
			msg, sizeof(carmen_navigator_ackerman_plan_message),
			handler, subscribe_how);
}


void
carmen_obstacle_avoider_subscribe_path_message(carmen_navigator_ackerman_plan_message *msg,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_OBSTACLE_AVOIDER_PATH_NAME,
			CARMEN_OBSTACLE_AVOIDER_PATH_FMT,
			msg, sizeof(carmen_navigator_ackerman_plan_message),
			handler, subscribe_how);
}


void
carmen_robot_ackerman_subscribe_road_velocity_control_message(carmen_robot_ackerman_road_velocity_control_message *msg,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_ROBOT_ACKERMAN_ROAD_VELOCITY_CONTROL_NAME,
			CARMEN_ROBOT_ACKERMAN_ROAD_VELOCITY_CONTROL_FMT,
			msg, sizeof(carmen_robot_ackerman_road_velocity_control_message),
			handler, subscribe_how);
}
