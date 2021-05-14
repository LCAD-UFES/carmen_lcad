/*
 * motion_planner_main.c
 *
 *  Created on: 20/09/2012
 *      Author: romulo
 */

#include <carmen/carmen.h>
#include "motion_planner.h"
#include "motion_planner_interface.h"
#include <carmen/behavior_selector_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/obstacle_avoider_interface.h>
#include <carmen/mapper_interface.h>


extern int autonomous_status;
extern carmen_motion_planner_path_message path;
extern carmen_robot_and_trailer_traj_point_t g_robot_position;

extern int current_map;
extern carmen_map_p map_vector[NUM_MAPS];
extern int necessary_maps_available;

extern carmen_behavior_selector_algorithm_t current_algorithm;


/*********************************************************
		   --- Publishers ---
 **********************************************************/

void
publish_obstacle_avoider_path(carmen_robot_and_trailer_motion_command_t *motion_commands_vector, int num_motion_commands)
{
	carmen_navigator_ackerman_plan_message msg;
	
	msg = build_navigator_ackerman_plan_message(motion_commands_vector, num_motion_commands);
	carmen_obstacle_avoider_publish_path(msg);

	free(msg.path);
}
 

void
publish_motion_planner_path(carmen_robot_and_trailer_motion_command_t *motion_commands_vector, int num_motion_commands)
{
	carmen_navigator_ackerman_plan_message msg;
	
	msg = build_navigator_ackerman_plan_message(motion_commands_vector, num_motion_commands);
	carmen_obstacle_avoider_publish_motion_planner_path(msg);
	
	free(msg.path);
}


void
publish_status(void)
{
	carmen_navigator_ackerman_status_message status_msg;
	IPC_RETURN_TYPE err;

	status_msg.timestamp = carmen_get_time();
	status_msg.host = carmen_get_host();

	status_msg.autonomous = autonomous_status;
	status_msg.goal_set = path.path_size > 0;
	if ((status_msg.goal_set) && (path.path != NULL))
	{
		status_msg.goal.x = path.path[path.path_size-1].x;
		status_msg.goal.y = path.path[path.path_size-1].y;
		status_msg.goal.theta = path.path[path.path_size-1].theta;
	}
	status_msg.robot = g_robot_position;

	err = IPC_publishData(CARMEN_NAVIGATOR_ACKERMAN_STATUS_NAME, &status_msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_NAVIGATOR_ACKERMAN_STATUS_NAME);
}


void
publish_plan(void)
{
	carmen_navigator_ackerman_plan_message plan_msg;
	IPC_RETURN_TYPE err;

	if (!path.path)
		return;

	plan_msg.timestamp = carmen_get_time();
	plan_msg.host = carmen_get_host();

	plan_msg.path_length = path.path_size;
	plan_msg.path = path.path;

	err = IPC_publishData(CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME, &plan_msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_NAVIGATOR_ACKERMAN_PLAN_NAME);
}



/*********************************************************
		   --- Handlers ---
 **********************************************************/


static void
signal_handler(int signo __attribute__ ((unused)) )
{
	carmen_ipc_disconnect();
	exit(0);
}


static void
current_algorithm_handler(carmen_behavior_selector_state_message *msg)
{
	motion_planner_set_algorithm(msg->algorithm, msg->task);
}


static void
path_handler(carmen_motion_planner_path_message *msg)
{
	if (current_algorithm != CARMEN_BEHAVIOR_SELECTOR_RRT)
		motion_planner_set_path(msg);
}


static void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message * msg)
{
	if (current_algorithm != CARMEN_BEHAVIOR_SELECTOR_RRT)
		motion_planner_set_robot_pose(msg->globalpos, msg->v, msg->phi);
}


static void
map_handler(carmen_mapper_map_message *message)
{
	if (current_map < NUM_MAPS - 1)
		current_map++;
	else
		current_map = 0;

	copy_grid_mapping_to_map_vector(message, current_map);

	necessary_maps_available = 1;
}


static void
go_handler()
{
	motion_planner_go();
}


static void
stop_handler()
{
	motion_planner_stop();
}




/*********************************************************
		   --- Inicialization ---
 **********************************************************/


static void
define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, IPC_VARIABLE_LENGTH, CARMEN_BASE_ACKERMAN_MOTION_COMMAND_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME);

	carmen_motion_planner_define_path_message();
}


static void
register_handlers()
{
	carmen_motion_planner_subscribe_path_message(NULL, (carmen_handler_t) path_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_behavior_selector_subscribe_current_state_message(NULL, (carmen_handler_t) current_algorithm_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(CARMEN_NAVIGATOR_ACKERMAN_GO_NAME, CARMEN_DEFAULT_MESSAGE_FMT,	NULL, sizeof(carmen_default_message), go_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_subscribe_message(CARMEN_NAVIGATOR_ACKERMAN_STOP_NAME, CARMEN_DEFAULT_MESSAGE_FMT, NULL, sizeof(carmen_default_message), stop_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_mapper_subscribe_map_message(NULL, (carmen_handler_t) map_handler, CARMEN_SUBSCRIBE_LATEST);
}


static void 
initialize_map_vector(int number_of_maps)
{
	int i;

	for (i = 0; i < number_of_maps; i++)
	{
		map_vector[i] = NULL;
	}
}


static void
motion_planner_initialize()
{
	path.path_size = 0;
	path.path = NULL;

	initialize_map_vector(NUM_MAPS);
}


int
main(int argc, char **argv)
{
	signal(SIGINT, signal_handler);

	carmen_ipc_initialize(argc, argv);

	motion_planner_read_parameters(argc, argv);

	motion_planner_initialize();

	define_messages();

	register_handlers();

	carmen_ipc_dispatch();

	return 0;
}
