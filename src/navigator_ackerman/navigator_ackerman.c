/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <carmen/carmen.h>
#include "navigator_ackerman.h"
#include "navigator_ackerman_ipc.h"
#include "planner_ackerman_interface.h"
#include <carmen/mapper_interface.h>
#include <prob_measurement_model.h>
#include <prob_map.h>
#include <carmen/rddf_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/motion_planner_interface.h>
#include <carmen/collision_detection.h>
#include <carmen/map_server_interface.h>
#include "navigator_ackerman_interface.h"

typedef void(*handler)(int);


static carmen_map_p nav_map = NULL;
static carmen_map_placelist_t placelist;
static carmen_robot_ackerman_config_t robot_config;
static carmen_navigator_config_t nav_config;
carmen_navigator_ackerman_astar_t astar_config;

static int cheat = 0;
static int autonomous_status = 0;
int steering_model = 0;

static carmen_ackerman_traj_point_t robot_position;

static carmen_base_ackerman_odometry_message odometry;
static carmen_base_ackerman_odometry_message corrected_odometry;
//static carmen_robot_ackerman_laser_message frontlaser, rearlaser;
static carmen_localize_ackerman_globalpos_message globalpos;
static int robot_initialized = 0;

carmen_behavior_selector_algorithm_t current_algorithm = CARMEN_BEHAVIOR_SELECTOR_A_STAR;
carmen_behavior_selector_mission_t current_mission = BEHAVIOR_SELECTOR_PARK;
carmen_behavior_selector_goal_source_t goal_source = -1;


static void
publish_motion_planner_path_message()
{
	carmen_planner_status_t status;
	carmen_planner_ackerman_get_status(&status);
	if (status.path.points)
	{
		carmen_motion_planner_publish_path_message(status.path.points, status.path.length, current_algorithm);
		free(status.path.points);
	}
}


static void
publish_navigator_ackerman_messages()
{
	if (current_algorithm == CARMEN_BEHAVIOR_SELECTOR_GRADIENT)
		publish_motion_planner_path_message();

	if (current_algorithm == CARMEN_BEHAVIOR_SELECTOR_A_STAR)
		publish_motion_planner_path_message();
}


static void
goal_list_handler(carmen_behavior_selector_goal_list_message *msg)
{
	if (current_algorithm != CARMEN_BEHAVIOR_SELECTOR_A_STAR && current_algorithm != CARMEN_BEHAVIOR_SELECTOR_GRADIENT)
		return;

	carmen_planner_ackerman_set_goal_list(msg->goal_list, msg->size, &nav_config);
}

static void
state_handler(carmen_behavior_selector_state_message *msg)
{
	current_algorithm = msg->algorithm;
	current_mission = msg->mission;

	if (current_algorithm != CARMEN_BEHAVIOR_SELECTOR_A_STAR && current_algorithm != CARMEN_BEHAVIOR_SELECTOR_GRADIENT)
		return;

	if (goal_source != msg->goal_source)
	{
		goal_source = msg->goal_source;
		carmen_ackerman_traj_point_t point;
		point.x = -1;
		point.y = -1;
		point.theta = -1;
		carmen_planner_ackerman_update_goal(&point, 1, &nav_config);
	}

	switch(current_algorithm)
	{
	case CARMEN_BEHAVIOR_SELECTOR_GRADIENT:
		steering_model = 0;
		carmen_planner_ackerman_regenerate_trajectory(&nav_config);
		break;

	case CARMEN_BEHAVIOR_SELECTOR_A_STAR:
		steering_model = 1;
		carmen_planner_ackerman_regenerate_trajectory(&nav_config);
		break;

	default:
		break;
	}

}


static void
update_positions(void)
{
	static carmen_point_t last_position = {0, 0, 0};
	static double last_timestamp = 0.0;


	if (current_algorithm != CARMEN_BEHAVIOR_SELECTOR_A_STAR && current_algorithm != CARMEN_BEHAVIOR_SELECTOR_GRADIENT)
		return;


	robot_initialized = 1;
	corrected_odometry = odometry;

	robot_position.x = globalpos.globalpos.x;
	robot_position.y = globalpos.globalpos.y;
	robot_position.theta = globalpos.globalpos.theta;
	robot_position.v = globalpos.v;
	robot_position.phi = globalpos.phi;

	carmen_planner_ackerman_update_robot(&robot_position, &nav_config);

	/* If the distance between the new position and the old position is further
     	   than twice the maximum distance we could have travelled, travelling at
     	   top speed, then obviously this is a localize reset, and we should reset
     	   the map. */

	if (carmen_distance(&last_position, &(globalpos.globalpos)) > 2 * robot_config.max_v / (globalpos.timestamp - last_timestamp))
	{
		carmen_planner_ackerman_reset_map(&robot_config);
	}

	last_timestamp = globalpos.timestamp;
	last_position = globalpos.globalpos;

	carmen_planner_ackerman_change_goal(&nav_config);
}


static void
simulator_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void *clientData __attribute__ ((unused)))
{
	IPC_RETURN_TYPE err = IPC_OK;
	FORMATTER_PTR formatter;
	carmen_simulator_ackerman_truepos_message msg;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &msg,
			sizeof(carmen_simulator_ackerman_truepos_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall",
			IPC_msgInstanceName(msgRef));

	if (!cheat)
		return;

	globalpos.globalpos = msg.truepose;
	globalpos.odometrypos = msg.odometrypose;

	update_positions();
}


static void
localize_globalpos_handler(void)
{
	update_positions();
	publish_navigator_ackerman_messages();
}


void
carmen_navigator_ackerman_start_autonomous(void)
{
	autonomous_status = 1;
	carmen_planner_ackerman_reset_map(&robot_config);
	//	generate_next_motion_command();
}


int
carmen_navigator_ackerman_autonomous_status(void)
{
	return autonomous_status;
}


carmen_map_placelist_p
carmen_navigator_ackerman_get_places(void)
{
	return &placelist;
}


void
carmen_navigator_ackerman_set_max_velocity(double vel)
{
	robot_config.max_v = vel;
}


void
carmen_navigator_ackerman_goal(double x, double y, double theta)
{
	carmen_ackerman_traj_point_t point;

	point.x = x;
	point.y = y;
	point.theta = theta;

	carmen_planner_ackerman_update_goal(&point, 1, &nav_config);
}


void
carmen_navigator_ackerman_goal_triplet(carmen_ackerman_traj_point_p point)
{
	carmen_planner_ackerman_update_goal(point, 0, &nav_config);
}


int
carmen_navigator_ackerman_goal_place(char *name)
{
	int index;
	carmen_ackerman_traj_point_t goal;

	for (index = 0; index < placelist.num_places; index++)
	{
		if (strcmp(name, placelist.places[index].name) == 0)
			break;
	}

	if (index == placelist.num_places)
		return -1;

	goal.x = placelist.places[index].x;
	goal.y = placelist.places[index].y;

	if (placelist.places[index].type == CARMEN_NAMED_POSITION_TYPE)
		carmen_planner_ackerman_update_goal(&goal, 1, &nav_config);
	else
	{
		goal.theta = placelist.places[index].theta;
		carmen_planner_ackerman_update_goal(&goal, 0, &nav_config);
	}

	return 0;
}


void
mapper_map_handler(carmen_mapper_map_message *online_map_message)
{
	carmen_map_t *new_map;

	if (current_algorithm != CARMEN_BEHAVIOR_SELECTOR_A_STAR && current_algorithm != CARMEN_BEHAVIOR_SELECTOR_GRADIENT)
		return;

	new_map = (carmen_map_t *) calloc(1, sizeof(carmen_map_t));
	carmen_mapper_copy_map_from_message(new_map, online_map_message);

	if (nav_map != NULL)
		carmen_map_destroy(&nav_map);
	nav_map = new_map;

	carmen_planner_ackerman_set_map(nav_map, &robot_config);
}


static void
navigator_shutdown(int signo __attribute__ ((unused)) )
{
	static int done = 0;

	if(!done) {
		carmen_ipc_disconnect();
		printf("Disconnected from IPC.\n");

		done = 1;
	}
	exit(0);
}


static void
read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] = 
	{
			{"robot", "max_velocity", CARMEN_PARAM_DOUBLE, &robot_config.max_v, 1, NULL},//todo add max_v and max_phi in carmen.ini
			{"robot", "max_steering_angle", CARMEN_PARAM_DOUBLE, &robot_config.max_phi, 1, NULL},
			{"robot", "min_approach_dist", CARMEN_PARAM_DOUBLE, &robot_config.approach_dist, 1, NULL},
			{"robot", "min_side_dist", CARMEN_PARAM_DOUBLE, &robot_config.side_dist, 1, NULL},
			{"robot", "length", CARMEN_PARAM_DOUBLE, &robot_config.length, 0, NULL},
			{"robot", "width", CARMEN_PARAM_DOUBLE, &robot_config.width, 0, NULL},
			{"robot", "maximum_acceleration_forward", CARMEN_PARAM_DOUBLE, &robot_config.maximum_acceleration_forward, 1, NULL},
			{"robot", "reaction_time", CARMEN_PARAM_DOUBLE,	&robot_config.reaction_time, 0, NULL},
			{"robot", "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_front_and_rear_axles, 1, NULL},
			{"robot", "maximum_steering_command_rate", CARMEN_PARAM_DOUBLE, &robot_config.maximum_steering_command_rate, 1, NULL},
			{"robot", "distance_between_rear_car_and_rear_wheels", CARMEN_PARAM_DOUBLE, &robot_config.distance_between_rear_car_and_rear_wheels, 1, NULL},
			{"navigator", "goal_size", CARMEN_PARAM_DOUBLE, &nav_config.goal_size, 1, NULL},
			{"navigator", "waypoint_tolerance", CARMEN_PARAM_DOUBLE, &nav_config.waypoint_tolerance, 1, NULL},
			{"navigator", "goal_theta_tolerance", CARMEN_PARAM_DOUBLE, &nav_config.goal_theta_tolerance, 1, NULL},
			{"navigator", "map_update_radius", CARMEN_PARAM_DOUBLE,	&nav_config.map_update_radius, 1, NULL},
			{"navigator", "map_update_num_laser_beams", CARMEN_PARAM_INT, &nav_config.num_lasers_to_use, 1, NULL},
			{"navigator", "map_update_obstacles", CARMEN_PARAM_ONOFF, &nav_config.map_update_obstacles, 1, NULL},
			{"navigator", "map_update_freespace", CARMEN_PARAM_ONOFF, &nav_config.map_update_freespace, 1, NULL},
			{"navigator", "replan_frequency", CARMEN_PARAM_DOUBLE, &nav_config.replan_frequency, 1, NULL},
			{"navigator", "smooth_path", CARMEN_PARAM_ONOFF, &nav_config.smooth_path, 1, NULL},
			{"navigator", "dont_integrate_odometry", CARMEN_PARAM_ONOFF, &nav_config.dont_integrate_odometry, 1, NULL},
			{"navigator", "plan_to_nearest_free_point", CARMEN_PARAM_ONOFF,	&nav_config.plan_to_nearest_free_point, 1, NULL},
			{"navigator_astar", "path_interval", CARMEN_PARAM_DOUBLE, &astar_config.path_interval, 1, NULL},
			{"navigator_astar", "state_map_resolution", CARMEN_PARAM_INT, &astar_config.state_map_resolution, 1, NULL},
			{"navigator_astar", "state_map_theta_resolution", CARMEN_PARAM_INT, &astar_config.state_map_theta_resolution, 1, NULL},
			{"navigator_astar", "precomputed_cost_size", CARMEN_PARAM_INT, &astar_config.precomputed_cost_size, 1, NULL},
			{"navigator_astar", "precomputed_cost_file_name", CARMEN_PARAM_STRING, &astar_config.precomputed_cost_file_name, 1, NULL},
			{"navigator_astar", "use_rs", CARMEN_PARAM_ONOFF, &astar_config.use_rs, 1, NULL},
			{"navigator_astar", "smooth_path", CARMEN_PARAM_ONOFF, &astar_config.smooth_path, 1, NULL},

			{"navigator_astar", "onroad_max_plan_time", CARMEN_PARAM_DOUBLE, &astar_config.onroad_max_plan_time, 1, NULL},
			{"navigator_astar", "robot_fat_space", CARMEN_PARAM_DOUBLE, &astar_config.robot_fat_space, 1, NULL},

	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);

	carmen_param_install_params(argc, argv, param_list, num_items);

	if (nav_config.goal_size < robot_config.approach_dist) {
		carmen_warn("%sBad things will happen when the approach distance is\n"
				"less than the navigator goal size. Changing navigator\n"
				"goal size to be %f (robot approach distance).%s\n\n",
				carmen_red_code, robot_config.approach_dist,
				carmen_normal_code);
		nav_config.goal_size = robot_config.approach_dist;
	}

	carmen_param_get_onoff("cheat", &cheat, NULL);
}


//static void
//publish_navigator_ackerman_periodic_messages()
//{
//	publish_navigator_ackerman_messages();
//}


int 
main(int argc, char **argv)
{
	int x, y;
	float theta;
	char *goal_string;

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	read_parameters(argc, argv);

	if (carmen_navigator_ackerman_initialize_ipc() < 0)
		carmen_die("Error: could not connect to IPC Server\n");

	signal(SIGINT, navigator_shutdown);

	carmen_mapper_subscribe_map_message(NULL, (carmen_handler_t) mapper_map_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_behavior_selector_subscribe_goal_list_message(NULL, (carmen_handler_t) goal_list_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_behavior_selector_subscribe_current_state_message(NULL, (carmen_handler_t) state_handler, CARMEN_SUBSCRIBE_LATEST);

	if (cheat) 
	{
		if (IPC_isMsgDefined(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME)) 
		{
			IPC_subscribe(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME, simulator_handler, NULL);
			IPC_setMsgQueueLength(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME, 1);
		} 
		else 
		{
			carmen_warn("Can't cheat: not using simulator (truepos message not defined).\n");
			cheat = 0;
		}
	} 
	else 
	{
		carmen_localize_ackerman_subscribe_globalpos_message(
				&globalpos, (carmen_handler_t)localize_globalpos_handler,
				CARMEN_SUBSCRIBE_LATEST);
	}

	if (carmen_param_get_string("init_goal", &goal_string, NULL) == 1) 
	{
		sscanf(goal_string, "%d %d %f", &x, &y, &theta);
		carmen_navigator_ackerman_goal(x, y, theta);
	}

	//	carmen_ipc_addPeriodicTimer(2, (TIMER_HANDLER_TYPE) publish_navigator_ackerman_periodic_messages, NULL);

	carmen_ipc_dispatch();

	return 0;
}
