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


#include <carmen/carmen_graphics.h>
#include <carmen/navigator_ackerman_interface.h>
#include <carmen/carmen.h>
#include "navigator_graphics.h"
#include "navigator_panel.h"
#include <carmen/mapper_interface.h>
#include <carmen/fused_odometry_messages.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/map_server_interface.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/grid_mapping.h>
#include "navigator_gui_interface.h"

#ifdef USE_DOT
#include <carmen/dot.h>
#include <carmen/dot_messages.h>
#include <carmen/dot_interface.h>
#endif

static carmen_robot_ackerman_config_t	 robot_config;
static carmen_navigator_config_t nav_config;
static carmen_navigator_panel_config_t nav_panel_config;
static carmen_navigator_map_t map_type = CARMEN_NAVIGATOR_MAP_v;
static carmen_navigator_map_t superimposedmap_type = CARMEN_NONE_v;
static carmen_map_p map, cost_map, offline_map = NULL, localize_map = NULL, navigator_map = NULL, complete_map = NULL;
static double last_navigator_status = 0.0;
static int	  is_graphics_up = 0;

static double last_v = 0, last_phi = 0;
static carmen_world_point_t last_goal;
static int goal_set = 0, autonomous = 0;
static carmen_point_t localize_std;

static char *map_path = NULL;

void
navigator_status_handler(carmen_navigator_ackerman_status_message *msg)
{
	carmen_verbose("Got Status message: Robot %.1f %.1f %.2f Goal: %.0f %.0f\n",
			msg->robot.x, msg->robot.y, msg->robot.theta,
			msg->goal.x, msg->goal.y);

	last_navigator_status = msg->timestamp;

	last_goal.map = map;
	goal_set = msg->goal_set;
	autonomous = msg->autonomous;


	if (!is_graphics_up)
		return;

	if (msg->goal_set)
	{
		last_goal.pose.x = msg->goal.x;
		last_goal.pose.y = msg->goal.y;
		last_goal.pose.theta = msg->goal.theta;

		navigator_graphics_update_display(NULL, &last_goal, msg->autonomous);
	}
	else
	{
		navigator_graphics_update_display(NULL, NULL, msg->autonomous);
	}
}


static void
navigator_ackerman_status_handler(carmen_navigator_ackerman_status_message *msg)
{
	carmen_verbose("Got Status message: Robot %.1f %.1f %.2f Goal: %.0f %.0f\n",
			msg->robot.x, msg->robot.y, msg->robot.theta,
			msg->goal.x, msg->goal.y);

	last_navigator_status = msg->timestamp;

	last_goal.map = map;
	goal_set = msg->goal_set;
	autonomous = msg->autonomous;


	if (!is_graphics_up)
	{
		return;
	}

	if (msg->goal_set)
	{
		last_goal.pose.x = msg->goal.x;
		last_goal.pose.y = msg->goal.y;
		last_goal.pose.theta = msg->goal.theta;

		navigator_graphics_update_display(NULL, &last_goal, msg->autonomous);
	}
	else
	{
		navigator_graphics_update_display(NULL, NULL, msg->autonomous);
	}
}


static void
path_goals_and_annotations_message_handler(carmen_behavior_selector_path_goals_and_annotations_message *path_goals_and_annotations)
{
	navigator_graphics_update_goal_list(path_goals_and_annotations->goal_list, path_goals_and_annotations->goal_list_size);
}

static void
navigator_rddf_waypoints_handler(carmen_rddf_waypoints_around_end_point_message *waypoints)
{
	navigator_graphics_update_waypoint_list(waypoints->poses, waypoints->number_of_poses);
}

static void
navigator_plan_handler(carmen_navigator_ackerman_plan_message *plan)
{
	navigator_graphics_update_plan(plan->path, plan->path_length);
}

static void
path_handler (carmen_navigator_gui_path_message *msg)
{
	navigator_graphics_update_path(msg->path, msg->path_length, msg->path_id);
}


static carmen_map_t *
get_empty_map()
{
	carmen_map_t *empty_map;

	empty_map = (carmen_map_t *) malloc(sizeof(carmen_map_t));
	carmen_test_alloc(empty_map);

	carmen_grid_mapping_initialize_map(empty_map, 10, 1.0, 'm');

	return (empty_map);
}

static void
navigator_get_empty_map(int is_superimposed)
{
	if (!is_superimposed)
	{
		navigator_graphics_display_map(NULL, CARMEN_NONE_v);
		map_type = CARMEN_NONE_v;
	}
	else
	{
		superimposedmap_type = CARMEN_NONE_v;
		carmen_map_interface_set_superimposed_map(NULL);
	}
}

static void
navigator_get_localize_map(carmen_navigator_map_t type, int is_superimposed)
{
	if (localize_map != NULL)
		carmen_map_destroy(&localize_map);

	localize_map = (carmen_map_t *)calloc(1, sizeof(carmen_map_t));
	carmen_test_alloc(localize_map);

	int index;
	if (type == CARMEN_LOCALIZE_LMAP_v)
	{
		carmen_localize_ackerman_get_map(0, localize_map);
	}
	else
	{
		carmen_localize_ackerman_get_map(1, localize_map);
	}

	if (localize_map->complete_map == NULL)
	{
		return;
	}

	for (index = 0; index < localize_map->config.x_size * localize_map->config.y_size; index++)
	{
		localize_map->complete_map[index] = exp(localize_map->complete_map[index]);
	}

	if (!is_superimposed)
	{
		navigator_graphics_display_map(localize_map, type);
		map_type = type;
	}
	else
	{
		superimposedmap_type = type;
		carmen_map_interface_set_superimposed_map(localize_map);
	}
}

static void
navigator_get_lane_map(int is_superimposed)
{
	if ((cost_map == NULL) || (cost_map->complete_map == NULL))
	{
		cost_map = get_empty_map();
	}

	if (!is_superimposed)
	{
		navigator_graphics_display_map(cost_map, CARMEN_LANE_MAP_v);
		map_type = CARMEN_LANE_MAP_v;
	}
	else
	{
		superimposedmap_type = CARMEN_LANE_MAP_v;
		carmen_map_interface_set_superimposed_map(cost_map);
	}
}

static void
navigator_get_complete_map(int is_superimposed)
{
	if (is_superimposed)
		return;


	if (complete_map == NULL)
	{
		complete_map = (carmen_map_t *)malloc(sizeof(carmen_map_t));

		carmen_grid_mapping_read_complete_map(map_path, complete_map);
	}

	if (map_type != CARMEN_COMPLETE_MAP_v)
	{
		navigator_graphics_display_map(complete_map, CARMEN_COMPLETE_MAP_v);
		map_type = CARMEN_COMPLETE_MAP_v;
	}
}

static void
navigator_get_offline_map(int is_superimposed)
{
	if ((offline_map == NULL) || (offline_map->complete_map == NULL))
	{
		offline_map = get_empty_map();
	}

	if (!is_superimposed)
	{
		navigator_graphics_display_map(offline_map, CARMEN_OFFLINE_MAP_v);
		map_type = CARMEN_OFFLINE_MAP_v;
	}
	else
	{
		superimposedmap_type = CARMEN_OFFLINE_MAP_v;
		carmen_map_interface_set_superimposed_map(offline_map);
	}
}

static void
navigator_get_grid_mapping(int is_superimposed)
{
	if ((map == NULL) || (map->complete_map == NULL))
	{
		map = get_empty_map();
	}

	if (!is_superimposed)
	{
		navigator_graphics_display_map(map, CARMEN_NAVIGATOR_MAP_v);
		map_type = CARMEN_NAVIGATOR_MAP_v;
	}
	else
	{
		superimposedmap_type = CARMEN_NAVIGATOR_MAP_v;
		carmen_map_interface_set_superimposed_map(map);
	}
}

static void
navigator_get_navigator_cost_map(carmen_navigator_map_t type, int is_superimposed)
{
	if (navigator_map != NULL)
		carmen_map_destroy(&navigator_map);

	navigator_map = (carmen_map_t *)calloc(1, sizeof(carmen_map_t));
	carmen_test_alloc(navigator_map);

	memset(navigator_map, 0, sizeof(carmen_map_t));

	carmen_navigator_ackerman_get_map(type, navigator_map);

	if (navigator_map->complete_map == NULL)
		return;

	if (!is_superimposed)
	{
		navigator_graphics_display_map(navigator_map, type);
		map_type = type;
	}
	else
	{
		superimposedmap_type = type;
		carmen_map_interface_set_superimposed_map(navigator_map);
	}


}

void navigator_get_map(carmen_navigator_map_t type, int is_superimposed)
{
	switch (type)
	{
	case CARMEN_NONE_v:
		navigator_get_empty_map(is_superimposed);
		break;
	case CARMEN_LOCALIZE_LMAP_v:
		navigator_get_localize_map(type, is_superimposed);
		break;

	case CARMEN_LOCALIZE_GMAP_v:
		navigator_get_localize_map(type, is_superimposed);
		break;

	case CARMEN_LANE_MAP_v:
		navigator_get_lane_map(is_superimposed);
		break;

	case CARMEN_NAVIGATOR_MAP_v:
		navigator_get_grid_mapping(is_superimposed);
		break;

	case CARMEN_OFFLINE_MAP_v:
		navigator_get_offline_map(is_superimposed);
		break;

	case CARMEN_COMPLETE_MAP_v:
		navigator_get_complete_map(is_superimposed);
		break;

	default:
		navigator_get_navigator_cost_map(type, is_superimposed);
		break;
	}

}


/*
static void map_update_handler(carmen_map_t *new_map)
{
	//if (strcmp(new_map->config.origin, nav_config.navigator_map) == 0)
	if (navigator_graphics_update_map())
	{
		carmen_map_destroy(&map);
		map = carmen_map_clone(new_map);

		if (is_graphics_up)
		{
			navigator_graphics_change_map(map);
		}
	}
}
 */

static carmen_map_t*
copy_grid_mapping_to_map(carmen_mapper_map_message *grid_map)
{
	int i;
	carmen_map_t *map;

	map = malloc(sizeof(carmen_map_t));
	map->config = grid_map->config;
	map->complete_map = (double *) malloc(sizeof(double) * grid_map->size);

	memcpy(map->complete_map, grid_map->complete_map,
			sizeof(double) * grid_map->size);

	map->map = (double **)calloc(grid_map->config.x_size, sizeof(double *));

	for (i = 0; i < map->config.x_size; i++)
	{
		map->map[i] = map->complete_map + i * map->config.y_size;
	}

	return map;
}

static void
clone_grid_mapping_to_map(carmen_mapper_map_message *grid_map, carmen_map_t *map)
{
	map->config = grid_map->config;

	memcpy(map->complete_map, grid_map->complete_map, sizeof(double) * grid_map->size);
}

carmen_map_t*
navigator_get_complete_map_map_pointer()
{
	return complete_map;
}

carmen_map_t*
navigator_get_offline_map_pointer()
{
	return offline_map;
}

static void
offline_map_update_handler(carmen_mapper_map_message *new_map)
{
	if (new_map->size <= 0)
		return;

	if (offline_map && (new_map->config.x_size != offline_map->config.x_size || new_map->config.y_size != offline_map->config.y_size))
		carmen_map_destroy(&offline_map);

	if (offline_map)
		clone_grid_mapping_to_map(new_map, offline_map);
	else
		offline_map = copy_grid_mapping_to_map(new_map);


	if (superimposedmap_type == CARMEN_OFFLINE_MAP_v)
	{
		carmen_map_interface_set_superimposed_map(offline_map);
		navigator_graphics_redraw_superimposed();
	}
	else if (superimposedmap_type != CARMEN_NONE_v)
	{
		navigator_get_map(superimposedmap_type, 1);
	}


	if (navigator_graphics_update_map())
	{
		if (is_graphics_up && map_type == CARMEN_OFFLINE_MAP_v)
			navigator_graphics_change_map(offline_map);
		else
			navigator_get_map(map_type, 0);
	}
}

static void 
mapper_handler(carmen_mapper_map_message *message)
{
	static double last_time_stamp = 0.0;

	if ( message->size <= 0)
		return;

	// @@@ Alberto: codigo adicionado para atualizar o mapa a uma taxa maxima de 10Hz
	if ((carmen_get_time() - last_time_stamp) > 0.1)
		last_time_stamp = carmen_get_time();
	else
		return;

	if (map && (message->config.x_size != map->config.x_size || message->config.y_size != map->config.y_size))
		carmen_map_destroy(&map);

	if (map)
		clone_grid_mapping_to_map(message, map);
	else
		map = copy_grid_mapping_to_map(message);

	if (superimposedmap_type == CARMEN_NAVIGATOR_MAP_v)
	{
		carmen_map_interface_set_superimposed_map(map);
		navigator_graphics_redraw_superimposed();
	}

	if (navigator_graphics_update_map() &&
			is_graphics_up &&
			map_type == CARMEN_NAVIGATOR_MAP_v)
	{
		navigator_graphics_change_map(map);
	}
}

static void
cost_map_handler(carmen_map_server_cost_map *message)
{

	carmen_mapper_map_message new_map;
	new_map.config = message->config;
	new_map.complete_map = message->complete_map;
	new_map.size = message->size;

	if (cost_map && (new_map.config.x_size != cost_map->config.x_size || new_map.config.y_size != cost_map->config.y_size))
		carmen_map_destroy(&cost_map);

	if (cost_map)
		clone_grid_mapping_to_map(&new_map, cost_map);
	else
		cost_map = copy_grid_mapping_to_map(&new_map);

	if (superimposedmap_type == CARMEN_LANE_MAP_v)
	{
		carmen_map_interface_set_superimposed_map(cost_map);
		navigator_graphics_redraw_superimposed();
	}

	if (navigator_graphics_update_map() && map_type == CARMEN_LANE_MAP_v)
	{
		if (is_graphics_up)
		{
			navigator_graphics_change_map(cost_map);
		}
	}
}

void navigator_update_robot(carmen_world_point_p robot)
{
	if (robot == NULL)
	{
		carmen_localize_ackerman_initialize_uniform_command();
	}
	else
	{
		carmen_verbose("Set robot position to %d %d %f\n",
				carmen_round(robot->pose.x),
				carmen_round(robot->pose.y),
				carmen_radians_to_degrees(robot->pose.theta));

//		carmen_localize_ackerman_initialize_gaussian_command(robot->pose, localize_std, 0.0);
		double current_trailer_theta[MAX_NUM_TRAILERS];
		for (size_t z = 1; z < MAX_NUM_TRAILERS; z++)
			current_trailer_theta[z] = 0.0;
		carmen_localize_ackerman_initialize_gaussian_command(robot->pose, localize_std, current_trailer_theta, 1);

	}

}

void navigator_set_goal(double x, double y, double theta)
{
	carmen_verbose("Set goal to %.1f %.1f\n", x, y);
	carmen_navigator_ackerman_set_goal(x, y, theta);
}

void navigator_set_algorithm(carmen_behavior_selector_algorithm_t algorithm, carmen_behavior_selector_task_t task)
{
	carmen_behavior_selector_set_algorithm(algorithm, task, carmen_get_time());
}

void navigator_unset_goal(double x, double y)
{
	carmen_navigator_ackerman_unset_goal(x, y);
}

void navigator_set_goal_by_place(carmen_place_p place)
{
	carmen_navigator_ackerman_set_goal_place(place->name);
}

void navigator_stop_moving(void)
{
	if (!carmen_navigator_ackerman_stop())
	{
		carmen_verbose("Said stop\n");
	}
	else
	{
		carmen_verbose("Could not say stop\n");
	}
}

void navigator_start_moving(void)
{
	if (!carmen_navigator_ackerman_go())
	{
		carmen_verbose("Said go!\n");
	}
	else
	{
		carmen_verbose("could not say go!\n");
	}
}

static void nav_shutdown(int signo __attribute__ ((unused)))
{
	static int done = 0;

	if (!done)
	{
		done = 1;
		carmen_ipc_disconnect();
		exit(-1);
	}
}

static gint handle_ipc(gpointer			*data __attribute__ ((unused)),
		gint				 source __attribute__ ((unused)),
		GdkInputCondition condition __attribute__ ((unused)))
{
	carmen_ipc_sleep(0.01);

	carmen_graphics_update_ipc_callbacks((GdkInputFunction)handle_ipc);

	return 1;
}

static void 
globalpos_ack_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	carmen_traj_point_t new_robot;

	new_robot.x		= msg->globalpos.x;
	new_robot.y		= msg->globalpos.y;
	new_robot.theta = msg->globalpos.theta;
	new_robot.t_vel = msg->v;
	new_robot.r_vel = msg->phi;

	if (!is_graphics_up)
	{
		return;
	}

	if (goal_set)
	{
		navigator_graphics_update_display(&new_robot, &last_goal, autonomous);
	}
	else
	{
		navigator_graphics_update_display(&new_robot, NULL, autonomous);
	}

}

void globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	carmen_traj_point_t new_robot;

	new_robot.x		= msg->globalpos.x;
	new_robot.y		= msg->globalpos.y;
	new_robot.theta = msg->globalpos.theta;
	new_robot.t_vel = last_v;
	new_robot.r_vel = last_phi;

	if (!is_graphics_up)
	{
		return;
	}

	if (goal_set)
	{
		navigator_graphics_update_display(&new_robot, &last_goal, autonomous);
	}
	else
	{
		navigator_graphics_update_display(&new_robot, NULL, autonomous);
	}

}

static void truepos_handler(carmen_simulator_ackerman_truepos_message *msg)
{
	navigator_graphics_update_simulator_truepos(msg->truepose);
}

static void objects_handler(carmen_simulator_ackerman_objects_message *msg)
{
	navigator_graphics_update_simulator_objects(msg->num_objects, msg->objects);
}

static void plan_tree_handler(carmen_navigator_ackerman_plan_tree_message *msg)
{
	navigator_graphics_update_plan_tree(msg->p1, msg->p2, msg->num_edges);
}


static void
fused_odometry_handler(carmen_fused_odometry_message *msg)
{
	carmen_point_t pose;
	pose.x = msg->pose.position.x;
	pose.y = msg->pose.position.y;
	pose.theta = msg->pose.orientation.yaw;

	last_v = msg->velocity.x;
	last_phi = msg->phi;

	navigator_graphics_update_fused_odometry(pose);
}

static void
state_handler(carmen_behavior_selector_state_message *msg)
{
	navigator_graphics_update_behavior_selector_state(*msg);
}


static void
odometry_handler(carmen_base_ackerman_odometry_message *msg)
{
	last_phi = msg->phi;
	last_v = msg->v;
}

static void
display_config_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData,
		void *clientData __attribute__ ((unused)))
{
	IPC_RETURN_TYPE err = IPC_OK;
	FORMATTER_PTR	formatter;
	carmen_navigator_ackerman_display_config_message msg;

	formatter = IPC_msgInstanceFormatter(msgRef);
	err = IPC_unmarshallData(formatter, callData, &msg,
			sizeof(carmen_navigator_ackerman_display_config_message));
	IPC_freeByteArray(callData);

	carmen_test_ipc_return(err, "Could not unmarshall",
			IPC_msgInstanceName(msgRef));

	if (msg.reset_all_to_defaults)
	{
		navigator_graphics_reset();
	}
	else
	{
		navigator_graphics_display_config
		(msg.attribute, msg.value, msg.status_message);
	}

	free(msg.attribute);

	if (msg.status_message)
	{
		free(msg.status_message);
	}
}


/*
void
get_initial_map()
{
	if(!carmen_map_server_get_current_offline_map(offline_map))
		carmen_map_get_gridmap(offline_map);

	if (offline_map->map == NULL)
	{
		exit(0);
	}

	if (is_graphics_up)
	{
		navigator_graphics_change_map(offline_map);
	}
}
 */


static void
read_parameters(int argc, char *argv[],
		carmen_robot_ackerman_config_t *robot_config,
		carmen_navigator_config_t *nav_config,
		carmen_navigator_panel_config_t
		*navigator_panel_config)
{
	int num_items;

	carmen_param_t param_list[] = {
			{"robot",		"length",			CARMEN_PARAM_DOUBLE, &(robot_config->length),				0, NULL},
			{"robot",		"width",			CARMEN_PARAM_DOUBLE, &(robot_config->width),				0, NULL},
			{"robot",		"maximum_acceleration_forward",	CARMEN_PARAM_DOUBLE, &(robot_config->maximum_acceleration_forward),			1, NULL},
			{"robot",		"rectangular",			CARMEN_PARAM_ONOFF,  &(robot_config->rectangular),			1, NULL},
			{"navigator",		"map_update_radius",		CARMEN_PARAM_INT,    &(nav_config->map_update_radius),			1, NULL},
			{"navigator",		"goal_size",			CARMEN_PARAM_DOUBLE, &(nav_config->goal_size),				1, NULL},
			{"navigator",		"goal_theta_tolerance",		CARMEN_PARAM_DOUBLE, &(nav_config->goal_theta_tolerance),		1, NULL},
			{"navigator_panel", 	"initial_map_zoom",		CARMEN_PARAM_DOUBLE, &(navigator_panel_config->initial_map_zoom),	1, NULL},
			{"navigator_panel", 	"track_robot",			CARMEN_PARAM_ONOFF,  &(navigator_panel_config->track_robot),		1, NULL},
			{"navigator_panel", 	"draw_waypoints",		CARMEN_PARAM_ONOFF,  &(navigator_panel_config->draw_waypoints),		1, NULL},
			{"navigator_panel", 	"draw_robot_waypoints",		CARMEN_PARAM_ONOFF,  &(navigator_panel_config->draw_robot_waypoints),	1, NULL},
			{"navigator_panel", 	"show_particles",		CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_particles),		1, NULL},
			{"navigator_panel", 	"show_gaussians",		CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_gaussians),		1, NULL},
			{"navigator_panel", 	"show_laser",			CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_lasers),		1, NULL},
			{"navigator_panel", 	"show_simulator_objects", 	CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_simulator_objects), 1, NULL},
			{"navigator_panel", 	"show_true_pos",		CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_true_pos),		1, NULL},
			{"navigator_panel", 	"show_tracked_objects",		CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_tracked_objects),	1, NULL},
			{"navigator_panel", 	"show_command_path",		CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_command_path),	1, NULL},
			{"navigator_panel", 	"show_motion_path",		CARMEN_PARAM_ONOFF,  &(navigator_panel_config->show_motion_path),	1, NULL},
			{"navigator_panel", 	"draw_path",			CARMEN_PARAM_ONOFF,  &(navigator_panel_config->draw_path),		1, NULL},
			{"navigator_panel", 	"use_ackerman",			CARMEN_PARAM_ONOFF,  &(navigator_panel_config->use_ackerman),		1, NULL},
			{"navigator_panel",	"localize_std_x",		CARMEN_PARAM_DOUBLE, &localize_std.x,								1, NULL},
			{"navigator_panel",	"localize_std_y",		CARMEN_PARAM_DOUBLE, &localize_std.y,								1, NULL},
			{"navigator_panel",	"localize_std_theta",		CARMEN_PARAM_DOUBLE, &localize_std.theta,							1, NULL},

	};

	num_items = sizeof(param_list) / sizeof(param_list[0]);

	carmen_param_install_params(argc, argv, param_list, num_items);

	robot_config->rectangular = 1;

	carmen_param_t param_ackerman_list[] =
	{
			{"robot", "distance_between_front_car_and_front_wheels", 	CARMEN_PARAM_DOUBLE, &(robot_config->distance_between_front_car_and_front_wheels), 1, NULL},
			{"robot", "distance_between_front_and_rear_axles",		CARMEN_PARAM_DOUBLE, &(robot_config->distance_between_front_and_rear_axles),	 1, NULL},
			{"robot", "distance_between_rear_car_and_rear_wheels",		CARMEN_PARAM_DOUBLE, &(robot_config->distance_between_rear_car_and_rear_wheels),	 1, NULL},
			{"robot", "distance_between_rear_wheels",			CARMEN_PARAM_DOUBLE, &(robot_config->distance_between_rear_wheels),				 1, NULL}
	};

	num_items = sizeof(param_ackerman_list) / sizeof(param_ackerman_list[0]);

	carmen_param_install_params(argc, argv, param_ackerman_list, num_items);

	carmen_param_t param_cmd_list[] = {
			{"commandline", "map_path", CARMEN_PARAM_STRING, &map_path, 0, NULL},
	};

	num_items = sizeof(param_cmd_list) / sizeof(param_cmd_list[0]);

	carmen_param_allow_unfound_variables(1);

	carmen_param_install_params(argc, argv, param_cmd_list, num_items);
}

int
main(int argc, char **argv)
{
	carmen_navigator_ackerman_status_message	  status_ackerman;

	carmen_localize_ackerman_globalpos_message globalpos;
	carmen_navigator_ackerman_plan_message	 *plan;
	IPC_RETURN_TYPE err;

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	signal(SIGINT, nav_shutdown);

	read_parameters(argc, argv, &robot_config, &nav_config, &nav_panel_config);

	carmen_grid_mapping_init_parameters(0.2, 150);

	carmen_navigator_ackerman_subscribe_status_message(
			(carmen_navigator_ackerman_status_message *)&status_ackerman,
			(carmen_handler_t)navigator_ackerman_status_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_behavior_selector_subscribe_path_goals_and_annotations_message(
			NULL, (carmen_handler_t)path_goals_and_annotations_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_navigator_ackerman_subscribe_plan_message(
			NULL,
			(carmen_handler_t)navigator_plan_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_localize_ackerman_subscribe_globalpos_message(
			NULL,
			(carmen_handler_t)globalpos_ack_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_simulator_ackerman_subscribe_truepos_message(
			NULL,
			(carmen_handler_t)truepos_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_simulator_ackerman_subscribe_objects_message(
			NULL,
			(carmen_handler_t)objects_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_behavior_selector_subscribe_current_state_message(
			NULL,
			(carmen_handler_t) state_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_navigator_ackerman_subscribe_plan_tree_message(
			NULL,
			(carmen_handler_t)plan_tree_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_fused_odometry_subscribe_fused_odometry_message(
			NULL,
			(carmen_handler_t)fused_odometry_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_base_ackerman_subscribe_odometry_message(
			NULL,
			(carmen_handler_t)odometry_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_navigator_gui_subscribe_path_message(
			NULL,
			(carmen_handler_t) path_handler,
			CARMEN_SUBSCRIBE_LATEST);

	err = IPC_defineMsg(CARMEN_NAVIGATOR_ACKERMAN_DISPLAY_CONFIG_NAME, IPC_VARIABLE_LENGTH, CARMEN_NAVIGATOR_ACKERMAN_DISPLAY_CONFIG_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_NAVIGATOR_ACKERMAN_DISPLAY_CONFIG_NAME);

	err = IPC_subscribe(CARMEN_NAVIGATOR_ACKERMAN_DISPLAY_CONFIG_NAME, display_config_handler, NULL);
	carmen_test_ipc_exit(err, "Could not subscribe message", CARMEN_NAVIGATOR_ACKERMAN_DISPLAY_CONFIG_NAME);

	navigator_graphics_init(argc, argv, &globalpos, &robot_config, &nav_config, &nav_panel_config);
	navigator_graphics_add_ipc_handler((GdkInputFunction)handle_ipc);

	is_graphics_up = 1;

	map = (carmen_map_t *)calloc(1, sizeof(carmen_map_t));
	carmen_test_alloc(map);

	offline_map = (carmen_map_t *)calloc(1, sizeof(carmen_map_t));
	carmen_test_alloc(offline_map);

	cost_map = (carmen_map_t *)calloc(1, sizeof(carmen_map_t));
	carmen_test_alloc(cost_map);

	carmen_map_server_subscribe_offline_map(NULL, (carmen_handler_t) offline_map_update_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_mapper_subscribe_map_message(NULL, (carmen_handler_t) mapper_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_map_server_subscribe_cost_map(NULL, (carmen_handler_t) cost_map_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_rddf_subscribe_waypoints_around_end_point_message(NULL,
			(carmen_handler_t)navigator_rddf_waypoints_handler, CARMEN_SUBSCRIBE_LATEST);

	err = IPC_defineMsg(CARMEN_RDDF_END_POINT_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_RDDF_END_POINT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_RDDF_END_POINT_MESSAGE_NAME);


	carmen_navigator_ackerman_query_plan((carmen_navigator_ackerman_plan_message **)&plan);

#ifdef USE_DOT
	initialize_dynamics();
#endif

	if (plan && (plan->path_length > 0))
	{
		navigator_graphics_update_plan(plan->path, plan->path_length);
		free(plan->path);
		free(plan);
	}

	navigator_graphics_start(map_path);
	return 0;
}

