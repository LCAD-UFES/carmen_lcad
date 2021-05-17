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

#ifndef NAVIGATOR_GRAPHICS_H
#define NAVIGATOR_GRAPHICS_H

#include <carmen/carmen.h>
#include <carmen/dynamic_object_detector_interface.h>
#include "navigator_panel.h"
#include "navigator_gui_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

int navigator_graphics_init(int argc, char *argv[],
		carmen_localize_ackerman_globalpos_message * msg,
		carmen_robot_ackerman_config_t * robot_conf_param,
		carmen_navigator_config_t * nav_conf_param,
		carmen_navigator_panel_config_t * nav_panel_conf_param);

void navigator_graphics_update_display(carmen_traj_point_p	new_robot, carmen_world_point_p new_goal, int Autonomous);
void navigator_graphics_update_simulator_truepos(carmen_point_t truepose);
void navigator_graphics_update_fused_odometry(carmen_point_t fused_odometry_pose);
void navigator_graphics_update_plan(carmen_robot_and_trailer_traj_point_t *new_plan, int plan_length);
void navigator_graphics_update_path(carmen_robot_and_trailer_traj_point_t* new_plan, int path_length, int path_id);
void navigator_graphics_update_plan_tree(carmen_robot_and_trailer_traj_point_t *p1, carmen_robot_and_trailer_traj_point_t *p2, int plan_tree_length);
void navigator_graphics_update_people(carmen_world_point_p people_array, int num_people);
void navigator_graphics_start(char *path);
void navigator_graphics_add_ipc_handler(GdkInputFunction handle_ipc);
carmen_world_point_p navigator_graphics_get_current_path();
void navigator_graphics_change_map(carmen_map_p New_Map);
void navigator_graphics_display_map(carmen_map_t *new_map, carmen_navigator_map_t type);
void navigator_graphics_add_placelist(carmen_map_placelist_p new_placelist);
void navigator_graphics_update_goal_list(carmen_robot_and_trailer_traj_point_t* goal_list, int size);
void navigator_graphics_update_waypoint_list(carmen_robot_and_trailer_traj_point_t* waypoint_list, int size);

void navigator_graphics_update_xsens_pos(carmen_point_t xsenspose);

void navigator_graphics_update_simulator_objects(int num_objects, carmen_simulator_ackerman_objects_t *objects_list);
void navigator_graphics_initialize_dynamics(carmen_list_t *people);
void navigator_graphics_update_dynamics(void);

void navigator_graphics_display_config(char *attribute, int value,
		char *status_message);
void navigator_graphics_reset(void);

void navigator_graphics_update_behavior_selector_state(carmen_behavior_selector_state_message msg);

int navigator_graphics_update_map();

void navigator_graphics_redraw_superimposed();

struct point
{
	double posiX;
	double posiY;
};

typedef struct points
{
	carmen_world_point_t point;
	struct points *next;
} pointers;

typedef struct fpoints
{
	pointers *curr;
	pointers *begin;
	pointers *end;
	int count;
} fpointers;

#ifdef __cplusplus
}
#endif

#endif
