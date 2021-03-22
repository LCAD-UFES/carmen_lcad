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

#ifndef NAVIGATOR_PANEL_H
#define NAVIGATOR_PANEL_H

#include <carmen/carmen.h>
#include <carmen/behavior_selector_messages.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	double initial_map_zoom;
	int	   track_robot;
	int    draw_path;
	int	   draw_waypoints;
	int	   draw_robot_waypoints;
	int	   show_particles;
	int    show_fused_odometry;
	int	   show_gaussians;
	int	   show_lasers;
	int	   show_motion_path;
	int	   show_command_path;
	int	   show_dynamic_objects;
	int	   show_simulator_objects;
	int	   show_true_pos;
	int	   show_tracked_objects;
	int	   use_ackerman;
	int	   use_exploration;
} carmen_navigator_panel_config_t;

void navigator_update_robot(carmen_world_point_p robot);
void navigator_set_goal(double x, double y, double theta);
void navigator_set_algorithm(carmen_behavior_selector_algorithm_t algorithm, carmen_behavior_selector_task_t task);
void navigator_set_goal_by_place(carmen_place_p place);
void navigator_stop_moving(void);
void navigator_start_moving(void);
void navigator_get_map(carmen_navigator_map_t New_Display, int is_superimposed);
carmen_map_t* navigator_get_offline_map_pointer();
carmen_map_t* navigator_get_complete_map_map_pointer();
void get_initial_map(void);

#ifdef __cplusplus
}
#endif

#endif
