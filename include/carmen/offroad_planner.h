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

#ifndef OFFROAD_PLANNER_H
#define OFFROAD_PLANNER_H

#include <stdbool.h>
#include <carmen/behavior_selector_interface.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	carmen_ackerman_traj_point_t *points;
	int length;
	int capacity;
} offroad_planner_path_t;

typedef struct
{
	carmen_ackerman_traj_point_t robot;
	carmen_ackerman_traj_point_t goal;
	offroad_planner_path_t path;
	int goal_set;
} offroad_planner_plan_t;


void carmen_navigator_ackerman_goal_triplet(carmen_ackerman_traj_point_p point);
void carmen_navigator_ackerman_goal(double x, double y, double theta);
int carmen_navigator_ackerman_goal_place(char *name);
void carmen_navigator_ackerman_set_max_velocity(double vel);
carmen_map_placelist_p carmen_navigator_ackerman_get_places(void);
int carmen_navigator_ackerman_autonomous_status(void);
void carmen_navigator_ackerman_start_autonomous(void);
bool smooth_path_using_conjugate_gradient(carmen_ackerman_traj_point_t *input_path, int num_poses);
int update_distance_map(double *utility_map, double *cost_map, int x_size, int y_size, int goal_x, int goal_y);
int update_distance_map_new(double *utility_map, double *cost_map, int x_size, int y_size, int goal_x, int goal_y);

#ifdef __cplusplus
}
#endif

#endif
