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

#ifndef CARMEN_TRAJECTORY_ACKERMAN_H
#define CARMEN_TRAJECTORY_ACKERMAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "navigator_astar_messages.h"

	void check_path_capacity(carmen_planner_path_p path);
	int	carmen_planner_util_add_path_point(carmen_robot_and_trailers_traj_point_t point, carmen_planner_path_p path);

	carmen_robot_and_trailers_traj_point_t *carmen_planner_util_get_path_point(int index, carmen_planner_path_p path);

	void carmen_planner_util_set_path_point(int index, carmen_robot_and_trailers_traj_point_t *path_point,
			carmen_planner_path_p path);

	void carmen_planner_util_insert_blank(int index, carmen_planner_path_p path);
	void carmen_planner_util_insert_path_point(int index, carmen_robot_and_trailers_traj_point_t *current_point,
			carmen_planner_path_p path);
	void carmen_planner_util_set_path_velocities(int index, double v,
			double phi,
			carmen_planner_path_p path);
	void carmen_planner_util_clear_path(carmen_planner_path_p path);
	void carmen_planner_util_clip_path(int length, carmen_planner_path_p path);
	void carmen_planner_util_delete_path_point(int index, carmen_planner_path_p path);
	void carmen_planner_util_test_trajectory(carmen_planner_path_p path);

#ifdef __cplusplus
}
#endif

#endif
