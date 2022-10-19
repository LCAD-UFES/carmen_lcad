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


/** @addtogroup navigator libplanner_interface **/
// @{

/** \file planner_interface.h
 * \brief Definition of interface to the shortest-path motion planner library.
 *
 * This file specifies the interface to the motion planner library. It does not
 * handle any IPC communication or move the robot in any way. It accepts a
 * map, current position of the robot and a goal, and provides a plan.  This
 * library is not thread-safe or re-entrant. 
 **/

#ifndef PLANNER_ACKERMAN_INTERFACE_H
#define PLANNER_ACKERMAN_INTERFACE_H

#include "../../src/navigator_astar/navigator_astar.hpp"
#include "navigator_astar_messages.h"

#ifdef __cplusplus
extern "C" {
#endif




/** Updates the internal position of the robot inside the planner
      and regenerates the trajectory. 
      Returns 1 if a new path was generated, otherwise returns 0 **/

int carmen_planner_ackerman_update_robot(carmen_robot_and_trailers_traj_point_t *new_position,
		carmen_navigator_config_t *nav_conf, carmen_robot_ackerman_config_t *robot_conf);

/** Updates the current goal, replans and regenerates the trajectory.
      Returns 1 if a new path was generated, otherwise returns 0 */

int carmen_planner_ackerman_update_goal(carmen_robot_and_trailers_traj_point_t *new_goal, carmen_navigator_config_t *nav_conf);

/** Assumes value of waypoint passed in is current robot position.
     Regenerates trajectory, searches along path for next waypoint
     that is more than nav_conf->goal_dist away from the current
     robot position. If this is the goal point, also checks to 
     see if robot needs to turn to face goal orientation (only matters
     if the goal was specified with an orientation. 
     Returns 1 if robot reached goal, returns -1 if no path
     exists, otherwise returns 0 and fills in next destination
     waypoint. waypoint_index is the index of the waypoint in the 
     trajectory **/

int carmen_planner_ackerman_next_waypoint(carmen_robot_and_trailers_traj_point_t *waypoint,
		int* waypoint_index,
		carmen_navigator_config_t *nav_conf);


/** Updates the planner to have a new map. **/

void carmen_planner_ackerman_set_map(carmen_map_t *map,
		carmen_robot_ackerman_config_t *robot_conf);

void carmen_planner_ackerman_set_cost_map(carmen_map_t *map);

/** Clears any local modifications the planner may have made to
      its internal map. **/

void carmen_planner_ackerman_reset_map(carmen_robot_ackerman_config_t *robot_conf);

/** Adds a laser scan to the planner's internal map using ray-tracing.
      Scans are forgotten over time, or can be removed using 
      carmen_planner_reset_map. **/

void carmen_planner_ackerman_update_map(carmen_robot_ackerman_laser_message *laser_msg,
		carmen_navigator_config_t *nav_config,
		carmen_robot_ackerman_config_t *robot_conf);

/** A helper function for replacing the map and initializing the robot
      function. 
 **/

void carmen_planner_ackerman_update_grid(carmen_map_p new_map,
		carmen_robot_and_trailers_traj_point_t *new_position,
		carmen_robot_ackerman_config_t *robot_conf,
		carmen_navigator_config_t *nav_conf);

/** A helper function for extracting the internal representation
      of the map, cost map or utility function. 
 **/

carmen_navigator_ackerman_map_message* carmen_planner_get_map_message
(carmen_navigator_ackerman_map_t map_type);

/** Returns the current state of the plan.
 **/

void carmen_planner_ackerman_get_status(carmen_planner_status_p status);

/** A helper function for getting the current utility function.
      Probably subsumed by carmen_planner_get_map_message, but 
      returns the utility function as an array of doubles of same
      size as the map, in row-major order.
 **/

double *carmen_planner_ackerman_get_utility(void);

/** A utility function for determining whether or not the goal is accessible
      from the current robot position. Returns 1 if the goal has been set and
      a feasible path exists from the robot to the goal, returns 0 otherwise.
 **/

int carmen_planner_ackerman_goal_reachable(void);

void carmen_planner_ackerman_set_goal_list(carmen_robot_and_trailers_traj_point_t *goal_list, int list_size, carmen_navigator_config_t *nav_conf);

int carmen_planner_ackerman_change_goal(carmen_navigator_config_t *nav_conf);

void carmen_planner_ackerman_regenerate_trajectory();

void carmen_planner_ackerman_regenerate_trajectory_astar(void);

void clear_goal_list();

#ifdef __cplusplus
}
#endif

#endif
// @}
