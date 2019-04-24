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


/** @addtogroup navigator libnavigator_interface **/
// @{

/** \file navigator_interface.h
 * \brief Definition of the interface of the module navigator.
 *
 * This file specifies the interface to subscribe the messages of
 * that module and to receive its data via ipc.
 **/

#ifndef NAVIGATOR_ASTAR_INTERFACE_H
#define NAVIGATOR_ASTAR_INTERFACE_H

#include "path_planner_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Status messages are emitted whenever an odometry, front laser or
      global position message is received. 
 */

void carmen_navigator_ackerman_subscribe_status_message(carmen_navigator_ackerman_status_message 
		*status,
		carmen_handler_t handler,
		carmen_subscribe_t
		subscribe_how);

/** A plan message is emitted whenever the plan changes.
 */

void carmen_navigator_ackerman_subscribe_plan_message(carmen_navigator_ackerman_plan_message 
		*plan,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);


/** An autonomous stopped message is emitted whenever the robot stops trying to reach
      a goal. This can happen because the robot reached its goal, 
      the navigator was told to stop navigating, or some unmodelled failure. 
 */

void carmen_navigator_ackerman_subscribe_autonomous_stopped_message
(carmen_navigator_ackerman_autonomous_stopped_message *autonomous_stopped,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);

void carmen_navigator_ackerman_unsubscribe_autonomous_stopped_message
(carmen_handler_t handler);

void
carmen_navigator_ackerman_subscribe_plan_tree_message(carmen_navigator_ackerman_plan_tree_message *plan_tree,
						      carmen_handler_t handler,
						      carmen_subscribe_t subscribe_how);

void
carmen_navigator_ackerman_unsubscribe_plan_tree_message
(carmen_handler_t handler);

int carmen_navigator_ackerman_query_status(carmen_navigator_ackerman_status_message **status);
int carmen_navigator_ackerman_query_plan(carmen_navigator_ackerman_plan_message **plan);

/** Using this function causes the robot to reach the goal, without regard to
      its final orientation. This function does not start the robot moving:
      carmen_navigator_ackerman_go() must be called. 
 */

int carmen_navigator_ackerman_set_goal(double x, double y, double theta);

/** Using this function causes the robot to reach a final pose, including
      orientation. This function does not start the robot moving:
      carmen_navigator_ackerman_go() must be called. 
 */
int carmen_navigator_ackerman_set_goal_triplet(carmen_ackerman_traj_point_p goal);

/** Using this function causes the robot to try and reach specific place, as
      specified in the current map. The navigator queries the current set of
      places defined in the map (usually provided by the param_daemon, but
      possibly by some other map server process). If
      the place is specified only as an (x,y) pair then the robot's final
      orientation will not be controlled. If the place is specified as an
      (x,y,theta) triplet or an (x, y, theta) (sigma x, sigma y, sigma theta)
      6 dimensional location, the planner will try for a final orientation
      matching the specified theta as well. 
 */
int carmen_navigator_ackerman_set_goal_place(char *name);

/** Causes the navigator to stop trying to reach the goal. This also causes
      an autonomous_stopped message to be emitted. The goal position is
      unaffected. The trajectory can be resumed by calling
      carmen_navigator_ackerman_go() again. 
 */
int carmen_navigator_ackerman_stop(void);
/** Starts the robot moving toward the goal.
 */
int carmen_navigator_ackerman_go(void);

/** Asks for one of the navigator maps. The navigator maintains its own
      internal occupancy grid, updated with recent laser measurements. The
      cost map and utility function can also be queried. 
 */
int carmen_navigator_ackerman_get_map(carmen_navigator_ackerman_map_t map_type,
		carmen_map_t *map);

int carmen_navigator_ackerman_unset_goal(double x, double y);

void
carmen_navigator_ackerman_subscribe_astar_goal_list_message(carmen_navigator_ackerman_astar_goal_list_message *goal_list,
		carmen_handler_t handler, carmen_subscribe_t subscribe_how);



#ifdef __cplusplus
}
#endif

#endif
// @}


