/*
 * rrt_planner_interface.h
 *
 *  Created on: 16/12/2011
 *      Author: rradaelli
 */

#ifndef RRT_PLANNER_INTERFACE_H_
#define RRT_PLANNER_INTERFACE_H_

#include "rrt_planner_message.h"

#ifdef __cplusplus
extern "C" {
#endif

void carmen_rrt_planner_define_robot_tree_message();

void carmen_rrt_planner_define_goal_tree_message();

void carmen_rrt_planner_define_status_message();

void carmen_rrt_planner_define_plan_message();

void carmen_rrt_planner_subscribe_robot_tree_message(carmen_rrt_planner_tree_message *message,
													 carmen_handler_t				  handler,
													 carmen_subscribe_t				  subscribe_how);

void carmen_rrt_planner_subscribe_goal_tree_message(carmen_rrt_planner_tree_message *message,
													carmen_handler_t				 handler,
													carmen_subscribe_t				 subscribe_how);

void carmen_rrt_planner_set_goal(carmen_point_t goal);

void carmen_rrt_planner_subscribe_set_goal_message(carmen_rrt_planner_set_goal_message *message,
												   carmen_handler_t						handler,
												   carmen_subscribe_t					subscribe_how);

void carmen_rrt_planner_subscribe_status_message(carmen_rrt_planner_status_message *message,
												 carmen_handler_t					handler,
												 carmen_subscribe_t					subscribe_how);

void carmen_rrt_planner_subscribe_plan_message(carmen_rrt_planner_plan_message *message,
											   carmen_handler_t					handler,
											   carmen_subscribe_t				subscribe_how);

void carmen_rrt_planner_subscribe_go_message(carmen_handler_t	handler,
											 carmen_subscribe_t subscribe_how);

void carmen_rrt_planner_subscribe_stop_message(carmen_handler_t	  handler,
											   carmen_subscribe_t subscribe_how);

void carmen_rrt_planner_go();

void carmen_rrt_planner_stop();

#ifdef __cplusplus
}
#endif

#endif /* RRT_PLANNER_INTERFACE_H_ */
