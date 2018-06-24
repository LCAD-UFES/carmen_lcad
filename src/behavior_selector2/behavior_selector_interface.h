/*
 * behavior_selector_interface.h
 *
 *  Created on: 28/09/2012
 *      Author: romulo
 */

#ifndef BEHAVIOR_SELECTOR_INTERFACE_H_
#define BEHAVIOR_SELECTOR_INTERFACE_H_

#include "behavior_selector_messages.h"

#ifdef __cplusplus
extern "C" {
#endif


void carmen_behavior_selector_subscribe_current_state_message(
		carmen_behavior_selector_state_message *msg,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);

void carmen_behavior_selector_subscribe_goal_list_message(
		carmen_behavior_selector_goal_list_message *msg,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);

void carmen_behavior_selector_set_algorithm(carmen_behavior_selector_algorithm_t algorithm, carmen_behavior_selector_state_t state);

void carmen_behavior_selector_set_goal_source(carmen_behavior_selector_goal_source_t goal_source);

void carmen_behavior_selector_set_state(carmen_behavior_selector_state_t state);

void carmen_behavior_selector_add_goal(carmen_point_t goal);

void carmen_behavior_selector_clear_goal_list();

void carmen_behavior_selector_remove_goal();

char *get_low_level_state_name(carmen_behavior_selector_low_level_state_t state);

#ifdef __cplusplus
}
#endif



#endif /* BEHAVIOR_SELECTOR_INTERFACE_H_ */
