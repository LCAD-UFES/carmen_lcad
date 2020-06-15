/*********************************************************
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

#include "offroad_planner_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_offroad_planner_subscribe_plan_message(carmen_offroad_planner_plan_message *plan,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);

void
carmen_offroad_planner_unsubscribe_plan_message(carmen_handler_t handler);


void
carmen_offroad_planner_publish_plan(carmen_offroad_planner_plan_message *plan);


#ifdef __cplusplus
}
#endif

#endif
// @}
