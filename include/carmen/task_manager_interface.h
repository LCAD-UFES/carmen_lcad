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

/** @addtogroup simulator libsimulator_interface **/
// @{

/** \file simulator_interface.h
 * \brief Definition of the interface of the module simulator.
 *
 * This file specifies the interface to subscribe the messages of
 * that module and to receive its data via ipc.
 **/

#ifndef TASK_MANAGER_INTERFACE_H
#define TASK_MANAGER_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/carmen.h>
#include "task_manager_messages.h"

void
carmen_task_manager_subscribe_set_collision_geometry_message(carmen_task_manager_set_collision_geometry_message *msg,
				     carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_task_manager_publish_set_collision_geometry_message(int geometry, double timestamp);

void
carmen_task_manager_subscribe_desired_engage_state_message(carmen_task_manager_desired_engage_state_message *msg,
				     carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_task_manager_subscribe_set_semi_trailer_type_message(carmen_task_manager_set_semi_trailer_type_message *msg,
				     carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_task_manager_publish_desired_engage_state_message(int desired_engage_state, double timestamp);

void
carmen_task_manager_publish_set_semi_trailer_type_message(int semi_trailer_type, double timestamp);

#ifdef __cplusplus
}
#endif

#endif
// @}
