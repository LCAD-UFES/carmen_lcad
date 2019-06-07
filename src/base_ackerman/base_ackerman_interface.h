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

#ifndef BASE_ACKERMAN_INTERFACE_H
#define BASE_ACKERMAN_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/carmen.h>
#include "base_ackerman_messages.h"

int carmen_simulator_ackerman_set_truepose(carmen_point_t *point);

void carmen_simulator_ackerman_subscribe_truepos_message(carmen_simulator_ackerman_truepos_message
					   *truepos, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

int carmen_simulator_ackerman_query_truepos(carmen_simulator_ackerman_truepos_message **truepos_msg);

void carmen_simulator_ackerman_connect_robots(char *other_central);

void carmen_simulator_ackerman_next_tick(void);

void carmen_base_ackerman_subscribe_odometry_message(carmen_base_ackerman_odometry_message
					    *odometry, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void carmen_base_ackerman_unsubscribe_odometry_message(carmen_handler_t handler);

// Message redefined to insert a module between the obstacle_avoider and the ford_escape_hybrid
void carmen_base_ackerman_subscribe_odometry_message_2(carmen_base_ackerman_odometry_message *odometry,
				       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void carmen_base_ackerman_subscribe_motion_command(carmen_base_ackerman_motion_command_message *motion_command,
				     carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void carmen_base_ackerman_publish_motion_command(carmen_ackerman_motion_command_p motion_command, int num_motion_commands, double timestamp);


// Message redefined to insert a module between the obstacle_avoider and the ford_escape_hybrid
void carmen_base_ackerman_subscribe_motion_command_2(carmen_base_ackerman_motion_command_message *motion_command,
				     carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void carmen_base_ackerman_publish_motion_command_2(carmen_ackerman_motion_command_p motion_command, int num_motion_commands, double timestamp);

#ifdef __cplusplus
}
#endif

#endif
// @}
