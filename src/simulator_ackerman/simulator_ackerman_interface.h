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

#ifndef SIMULATOR_ACKERMAN_INTERFACE_H
#define SIMULATOR_ACKERMAN_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "simulator_ackerman_messages.h"

int 
carmen_simulator_ackerman_set_truepose(carmen_point_t *point);

void 
carmen_simulator_ackerman_subscribe_truepos_message(carmen_simulator_ackerman_truepos_message
					   *truepos,
					   carmen_handler_t handler,
					   carmen_subscribe_t subscribe_how);
int
carmen_simulator_ackerman_query_truepos(carmen_simulator_ackerman_truepos_message **truepos_msg);

void 
carmen_simulator_ackerman_subscribe_objects_message(carmen_simulator_ackerman_objects_message 
					   *objects,
					   carmen_handler_t handler,
					   carmen_subscribe_t subscribe_how);
int 
carmen_simulator_ackerman_query_objects(carmen_simulator_ackerman_objects_message **objects_msg);

int 
carmen_simulator_ackerman_set_object(carmen_point_t *point, double speed,
			    carmen_simulator_ackerman_object_t type);

void carmen_simulator_ackerman_connect_robots(char *other_central);

void carmen_simulator_ackerman_clear_objects(void);

void carmen_simulator_ackerman_next_tick(void);

void
carmen_simulator_ackerman_subscribe_external_truepos_message(carmen_simulator_ackerman_truepos_message
					    *truepos,
					    carmen_handler_t handler,
					    carmen_subscribe_t subscribe_how);

void
carmen_simulator_ackerman_unsubscribe_external_truepos_message(carmen_handler_t handler);

int
carmen_simulator_ackerman_publish_external_truepose(carmen_simulator_ackerman_truepos_message *msg);

#ifdef __cplusplus
}
#endif

#endif
// @}
