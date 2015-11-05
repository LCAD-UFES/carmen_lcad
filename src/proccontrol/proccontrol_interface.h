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


/** @addtogroup proccontrol libproccontrol_interface **/
// @{

/** \file proccontrol_interface.h
 * \brief Definition of the interface of the module proccontrol.
 *
 * This file specifies the interface to subscribe the messages of
 * that module and to receive its data via ipc.
 **/

#ifndef CARMEN_PROCCONTROL_INTERFACE_H
#define CARMEN_PROCCONTROL_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/proccontrol_messages.h>

void
carmen_proccontrol_subscribe_pidtable_message(carmen_proccontrol_pidtable_message
					     *pidtable,
					     carmen_handler_t handler,
					     carmen_subscribe_t subscribe_how);
  
void
carmen_proccontrol_unsubscribe_pidtable_message(carmen_handler_t handler);

void
carmen_proccontrol_subscribe_output_message(carmen_proccontrol_output_message *output,
					   carmen_handler_t handler,
					   carmen_subscribe_t subscribe_how);
  
void
carmen_proccontrol_unsubscribe_output_message(carmen_handler_t handler);

void 
carmen_proccontrol_set_module_state(char *module_name, int requested_state);

void 
carmen_proccontrol_set_group_state(char *group_name, int requested_state);

#ifdef __cplusplus
}
#endif

#endif
// @}
