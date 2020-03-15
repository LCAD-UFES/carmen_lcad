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

 /** @addtogroup base libsegwary_interface **/
// @{

/** \file segway_interface.h
 * \brief Definition of the messages for this module.
 *
 * This file specifies the messages for this modules used to transmit
 * data via ipc to other modules.
 **/

#ifndef SEGWAY_INTERFACE_H
#define SEGWAY_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "segway_messages.h"

void
carmen_segway_subscribe_pose_message(carmen_segway_pose_message *pose,
				     carmen_handler_t handler,
				     carmen_subscribe_t subscribe_how);

void
carmen_segway_subscribe_battery_message(carmen_segway_battery_message *battery,
					carmen_handler_t handler,
					carmen_subscribe_t subscribe_how);

void carmen_segway_kill_command(void);

#ifdef __cplusplus
}
#endif

#endif
// @}
