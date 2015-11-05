/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, and Sebastian Thrun
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

/** @addtogroup laser liblaser_interface **/
// @{

/** \file laser_interface.h
 * \brief Definition of the interface of the module laser.
 *
 * This file specifies the interface to subscribe the messages of
 * that module and to receive its data via ipc.
 **/

#ifndef CARMEN_LASER_INTERFACE_H
#define CARMEN_LASER_INTERFACE_H

#include "laser_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

/* laser message */

void
carmen_laser_subscribe_frontlaser_message(carmen_laser_laser_message *laser,
				     carmen_handler_t handler,
				     carmen_subscribe_t subscribe_how);

void
carmen_laser_subscribe_rearlaser_message(carmen_laser_laser_message *laser,
				     carmen_handler_t handler,
				     carmen_subscribe_t subscribe_how);

void
carmen_laser_subscribe_laser3_message(carmen_laser_laser_message *laser,
				     carmen_handler_t handler,
				     carmen_subscribe_t subscribe_how);

void
carmen_laser_subscribe_laser4_message(carmen_laser_laser_message *laser,
				     carmen_handler_t handler,
				     carmen_subscribe_t subscribe_how);


void
carmen_laser_subscribe_laser5_message(carmen_laser_laser_message *laser,
				     carmen_handler_t handler,
				     carmen_subscribe_t subscribe_how);


void
carmen_laser_subscribe_laser1_message(carmen_laser_laser_message *laser,
				     carmen_handler_t handler,
				     carmen_subscribe_t subscribe_how);


void
carmen_laser_subscribe_laser2_message(carmen_laser_laser_message *laser,
				     carmen_handler_t handler,
				     carmen_subscribe_t subscribe_how);




void
carmen_laser_subscribe_laser_message(int laser_id,
				     carmen_laser_laser_message *laser,
				     carmen_handler_t handler,
				     carmen_subscribe_t subscribe_how);

void 
carmen_laser_unsubscribe_laser_message(int laser_id,
				       carmen_handler_t handler);

void
carmen_laser_define_laser_message(int laser_id);


void
carmen_laser_publish_laser_message(int laser_id, carmen_laser_laser_message* msg);


/* alive message */

void
carmen_laser_subscribe_alive_message(carmen_laser_alive_message *alive,
				     carmen_handler_t handler,
				     carmen_subscribe_t subscribe_how);

void
carmen_laser_unsubscribe_alive_message(carmen_handler_t handler);

void
carmen_laser_define_alive_message();

void 
carmen_laser_get_offset(int which, carmen_point_t *laser_offset);

/* void */
/* carmen_laser_publish_alive_message(carmen_laser_alive_message* msg);*/



#ifdef __cplusplus
}
#endif

#endif
// @}
