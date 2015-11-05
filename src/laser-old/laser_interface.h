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

#include <carmen/laser_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Subscribes the raw front laser (rawlaser1) message **/
void
carmen_laser_subscribe_frontlaser_message(carmen_laser_laser_message *laser,
					  carmen_handler_t handler,
					  carmen_subscribe_t subscribe_how);

/** Subscribes the raw rear laser (rawlaser2) message **/
void
carmen_laser_subscribe_rearlaser_message(carmen_laser_laser_message *laser,
					 carmen_handler_t handler,
					 carmen_subscribe_t subscribe_how);

/** Subscribes the corresonding message. Note that laser1 is the same
    than frontlaser.  **/
void
carmen_laser_subscribe_laser1_message(carmen_laser_laser_message *laser,
				      carmen_handler_t handler,
				      carmen_subscribe_t subscribe_how);

/** Subscribes the corresonding message. Note that laser2 is the same
    than rearlaser.  **/
void
carmen_laser_subscribe_laser2_message(carmen_laser_laser_message *laser,
				      carmen_handler_t handler,
				      carmen_subscribe_t subscribe_how);

/** Subscribes the corresonding message.  **/
void
carmen_laser_subscribe_laser3_message(carmen_laser_laser_message *laser,
				      carmen_handler_t handler,
				      carmen_subscribe_t subscribe_how);

/** Subscribes the corresonding message.  **/
void
carmen_laser_subscribe_laser4_message(carmen_laser_laser_message *laser,
				      carmen_handler_t handler,
				      carmen_subscribe_t subscribe_how);


/** Subscribes the corresonding message.  **/
void
carmen_laser_subscribe_laser5_message(carmen_laser_laser_message *laser,
				      carmen_handler_t handler,
				      carmen_subscribe_t subscribe_how);

void 
carmen_laser_get_offset(int which, carmen_point_t *laser_offset);

/** Subscribes the corresonding message.  **/
void
carmen_laser_subscribe_alive_message(carmen_laser_alive_message *alive,
				     carmen_handler_t handler,
				     carmen_subscribe_t subscribe_how);


/** Unsubscribes the corresonding message. This means that the calling
    module does not want to receive this messages anymore. **/
void
carmen_laser_unsubscribe_frontlaser_message(carmen_handler_t handler);

/** Unsubscribes the corresonding message. This means that the calling
    module does not want to receive this messages anymore. **/
void
carmen_laser_unsubscribe_rearlaser_message(carmen_handler_t handler);

/** Unsubscribes the corresonding message. This means that the calling
    module does not want to receive this messages anymore. **/
void
carmen_laser_unsubscribe_laser3_message(carmen_handler_t handler);

/** Unsubscribes the corresonding message. This means that the calling
    module does not want to receive this messages anymore. **/
void
carmen_laser_unsubscribe_laser4_message(carmen_handler_t handler);


/** Unsubscribes the corresonding message. This means that the calling
    module does not want to receive this messages anymore. **/
void
carmen_laser_unsubscribe_laser5_message(carmen_handler_t handler);


/** Unsubscribes the corresonding message. This means that the calling
    module does not want to receive this messages anymore. **/
void
carmen_laser_unsubscribe_alive_message(carmen_handler_t handler);



/** Defines the corresonding message. One module needs to define a
    message before it can be send via IPC. **/
void
carmen_laser_define_frontlaser_message();

/** Defines the corresonding message. One module needs to define a
    message before it can be send via IPC. **/
void
carmen_laser_define_rearlaser_message();

/** Defines the corresonding message. One module needs to define a
    message before it can be send via IPC. **/
void
carmen_laser_define_laser3_message();

/** Defines the corresonding message. One module needs to define a
    message before it can be send via IPC. **/
void
carmen_laser_define_laser4_message();

/** Defines the corresonding message. One module needs to define a
    message before it can be send via IPC. **/
void
carmen_laser_define_laser5_message();

/** Defines the corresonding message. One module needs to define a
    message before it can be send via IPC. **/
void
carmen_laser_define_alive_message();

#ifdef __cplusplus
}
#endif

#endif
// @}
