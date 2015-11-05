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

/** @addtogroup imu libimu_interface **/
// @{

/** \file imu_interface.h
 * \brief Definition of the interface of the module imu.
 *
 * This file specifies the interface to subscribe the messages of
 * that module and to receive its data via ipc.
 **/

#ifndef CARMEN_IMU_INTERFACE_H
#define CARMEN_IMU_INTERFACE_H

#include "imu_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_imu_subscribe_imu_message(carmen_imu_message* msg,
				 carmen_handler_t handler,
				 carmen_subscribe_t subscribe_how);
void
carmen_imu_unsubscribe_imu_message(carmen_handler_t handler);

void
carmen_imu_define_imu_message();


void
carmen_imu_publish_imu_message(carmen_imu_message* msg);


/* alive message */

void
carmen_imu_subscribe_alive_message(carmen_imu_alive_message *alive,
				     carmen_handler_t handler,
				     carmen_subscribe_t subscribe_how);

void
carmen_imu_unsubscribe_alive_message(carmen_handler_t handler);

void
carmen_imu_define_alive_message();

void
carmen_imu_publish_alive_message(int value);


#ifdef __cplusplus
}
#endif

#endif
// @}
