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


/** @addtogroup gps libgps_interface **/
// @{

/** \file gps_nmea_interface.h
 * \brief Definition of the interface of the module gps.
 *
 * This file specifies the interface to subscribe the messages of
 * that module and to receive its data via ipc.
 **/

#ifndef GPS_XYZ_INTERFACE_H
#define GPS_XYZ_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/gps_xyz_messages.h>

void
carmen_gps_xyz_subscribe_message(carmen_gps_xyz_message *gps_xyz,
				 carmen_handler_t handler,
				 carmen_subscribe_t subscribe_how);

void
carmen_gps_xyz_unsubscribe_message(carmen_handler_t handler);


void
carmen_gps_xyz_define_message();


void
carmen_xsens_xyz_subscribe_message(carmen_xsens_xyz_message *xsens_xyz,
				 carmen_handler_t handler,
				 carmen_subscribe_t subscribe_how);

void
carmen_xsens_xyz_unsubscribe_message(carmen_handler_t handler);


void
carmen_xsens_xyz_define_message();

void
carmen_velodyne_gps_xyz_subscribe_message(carmen_velodyne_gps_xyz_message *velodyne_gps_xyz,
				  carmen_handler_t handler,
				  carmen_subscribe_t subscribe_how);


void
carmen_velodyne_gps_xyz_unsubscribe_message(carmen_handler_t handler);


void
carmen_velodyne_gps_xyz_define_message();
              
#ifdef __cplusplus
}
#endif

#endif

//@}
