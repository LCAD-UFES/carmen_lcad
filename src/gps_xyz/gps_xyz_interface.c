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

#include <carmen/carmen.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/gps_xyz_messages.h>


void
carmen_gps_xyz_subscribe_message(carmen_gps_xyz_message *gps_xyz,
				  carmen_handler_t handler,
				  carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_GPS_XYZ_MESSAGE_NAME, 
				   CARMEN_GPS_XYZ_MESSAGE_FMT,
				   gps_xyz, sizeof(carmen_gps_xyz_message), handler,
				   subscribe_how);
}


void
carmen_gps_xyz_unsubscribe_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_GPS_XYZ_MESSAGE_NAME, handler);
}


void
carmen_gps_xyz_define_message()
{
	IPC_RETURN_TYPE err;
  
	err = IPC_defineMsg(CARMEN_GPS_XYZ_MESSAGE_NAME,IPC_VARIABLE_LENGTH, 
			      CARMEN_GPS_XYZ_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_GPS_XYZ_MESSAGE_NAME);
}


void
carmen_xsens_xyz_subscribe_message(carmen_xsens_xyz_message *xsens_xyz,
				  carmen_handler_t handler,
				  carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_XSENS_XYZ_MESSAGE_NAME, 
				   CARMEN_XSENS_XYZ_MESSAGE_FMT,
				   xsens_xyz, sizeof(carmen_xsens_xyz_message), handler,
				   subscribe_how);
}


void
carmen_xsens_xyz_unsubscribe_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_XSENS_XYZ_MESSAGE_NAME, handler);
}


void
carmen_xsens_xyz_define_message()
{
	IPC_RETURN_TYPE err;
  
	err = IPC_defineMsg(CARMEN_XSENS_XYZ_MESSAGE_NAME,IPC_VARIABLE_LENGTH, 
			      CARMEN_XSENS_XYZ_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_XSENS_XYZ_MESSAGE_NAME);
}


void
carmen_velodyne_gps_xyz_subscribe_message(carmen_velodyne_gps_xyz_message *velodyne_gps_xyz,
				  carmen_handler_t handler,
				  carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_VELODYNE_GPS_XYZ_MESSAGE_NAME,
				   CARMEN_VELODYNE_GPS_XYZ_MESSAGE_FMT,
				   velodyne_gps_xyz, sizeof(carmen_velodyne_gps_xyz_message), handler,
				   subscribe_how);
}


void
carmen_velodyne_gps_xyz_unsubscribe_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_VELODYNE_GPS_XYZ_MESSAGE_NAME, handler);
}


void
carmen_velodyne_gps_xyz_define_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_VELODYNE_GPS_XYZ_MESSAGE_NAME,IPC_VARIABLE_LENGTH,
			CARMEN_VELODYNE_GPS_XYZ_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_VELODYNE_GPS_XYZ_MESSAGE_NAME);
}


