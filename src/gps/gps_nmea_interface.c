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
#include <carmen/gps_nmea_interface.h>

void
carmen_gps_subscribe_nmea_message(carmen_gps_gpgga_message *nmea,
				  carmen_handler_t handler,
				  carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_GPS_GPGGA_MESSAGE_NAME, 
			   CARMEN_GPS_GPGGA_MESSAGE_FMT,
			   nmea, sizeof(carmen_gps_gpgga_message), handler,
			   subscribe_how);
}


void
carmen_gps_subscribe_nmea_rmc_message(carmen_gps_gprmc_message *nmea,
				  carmen_handler_t handler,
				  carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_GPS_GPRMC_MESSAGE_NAME, 
			   CARMEN_GPS_GPRMC_MESSAGE_FMT,
			   nmea, sizeof(carmen_gps_gprmc_message), handler,
			   subscribe_how);
}

