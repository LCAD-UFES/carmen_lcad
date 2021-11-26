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

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdarg.h>
#include <sys/time.h>

#include <carmen/carmen.h>
#include <carmen/gps_nmea_interface.h>


#include "gps.h"

carmen_gps_gpgga_message *carmen_extern_gpgga_ptr = NULL;
carmen_gps_gphdt_message *carmen_extern_gphdt_ptr = NULL;
carmen_gps_gprmc_message *carmen_extern_gprmc_ptr = NULL;


void
ipc_publish_position(void)
{
	static double previous_utc_gpgga = -1.0;
	static double previous_utc_gprmc = -1.0;
	IPC_RETURN_TYPE err = IPC_OK;

	if (carmen_extern_gphdt_ptr != NULL)
	{
		err = IPC_publishData (CARMEN_GPS_GPHDT_MESSAGE_NAME, carmen_extern_gphdt_ptr);
		carmen_test_ipc(err, "Could not publish", CARMEN_GPS_GPHDT_MESSAGE_NAME);
	}

	if (carmen_extern_gpgga_ptr != NULL)
	{
		if (carmen_extern_gpgga_ptr->utc == previous_utc_gpgga) 
			return;
		previous_utc_gpgga = carmen_extern_gpgga_ptr->utc;

		err = IPC_publishData (CARMEN_GPS_GPGGA_MESSAGE_NAME, carmen_extern_gpgga_ptr);
		carmen_test_ipc(err, "Could not publish", CARMEN_GPS_GPGGA_MESSAGE_NAME);

		//fprintf( stderr, "(gga)" );
		//printf("gga lt:% .20lf lg:% .20lf\t", carmen_extern_gpgga_ptr->latitude, carmen_extern_gpgga_ptr->longitude);
	}

	if (carmen_extern_gprmc_ptr != NULL)
	{
		if (carmen_extern_gprmc_ptr->utc == previous_utc_gprmc) 
			return;
		previous_utc_gprmc = carmen_extern_gprmc_ptr->utc;

		err = IPC_publishData (CARMEN_GPS_GPRMC_MESSAGE_NAME, carmen_extern_gprmc_ptr);
		carmen_test_ipc(err, "Could not publish", CARMEN_GPS_GPRMC_MESSAGE_NAME);

		//fprintf( stderr, "(rmc)" );
		//printf("rmc lt:% .20lf lg:% .20lf", carmen_extern_gprmc_ptr->latitude, carmen_extern_gprmc_ptr->longitude);
	}
	
	//printf("\n");
	
}


void
ipc_initialize_messages(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_GPS_GPGGA_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			      CARMEN_GPS_GPGGA_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_GPS_GPGGA_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_GPS_GPRMC_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			      CARMEN_GPS_GPRMC_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_GPS_GPRMC_MESSAGE_NAME);
}

