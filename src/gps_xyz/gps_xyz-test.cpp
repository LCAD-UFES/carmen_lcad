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
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <stdio.h>

#include <carmen/carmen.h>
#include <carmen/carmen_gps.h>

#include <carmen/gps_xyz_interface.h>
#include <carmen/gps_xyz_messages.h>


//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Handlers																						//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////


void
ipc_gps_xyz_handler(carmen_gps_xyz_message *message)
{
	fprintf(stderr, "===================================\n");
	fprintf(stderr, "        gps xyz message\n");
	fprintf(stderr, "===================================\n");
	fprintf(stderr, " nr:               %d\n", message->nr);
	fprintf(stderr, " utc:              %f\n", message->utc);
	fprintf(stderr, " latitude:         %f\n", message->latitude);
	fprintf(stderr, " latitude (DM):    %f\n", message->latitude_dm);
	fprintf(stderr, " lat_orient:       %c\n", message->lat_orient);
	fprintf(stderr, " longitude:        %f\n", message->longitude);
	fprintf(stderr, " longitude (DM):   %f\n", message->longitude_dm);
	fprintf(stderr, " long_orient:      %c\n", message->long_orient);
	fprintf(stderr, " gps_quality:      %d\n", message->gps_quality);
	fprintf(stderr, " num_satellites:   %d\n", message->num_satellites);
	fprintf(stderr, " hdop:             %f\n", message->hdop);
	fprintf(stderr, " sea_level:        %f\n", message->sea_level);
	fprintf(stderr, " altitude:         %f\n", message->altitude);
	fprintf(stderr, " geo_sea_level:    %f\n", message->geo_sea_level);
	fprintf(stderr, " geo_sep:          %f\n", message->geo_sep);
	fprintf(stderr, " data_age:         %d\n", message->data_age);
	fprintf(stderr, " x:                %f\n", message->x);
	fprintf(stderr, " y:                %f\n", message->y);
	fprintf(stderr, " z:                %f\n", message->z);
	fprintf(stderr, " zone:             %f\n", message->zone);
	fprintf(stderr, " hemisphere_north: %d\n\n", message->hemisphere_north);

	//Transformando novamente em latitude e longitude
	Gdc_Coord_3d teste = carmen_Utm_Gdc2(-message->y, message->x, message->z, message->zone, message->hemisphere_north);
	fprintf(stderr, "Transformando novamente em latitude, longitude e altitude:\n");
	fprintf(stderr, "(%f, %f, %f)\n\n", teste.latitude, teste.longitude, teste.elevation);

	fprintf(stderr, "===================================\n");
	fprintf(stderr, "\n");
}


void
ipc_gps_gphdt_handler(carmen_gps_gphdt_message *message)
{
	fprintf(stderr, "===================================\n");
	fprintf(stderr, "        gps gphdt message\n" );
	fprintf(stderr, "===================================\n");
	fprintf(stderr, " gps number:        %d\n", message->nr);
	fprintf(stderr, " valid:             %d\n", message->valid);
	fprintf(stderr, " heading:           %f\n", message->heading);
	fprintf(stderr, " timestamp:         %f\n", message->timestamp);
	fprintf(stderr, "===================================\n");
	fprintf(stderr, "\n");
}


void
shutdown_module(int signal __attribute__ ((unused)))
{
	static int done = 0;

	if (!done)
	{
		carmen_ipc_disconnect();
		printf("Disconnected from IPC.\n");
		done = 1;
	}

	exit(0);
}
//////////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc, char *argv[])
{
	signal(SIGINT, shutdown_module);

	carmen_ipc_initialize(argc, argv);
	carmen_gps_xyz_subscribe_message(NULL, (carmen_handler_t) ipc_gps_xyz_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_gps_subscribe_nmea_hdt_message(NULL, (carmen_handler_t) ipc_gps_gphdt_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();

	return(0);
}

