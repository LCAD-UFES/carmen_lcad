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


int gps_xyz_update = 0;
carmen_gps_xyz_message gps_xyz;


void
ipc_gps_xyz_handler(carmen_gps_xyz_message *data __attribute__ ((unused)))
{
	gps_xyz_update++;
}


int
main(int argc, char *argv[])
{
	carmen_ipc_initialize(argc, argv);
	carmen_gps_xyz_subscribe_message(&gps_xyz, (carmen_handler_t) ipc_gps_xyz_handler, CARMEN_SUBSCRIBE_LATEST);

	while (1)
	{
		IPC_listen(0);
		if (gps_xyz_update != 0)
		{
			fprintf(stderr, "===================================\n");
			fprintf(stderr, "        gps xyz message\n");
			fprintf(stderr, "===================================\n");
			fprintf(stderr, " nr:               %d\n", gps_xyz.nr);
			fprintf(stderr, " utc:              %f\n", gps_xyz.utc);
			fprintf(stderr, " latitude:         %f\n", gps_xyz.latitude);
			fprintf(stderr, " latitude (DM):    %f\n", gps_xyz.latitude_dm);
			fprintf(stderr, " lat_orient:       %c\n", gps_xyz.lat_orient);
			fprintf(stderr, " longitude:        %f\n", gps_xyz.longitude);
			fprintf(stderr, " longitude (DM):   %f\n", gps_xyz.longitude_dm);
			fprintf(stderr, " long_orient:      %c\n", gps_xyz.long_orient);
			fprintf(stderr, " gps_quality:      %d\n", gps_xyz.gps_quality);
			fprintf(stderr, " num_satellites:   %d\n", gps_xyz.num_satellites);
			fprintf(stderr, " hdop:             %f\n", gps_xyz.hdop);
			fprintf(stderr, " sea_level:        %f\n", gps_xyz.sea_level);
			fprintf(stderr, " altitude:         %f\n", gps_xyz.altitude);
			fprintf(stderr, " geo_sea_level:    %f\n", gps_xyz.geo_sea_level);
			fprintf(stderr, " geo_sep:          %f\n", gps_xyz.geo_sep);
			fprintf(stderr, " data_age:         %d\n", gps_xyz.data_age);
			fprintf(stderr, " x:                %f\n", gps_xyz.x);
			fprintf(stderr, " y:                %f\n", gps_xyz.y);
			fprintf(stderr, " z:                %f\n", gps_xyz.z);
			fprintf(stderr, " zone:             %f\n", gps_xyz.zone);
			fprintf(stderr, " hemisphere_north: %d\n\n", gps_xyz.hemisphere_north);

			//Transformando novamente em latitude e longitude
			Gdc_Coord_3d teste = carmen_Utm_Gdc2(-gps_xyz.y, gps_xyz.x, gps_xyz.z, gps_xyz.zone, gps_xyz.hemisphere_north);
			fprintf(stderr, "Transformando novamente em latitude, longitude e altitude:\n");
			fprintf(stderr, "(%f, %f, %f)\n\n", teste.latitude, teste.longitude, teste.elevation);

			fprintf(stderr, "===================================\n");
			fprintf(stderr, "\n");

			gps_xyz_update = 0;
		}
		usleep(10000);
	}
	return(0);
}

