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
#include <carmen/gps_nmea_interface.h>


//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Handlers																						//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////


void
ipc_gps_gpgga_handler(carmen_gps_gpgga_message *message)
{
	fprintf(stderr, "===================================\n");
	fprintf(stderr, "        gps gpgga message\n" );
	fprintf(stderr, "===================================\n");
	fprintf(stderr, " gps number:     %d\n", message->nr);
	fprintf(stderr, " utc:            %lf\n", message->utc);
	fprintf(stderr, " latitude:       %.9lf\n", message->latitude);
	fprintf(stderr, " latitude (DM):  %.7lf\n", message->latitude_dm);
	fprintf(stderr, " lat_orient:     %c\n", message->lat_orient);
	fprintf(stderr, " longitude:      %.9lf\n", message->longitude);
	fprintf(stderr, " longitude (DM): %.7lf\n", message->longitude_dm);
	fprintf(stderr, " long_orient:    %c\n", message->long_orient);
	fprintf(stderr, " gps_quality:    %d\n", message->gps_quality);
	fprintf(stderr, " num_satellites: %d\n", message->num_satellites);
	fprintf(stderr, " hdop:           %lf\n", message->hdop);
	fprintf(stderr, " sea_level:      %lf\n", message->sea_level);
	fprintf(stderr, " altitude:       %lf\n", message->altitude);
	fprintf(stderr, " geo_sea_level:  %lf\n", message->geo_sea_level);
	fprintf(stderr, " geo_sep:        %lf\n", message->geo_sep);
	fprintf(stderr, " data_age:       %d\n", message->data_age);

	double latitude = 0;
	double longitude = 0;
	if (message->lat_orient == 'S')
		latitude = -message->latitude;
	if (message->long_orient == 'W')
		longitude = -message->longitude;

	Gdc_Coord_3d gdc = Gdc_Coord_3d(latitude, longitude,  message->sea_level + message->geo_sea_level);
	Utm_Coord_3d utm;
	Gdc_To_Utm_Converter::Init();
	Gdc_To_Utm_Converter::Convert(gdc,utm);

	fprintf(stderr, " x:              %.3lf\n", utm.x);
	fprintf(stderr, " y:              %.3lf\n", utm.y);
	fprintf(stderr, " z:              %.3lf\n", utm.z);
	fprintf(stderr, " timestamp:      %lf\n", message->timestamp);

	fprintf(stderr, "===================================\n");
	fprintf(stderr, "\n");
	fflush(stderr);
}


void
ipc_gps_gphdt_handler(carmen_gps_gphdt_message *message)
{
	fprintf(stderr, "===================================\n");
	fprintf(stderr, "        gps gphdt message\n" );
	fprintf(stderr, "===================================\n");
	fprintf(stderr, " gps number:        %d\n", message->nr);
	fprintf(stderr, " valid:             %d\n", message->valid);
	fprintf(stderr, " heading:           %lf\n", message->heading);
	fprintf(stderr, " timestamp:         %lf\n", message->timestamp);
	fprintf(stderr, "===================================\n");
	fprintf(stderr, "\n\n");
	fflush(stderr);
}


void
ipc_gps_gprmc_handler(carmen_gps_gprmc_message *message)
{
	fprintf(stderr, "===================================\n");
	fprintf(stderr, "        gps gprmc message\n" );
	fprintf(stderr, "===================================\n");
	fprintf(stderr, " gps number:        %d\n", message->nr);
	fprintf(stderr, " validity:          %d\n", message->validity);
	fprintf(stderr, " utc:               %lf\n", message->utc);
	fprintf(stderr, " latitude:          %.9lf\n", message->latitude);
	fprintf(stderr, " latitude (DM):     %.7lf\n", message->latitude_dm);
	fprintf(stderr, " lat_orient:        %c\n", message->lat_orient);
	fprintf(stderr, " longitude:         %.9lf\n", message->longitude);
	fprintf(stderr, " longitude (DM):    %.7lf\n", message->longitude_dm);
	fprintf(stderr, " long_orient:       %c\n", message->long_orient);
	fprintf(stderr, " speed over ground: %lf\n", message->speed);
	fprintf(stderr, " true_course:       %lf\n", message->true_course);
	fprintf(stderr, " date:              %i\n", message->date);
	fprintf(stderr, " variation:         %lf\n", message->variation);
	fprintf(stderr, " variation dir:     %c\n", message->var_dir);
	fprintf(stderr, " timestamp:         %lf\n", message->timestamp);
	fprintf(stderr, "===================================\n");
	fprintf(stderr, "\n");
	fflush(stderr);
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


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Inicializations																				//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////


void
subscribe_to_relevant_messages()
{
	carmen_gps_subscribe_nmea_message(NULL, (carmen_handler_t) ipc_gps_gpgga_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_gps_subscribe_nmea_hdt_message(NULL, (carmen_handler_t) ipc_gps_gphdt_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_gps_subscribe_nmea_rmc_message(NULL, (carmen_handler_t) ipc_gps_gprmc_handler, CARMEN_SUBSCRIBE_LATEST);
}
//////////////////////////////////////////////////////////////////////////////////////////////////


int
main( int argc, char *argv[] )
{
	carmen_ipc_initialize(argc, argv);
	subscribe_to_relevant_messages();

	carmen_ipc_dispatch();

	return(0);
}


