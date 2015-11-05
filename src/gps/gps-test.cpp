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

int gpgga_update = FALSE;
int gprmc_update = FALSE;
carmen_gps_gpgga_message gps_gpgga;
carmen_gps_gprmc_message gps_gprmc;


void
ipc_gps_gpgga_handler( carmen_gps_gpgga_message *data __attribute__ ((unused)))
{
	gpgga_update = TRUE;
}


void
ipc_gps_gprmc_handler( carmen_gps_gprmc_message *data __attribute__ ((unused)))
{
	gprmc_update = TRUE;
}


void
ipc_update( void )
{
	IPC_listen(0);      
}


void
ipc_init( int argc, char * argv[] )
{
	carmen_ipc_initialize( argc, argv );
	/*************************** SUBSCRIBE ***********************/
	carmen_gps_subscribe_nmea_message( &gps_gpgga,
				     (carmen_handler_t) ipc_gps_gpgga_handler,
				     CARMEN_SUBSCRIBE_LATEST );
	carmen_gps_subscribe_nmea_rmc_message( &gps_gprmc,
				     (carmen_handler_t) ipc_gps_gprmc_handler,
				     CARMEN_SUBSCRIBE_LATEST );
}


int
main( int argc, char *argv[] )
{
	ipc_init(argc, argv );

	while(1)
	{
		ipc_update();
		if (gpgga_update)
		{
			double latitude = 0;
			double longitude = 0;
      
			fprintf( stderr, "===================================\n" );
			fprintf( stderr, "        gps gpgga message\n" );
			fprintf( stderr, "===================================\n" );
			fprintf( stderr, " utc:            %f\n", gps_gpgga.utc );
			fprintf( stderr, " latitude:       %f\n", gps_gpgga.latitude );
			fprintf( stderr, " latitude (DM):  %f\n", gps_gpgga.latitude_dm );
			fprintf( stderr, " lat_orient:     %c\n", gps_gpgga.lat_orient );
			fprintf( stderr, " longitude:      %f\n", gps_gpgga.longitude );
			fprintf( stderr, " longitude (DM): %f\n", gps_gpgga.longitude_dm );
			fprintf( stderr, " long_orient:    %c\n", gps_gpgga.long_orient ); 
			fprintf( stderr, " gps_quality:    %d\n", gps_gpgga.gps_quality );
			fprintf( stderr, " num_satellites: %d\n", gps_gpgga.num_satellites );
			fprintf( stderr, " hdop:           %f\n", gps_gpgga.hdop );  
			fprintf( stderr, " sea_level:      %f\n", gps_gpgga.sea_level ); 
			fprintf( stderr, " altitude:       %f\n", gps_gpgga.altitude );  
			fprintf( stderr, " geo_sea_level:  %f\n", gps_gpgga.geo_sea_level );
			fprintf( stderr, " geo_sep:        %f\n", gps_gpgga.geo_sep ); 
			fprintf( stderr, " data_age:       %d\n", gps_gpgga.data_age );
      
			if (gps_gpgga.lat_orient == 'S') latitude = -gps_gpgga.latitude;
			if (gps_gpgga.long_orient == 'W') longitude = -gps_gpgga.longitude;      
      
			Gdc_Coord_3d gdc = Gdc_Coord_3d(latitude, longitude,  gps_gpgga.sea_level + gps_gpgga.geo_sea_level);
			Utm_Coord_3d utm;
			Gdc_To_Utm_Converter::Init();
			Gdc_To_Utm_Converter::Convert(gdc,utm);
      
			fprintf( stderr, " x:              %f\n", utm.x );    
			fprintf( stderr, " y:              %f\n", utm.y );
			fprintf( stderr, " z:              %f\n", utm.z );
			fprintf( stderr, " timestamp:      %f\n", gps_gpgga.timestamp );
      
			fprintf( stderr, "===================================\n" );
			fprintf( stderr, "\n" );
			gpgga_update = FALSE;
/*
      			// Transformando o timestamp em data
      			time_t now;
      			struct tm ts;
      			char buf[80];
      			now = (time_t)gps_gpgga.timestamp;
      			// Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
      			ts = *localtime(&now);
      			strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &ts);
      			fprintf(stderr, "Data do timestamp = %s\n", buf);
      
  			//    time_t t_of_day = mktime(&ts);
			//  fprintf(stderr, "seconds since the Epoch: %ld\n", (long) t_of_day);

*/
		}
 
		if (gprmc_update)
		{
			fprintf( stderr, "===================================\n" );
			fprintf( stderr, "        gps gprmc message\n" );
			fprintf( stderr, "===================================\n" );
			fprintf( stderr, " validity:          %d\n", gps_gprmc.validity );
			fprintf( stderr, " utc:               %f\n", gps_gprmc.utc );
			fprintf( stderr, " latitude:          %f\n", gps_gprmc.latitude );
			fprintf( stderr, " latitude (DM):     %f\n", gps_gprmc.latitude_dm );
			fprintf( stderr, " lat_orient:        %c\n", gps_gprmc.lat_orient );
			fprintf( stderr, " longitude:         %f\n", gps_gprmc.longitude );
			fprintf( stderr, " longitude (DM):    %f\n", gps_gprmc.longitude_dm );
			fprintf( stderr, " long_orient:       %c\n", gps_gprmc.long_orient ); 
			fprintf( stderr, " speed over ground: %f\n", gps_gprmc.speed );
			fprintf( stderr, " true_course:       %f\n", gps_gprmc.true_course ); 
			fprintf( stderr, " date:              %i\n", gps_gprmc.date );  
			fprintf( stderr, " variation:         %f\n", gps_gprmc.variation );
			fprintf( stderr, " variation dir:     %c\n", gps_gprmc.var_dir ); 
			fprintf( stderr, " timestamp:         %f\n", gps_gprmc.timestamp );
			fprintf( stderr, "===================================\n" );
			fprintf( stderr, "\n" );
			gprmc_update = FALSE;
		}

		usleep(10000);
	}
	return(0);
}


