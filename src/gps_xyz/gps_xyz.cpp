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
#include <carmen/gps_xyz_messages.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/xsens_mtig_interface.h>

static carmen_gps_xyz_message msg;


void
carmen_gps_xyz_publish_message(carmen_gps_gpgga_message *gps_gpgga)
{
	IPC_RETURN_TYPE err = IPC_OK;
	double latitude = 0;
	double longitude = 0;
	static double previous_utc = -1.0;
	//static double previous_timestamp = -1.0;

	if (gps_gpgga->utc == previous_utc)
		return;

	previous_utc = gps_gpgga->utc;
	msg.utc = gps_gpgga->utc;
	msg.latitude = gps_gpgga->latitude;
	msg.latitude_dm = gps_gpgga->latitude_dm;
	msg.lat_orient = gps_gpgga->lat_orient;
	msg.longitude = gps_gpgga->longitude;
	msg.longitude_dm = gps_gpgga->longitude_dm;
	msg.long_orient = gps_gpgga->long_orient;
	msg.gps_quality = gps_gpgga->gps_quality;
	msg.num_satellites = gps_gpgga->num_satellites;
	msg.hdop = gps_gpgga->hdop;
	msg.sea_level = gps_gpgga->sea_level;
	msg.altitude = gps_gpgga->altitude;
	msg.geo_sea_level = gps_gpgga->geo_sea_level;
	msg.geo_sep = gps_gpgga->geo_sep;
	msg.data_age = gps_gpgga->data_age;

	if (gps_gpgga->lat_orient == 'S') latitude = -gps_gpgga->latitude;
	if (gps_gpgga->long_orient == 'W') longitude = -gps_gpgga->longitude;

// Transformando o z utilizando como altitude o sea_level
	Gdc_Coord_3d gdc = Gdc_Coord_3d(latitude, longitude, gps_gpgga->sea_level);

// Transformando o z utilizando como altitude a altitude mesmo - que esta vindo como zero
	//Gdc_Coord_3d gdc = Gdc_Coord_3d(latitude, longitude, gps_gpgga->altitude);
	Utm_Coord_3d utm;
	Gdc_To_Utm_Converter::Init();
	Gdc_To_Utm_Converter::Convert(gdc,utm);

	//msg.x = utm.x;
	//msg.y = utm.y;
	//msg.z = utm.z;

	msg.x = utm.y;
	msg.y = -utm.x;
	
	// *********************************************************************************************************************************************
	// @@@ Alberto: ATENCAO! ATENCAO! ATENCAO: estou zerando o z do gps que fiz pois ele varia muito. Quando tivermos um bom tem que parar de zerar!
	// msg.z = utm.z;
	// *********************************************************************************************************************************************
	msg.z = 0.0;
	msg.zone = (double)utm.zone;

	//printf("%lf %lf -> %lf %lf %lf\n", latitude, longitude, msg.x, msg.y, msg.z);

	if (utm.hemisphere_north == true)
		msg.hemisphere_north = 1;
	else
		msg.hemisphere_north = 0;

	msg.timestamp = gps_gpgga->timestamp;
	msg.host = carmen_get_host();

	err = IPC_publishData(CARMEN_GPS_XYZ_MESSAGE_NAME, &msg);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_GPS_XYZ_MESSAGE_NAME);
}


void
ipc_gps_gpgga_handler(carmen_gps_gpgga_message *gps_gpgga)
{
	carmen_gps_xyz_publish_message(gps_gpgga);
}


static void
publish_xsens_xyz_message(carmen_xsens_xyz_message xsens_xyz_message)
{
	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_XSENS_XYZ_MESSAGE_NAME, &xsens_xyz_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_XSENS_XYZ_MESSAGE_NAME);
}

static void
publish_velodyne_gps_xyz_message(carmen_velodyne_gps_xyz_message velodyne_gps_xyz_message)
{
	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_VELODYNE_GPS_XYZ_MESSAGE_NAME, &velodyne_gps_xyz_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VELODYNE_GPS_XYZ_MESSAGE_NAME);
}


static void
xsens_mtig_handler(carmen_xsens_mtig_message *message)
{
	carmen_xsens_xyz_message xsens_xyz_message;

	xsens_xyz_message.quat = message->quat;
	xsens_xyz_message.acc = message->acc;
	xsens_xyz_message.gyr = message->gyr;
	xsens_xyz_message.mag = message->mag;
	xsens_xyz_message.velocity = message->velocity;

	Gdc_Coord_3d gdc = Gdc_Coord_3d(message->latitude, message->longitude, message->height);
	Utm_Coord_3d utm;
	Gdc_To_Utm_Converter::Init();
	Gdc_To_Utm_Converter::Convert(gdc,utm);

	// The axis are changed to match the carmen frame of reference
	xsens_xyz_message.position.x = utm.y;
	xsens_xyz_message.position.y = -utm.x;
	xsens_xyz_message.position.z = utm.z;

	xsens_xyz_message.gps_fix = message->gps_fix;
	xsens_xyz_message.xkf_valid = message->xkf_valid;
	xsens_xyz_message.sensor_ID = message->sensor_ID;
	xsens_xyz_message.timestamp = message->timestamp;
	xsens_xyz_message.host = carmen_get_host();

	publish_xsens_xyz_message(xsens_xyz_message);
}

void
velodyne_gps_handler(carmen_velodyne_gps_message *message)
{
	carmen_velodyne_gps_xyz_message velodyne_gps_xyz_message;

	velodyne_gps_xyz_message.accel1_x = message->accel1_x;
	velodyne_gps_xyz_message.accel1_y = message->accel1_y;
	velodyne_gps_xyz_message.accel2_x = message->accel2_x;
	velodyne_gps_xyz_message.accel2_y = message->accel2_y;
	velodyne_gps_xyz_message.accel3_x = message->accel3_x;
	velodyne_gps_xyz_message.accel3_y = message->accel3_y;

	velodyne_gps_xyz_message.gyro1 = message->gyro1;
	velodyne_gps_xyz_message.gyro2 = message->gyro2;
	velodyne_gps_xyz_message.gyro3 = message->gyro3;

	velodyne_gps_xyz_message.temp1 = message->temp1;
	velodyne_gps_xyz_message.temp2 = message->temp2;
	velodyne_gps_xyz_message.temp3 = message->temp3;

	velodyne_gps_xyz_message.utc_time = message->utc_time;
	velodyne_gps_xyz_message.utc_date = message->utc_date;
	velodyne_gps_xyz_message.status = message->status;

	velodyne_gps_xyz_message.latitude = carmen_global_convert_degmin_to_double(message->latitude);
	velodyne_gps_xyz_message.latitude_hemisphere = message->latitude_hemisphere;
	velodyne_gps_xyz_message.longitude = carmen_global_convert_degmin_to_double(message->longitude);
	velodyne_gps_xyz_message.longitude_hemisphere = message->longitude_hemisphere;
	velodyne_gps_xyz_message.speed_over_ground = message->speed_over_ground;
	velodyne_gps_xyz_message.course_over_ground = message->course_over_ground;
	velodyne_gps_xyz_message.magnetic_variation_course = message->magnetic_variation_course;
	velodyne_gps_xyz_message.magnetic_variation_direction = message->magnetic_variation_direction;
	velodyne_gps_xyz_message.mode_indication = message->mode_indication;
	velodyne_gps_xyz_message.timestamp = message->timestamp;

	if (velodyne_gps_xyz_message.latitude_hemisphere == 'S')
		velodyne_gps_xyz_message.latitude = -velodyne_gps_xyz_message.latitude;
	if (velodyne_gps_xyz_message.longitude_hemisphere == 'W')
		velodyne_gps_xyz_message.longitude = -velodyne_gps_xyz_message.longitude;


	// Transformando o z utilizando como altitude o sea_level

	Gdc_Coord_3d gdc = Gdc_Coord_3d((double) velodyne_gps_xyz_message.latitude, (double)velodyne_gps_xyz_message.longitude, 0.0);
	Utm_Coord_3d utm;
	Gdc_To_Utm_Converter::Init();
	Gdc_To_Utm_Converter::Convert(gdc,utm);

	// The axis are changed to match the carmen frame of reference
	velodyne_gps_xyz_message.position.x = utm.y;
	velodyne_gps_xyz_message.position.y = -utm.x;
	velodyne_gps_xyz_message.position.z = utm.z;

	publish_velodyne_gps_xyz_message(velodyne_gps_xyz_message);
}


void
shutdown_module(int signo)
{
		carmen_ipc_disconnect();
	if (signo == SIGINT)
	{
		printf("gps_xyz: disconnected.\n");
		exit(0);
	}
}


int
main( int argc, char *argv[] )
{
	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);

	carmen_gps_xyz_define_message();
	carmen_xsens_xyz_define_message();
	carmen_velodyne_gps_xyz_define_message();

	/* Subscribe to sensor messages */
	carmen_gps_subscribe_nmea_message(NULL, (carmen_handler_t) ipc_gps_gpgga_handler, CARMEN_SUBSCRIBE_ALL);
	carmen_xsens_mtig_subscribe_message(NULL, (carmen_handler_t) xsens_mtig_handler, CARMEN_SUBSCRIBE_ALL);
	carmen_velodyne_subscribe_gps_message(NULL, (carmen_handler_t) velodyne_gps_handler, CARMEN_SUBSCRIBE_ALL);

	/* Loop forever waiting for messages */
	carmen_ipc_dispatch();

	return(0);
}

