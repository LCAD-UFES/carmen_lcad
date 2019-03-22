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

#include <vector>


#define GPS_MESSAGE_QUEUE_SIZE 20
#define GPS_REACH1 2
#define GPS_REACH2 3
#define SMALL_DELTA_T 0.1
#define REFERENCE_ANGLE (M_PI / 2.0)

using namespace std;


vector<carmen_gps_xyz_message> gps_xyz_message_queue;


double
get_angle_between_gpss(carmen_gps_xyz_message reach2, carmen_gps_xyz_message reach1)
{
	double angle = atan2(reach1.y - reach2.y, reach1.x - reach2.x) + REFERENCE_ANGLE;
	angle = carmen_normalize_theta(angle);

	return (angle);
}


bool
get_carmen_gps_gphdt_message(vector<carmen_gps_xyz_message> gps_xyz_message_queue, carmen_gps_gphdt_message *carmen_extern_gphdt_ptr)
{
	double angle = 1000.0;
	int i;

	for (i = gps_xyz_message_queue.size() - 1; i > 0; i--)
	{
		if (gps_xyz_message_queue[i].nr == GPS_REACH1)
		{
			for (int j = gps_xyz_message_queue.size() - 1; j > 0; j--)
			{
				if ((gps_xyz_message_queue[j].nr == GPS_REACH2) && (fabs(gps_xyz_message_queue[j].utc - gps_xyz_message_queue[i].utc) < SMALL_DELTA_T))
					angle = get_angle_between_gpss(gps_xyz_message_queue[j], gps_xyz_message_queue[i]);
			}
			if (angle != 1000.0)
				break;
		}
	}

	if (angle != 1000.0)
	{
		carmen_extern_gphdt_ptr->heading = angle;
		carmen_extern_gphdt_ptr->host = gps_xyz_message_queue[i].host;
		carmen_extern_gphdt_ptr->nr = gps_xyz_message_queue[i].nr;
		carmen_extern_gphdt_ptr->timestamp = gps_xyz_message_queue[i].timestamp;
		carmen_extern_gphdt_ptr->valid = 1;

		return (true);
	}
	else
		return (false);
}
//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Publishers                                                                                   //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_gps_xyz_publish_message(carmen_gps_xyz_message gps_xyz_message)
{
	IPC_RETURN_TYPE err = IPC_OK;

	err = IPC_publishData(CARMEN_GPS_XYZ_MESSAGE_NAME, &gps_xyz_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_GPS_XYZ_MESSAGE_NAME);
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
publish_carmen_gps_gphdt_message(carmen_gps_gphdt_message *carmen_extern_gphdt_ptr)
{
	IPC_RETURN_TYPE err = IPC_OK;

	if (carmen_extern_gphdt_ptr != NULL)
	{
		err = IPC_publishData (CARMEN_GPS_GPHDT_MESSAGE_NAME, carmen_extern_gphdt_ptr);
		carmen_test_ipc(err, "Could not publish", CARMEN_GPS_GPHDT_MESSAGE_NAME);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Handlers                                                                                     //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_gps_gpgga_message_handler(carmen_gps_gpgga_message *gps_gpgga)
{
	double latitude = 0;
	double longitude = 0;
	carmen_gps_xyz_message gps_xyz_message;

//	static double previous_utc = -1.0;
//	static double previous_timestamp = -1.0;

//	if (gps_gpgga->utc == previous_utc)
//		return;

//	previous_utc = gps_gpgga->utc;
	gps_xyz_message.nr = gps_gpgga->nr;
	gps_xyz_message.utc = gps_gpgga->utc;
	gps_xyz_message.latitude = gps_gpgga->latitude;
	gps_xyz_message.latitude_dm = gps_gpgga->latitude_dm;
	gps_xyz_message.lat_orient = gps_gpgga->lat_orient;
	gps_xyz_message.longitude = gps_gpgga->longitude;
	gps_xyz_message.longitude_dm = gps_gpgga->longitude_dm;
	gps_xyz_message.long_orient = gps_gpgga->long_orient;
	gps_xyz_message.gps_quality = gps_gpgga->gps_quality;
	gps_xyz_message.num_satellites = gps_gpgga->num_satellites;
	gps_xyz_message.hdop = gps_gpgga->hdop;
	gps_xyz_message.sea_level = gps_gpgga->sea_level;
	gps_xyz_message.altitude = gps_gpgga->altitude;
	gps_xyz_message.geo_sea_level = gps_gpgga->geo_sea_level;
	gps_xyz_message.geo_sep = gps_gpgga->geo_sep;
	gps_xyz_message.data_age = gps_gpgga->data_age;

	if (gps_gpgga->lat_orient == 'S') latitude = -gps_gpgga->latitude;
	if (gps_gpgga->long_orient == 'W') longitude = -gps_gpgga->longitude;

// Transformando o z utilizando como altitude o sea_level
	Gdc_Coord_3d gdc = Gdc_Coord_3d(latitude, longitude, gps_gpgga->sea_level);

// Transformando o z utilizando como altitude a altitude mesmo - que esta vindo como zero
	//Gdc_Coord_3d gdc = Gdc_Coord_3d(latitude, longitude, gps_gpgga->altitude);
	Utm_Coord_3d utm;
	Gdc_To_Utm_Converter::Init();
	Gdc_To_Utm_Converter::Convert(gdc,utm);

	//gps_xyz_message.x = utm.x;
	//gps_xyz_message.y = utm.y;
	//gps_xyz_message.z = utm.z;

	gps_xyz_message.x = utm.y;
	gps_xyz_message.y = -utm.x;
	
	// *********************************************************************************************************************************************
	// @@@ Alberto: ATENCAO! ATENCAO! ATENCAO: estou zerando o z do gps que fiz pois ele varia muito. Quando tivermos um bom tem que parar de zerar!
	// gps_xyz_message.z = utm.z;
	// *********************************************************************************************************************************************
	gps_xyz_message.z = 0.0;
	gps_xyz_message.zone = (double)utm.zone;

	//printf("%lf %lf -> %lf %lf %lf\n", latitude, longitude, gps_xyz_message.x, gps_xyz_message.y, gps_xyz_message.z);

	if (utm.hemisphere_north == true)
		gps_xyz_message.hemisphere_north = 1;
	else
		gps_xyz_message.hemisphere_north = 0;

	gps_xyz_message.timestamp = gps_gpgga->timestamp;
	gps_xyz_message.host = carmen_get_host();

	carmen_gps_xyz_publish_message(gps_xyz_message);


	if ((gps_xyz_message.nr == GPS_REACH1) || (gps_xyz_message.nr == GPS_REACH2))
		gps_xyz_message_queue.push_back(gps_xyz_message);

	if (gps_xyz_message_queue.size() > GPS_MESSAGE_QUEUE_SIZE)
		gps_xyz_message_queue.erase(gps_xyz_message_queue.begin());

	carmen_gps_gphdt_message carmen_extern_gphdt;
	if (get_carmen_gps_gphdt_message(gps_xyz_message_queue, &carmen_extern_gphdt))
		publish_carmen_gps_gphdt_message(&carmen_extern_gphdt);
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
//////////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc, char *argv[])
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_module);

	carmen_gps_xyz_define_message();
	carmen_xsens_xyz_define_message();
	carmen_velodyne_gps_xyz_define_message();

	carmen_gps_subscribe_nmea_message(NULL, (carmen_handler_t) carmen_gps_gpgga_message_handler, CARMEN_SUBSCRIBE_ALL);
	carmen_xsens_mtig_subscribe_message(NULL, (carmen_handler_t) xsens_mtig_handler, CARMEN_SUBSCRIBE_ALL);
	carmen_velodyne_subscribe_gps_message(NULL, (carmen_handler_t) velodyne_gps_handler, CARMEN_SUBSCRIBE_ALL);

	/* Loop forever waiting for messages */
	carmen_ipc_dispatch();

	return(0);
}

