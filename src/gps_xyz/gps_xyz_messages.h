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


/** @addtogroup gps **/
// @{

/** \file gps_nmea_messages.h
 * \brief Definition of the messages for this module.
 *
 * This file specifies the messages for this modules used to transmit
 * data via ipc to other modules.
 **/


#ifndef GPS_XYZ_MESSAGES_H
#define GPS_XYZ_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif


typedef struct 
{
	int nr; /**< number of the gps unit **/
	double utc; /**< Universal Time Coordinated (UTC) **/
	double latitude; /**< latitude as a real floating point number **/
	double latitude_dm; /**< latitude as DDMM.MMMM format (as in the NMEA text message) **/
	char lat_orient; /**< N or S (North or South) **/
	double longitude; /**< longitude as a real floating point number **/
	double longitude_dm; /**< longitude as DDMM.MMMM format (as in the NMEA text message) **/
	char long_orient; /**< E or W (East or West) **/
	int gps_quality; /**< GPS Quality Indicator,
		         0 - fix not available,
		         1 - GPS fix,
		         2 - Differential GPS fix
				 3 - ? (it is not in the Trimble documentation in http://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_GGA.html
				 4 - Real-Time Kinematic, fixed integers
				 5 - Real-Time Kinematic, float integers, OmniSTAR XP/HP or Location RTK **/
	int num_satellites; /**< Number of satellites in view, 00-12 **/
	double hdop; /**< Horizontal Dilution of precision **/
	double sea_level; /**< Antenna Altitude above/below 
			 mean-sea-level (geoid) **/
	double altitude; /**< Units of antenna altitude, meters **/
	double geo_sea_level; /**< Geoidal separation, the difference
				 between the WGS-84 earth ellipsoid and
				 mean-sea-level (geoid), "-" means
				 mean-sea-level below ellipsoid **/
	double geo_sep; /**< Units of geoidal separation, meters **/
	int data_age; /**< Age of differential GPS data, time 
			in seconds since last SC104 type 1 or
			9 update, null field when DGPS is not
			used **/
	double timestamp;
	char* host;
	double x;
	double y;
	double z;
	double zone;
	int hemisphere_north; // true == north false == south
} carmen_gps_xyz_message;


#define CARMEN_GPS_XYZ_MESSAGE_FMT "{int,double,double,double,char,double,double,char,int,int,double,double,double,double,double,int,double,string,double,double,double,double,int}"
#define CARMEN_GPS_XYZ_MESSAGE_NAME "carmen_gps_xyz"

typedef struct
{
	carmen_quaternion_t quat;
	carmen_vector_3D_t acc;
	carmen_vector_3D_t gyr;
	carmen_vector_3D_t mag;

	carmen_vector_3D_t velocity;
	carmen_vector_3D_t position;	

	int gps_fix;
	int xkf_valid;	

	int sensor_ID;

	double timestamp;
	char *host;
} carmen_xsens_xyz_message;

#define CARMEN_XSENS_XYZ_MESSAGE_FMT "{{double,double,double,double},{double,double,double},{double,double,double},{double,double,double},{double,double,double},{double,double,double},int,int,int,double,string}"
#define CARMEN_XSENS_XYZ_MESSAGE_NAME "carmen_xsens_xyz"


typedef struct
{
	float gyro1, gyro2, gyro3;
	float temp1, temp2, temp3;
	float accel1_x, accel2_x, accel3_x;
	float accel1_y, accel2_y, accel3_y;

	int utc_time;													// hhmmss format
	char status;													// A = valid position, V = NAV receiver warning
	float latitude; 											// ddmm.mmm format
	char latitude_hemisphere;							// N or S
	float longitude;											// ddmm.mm format
	char longitude_hemisphere;						// E or W
	float speed_over_ground;							// 000.0 to 999.9 knots
	float course_over_ground;						// 000.0 to 39.9 degrees
	int utc_date;													// ddmmyy format
	float magnetic_variation_course;		// 000.0 to 180.0 degrees
	float magnetic_variation_direction; // E or W
	char mode_indication;				// A = Autonomous, D = Differential, E = Estimated, N =m Data not valid

	carmen_vector_3D_t position;

	double timestamp;
	char* host;
} carmen_velodyne_gps_xyz_message;

#define      CARMEN_VELODYNE_GPS_XYZ_MESSAGE_NAME       "carmen_velodyne_gps_xyz_message"
#define      CARMEN_VELODYNE_GPS_XYZ_MESSAGE_FMT        "{float, float, float, float, float, float, float, float, float, float, float, float, int, byte, float, byte, float, byte, float, float, int, float, float, byte, {double,double,double}, double, string}"


#ifdef __cplusplus
}
#endif

#endif

// @}
