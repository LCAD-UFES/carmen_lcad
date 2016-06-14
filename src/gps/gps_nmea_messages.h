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


#ifndef GPS_NMEA_MESSAGES_H
#define GPS_NMEA_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
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
		         2 - Differential GPS fix, OmniSTAR VBS
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
	char *host;
} carmen_gps_gpgga_message;

#define CARMEN_GPS_GPGGA_MESSAGE_FMT "{int, double,double,double,char,double,double,char,int,int,double,double,double,double,double,int,double,string}"
#define CARMEN_GPS_GPGGA_MESSAGE_NAME "carmen_gps_nmea_gpgga"


typedef struct {
	int nr; /**< number of the gps unit **/
	double heading; /**< heading in radians >**/
	int valid; /* 1 or 0 */
	double timestamp;
	char *host;
} carmen_gps_gphdt_message;

#define CARMEN_GPS_GPHDT_MESSAGE_FMT "{int, double,int,double,string}"
#define CARMEN_GPS_GPHDT_MESSAGE_NAME "carmen_gps_nmea_gphdt"


typedef struct {
	int nr; /**< number of the gps unit **/
	int validity; /**< 1 ok, 0 invalid       **/
	double utc; /**< Universal Time Coordinated (UTC) **/
	double latitude; /**< latitude as a real floating point number **/
	double latitude_dm; /**< latitude as DDMM.MMMM format (as in the NMEA text message) **/
	char lat_orient; /**< N or S (North or South) **/
	double longitude; /**< longitude as a real floating point number **/
	double longitude_dm; /**< longitude as DDMM.MMMM format (as in the NMEA text message) **/
	char long_orient; /**< E or W (East or West) **/
	double speed; /**< Speed over ground in m/s in the direction of true_course **/
	double true_course; /**< heading to north (in rads) **/
	double variation; /**< Magnetic variation (in rads) (E or W) subtracts 
			 from true course  **/
	char var_dir;
	int date; /**< UT Date  **/
	double timestamp;
	char* host; 
} carmen_gps_gprmc_message;

#define CARMEN_GPS_GPRMC_MESSAGE_FMT  "{int, char,double,double,double,char,double,double,char,double,double,double,char,int,double,string}"
#define CARMEN_GPS_GPRMC_MESSAGE_NAME "carmen_gps_nmea_gprmc"

#ifdef __cplusplus
}
#endif

#endif

// @}
