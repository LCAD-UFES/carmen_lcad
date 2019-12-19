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

/** @addtogroup logger libreadlog **/
// @{

/**
 * \file readlog.h
 * \brief Library for reading log files.
 *
 * This library should be used to read logfiles. It uses an index
 * structure in order to go thourgh the individual messages of the
 * carmen log file. This library furthermore provides conversion
 * functions that convert strings (representing a message in the
 * carmen log file format) into the corresponding
 * laser/odometry/etc. message.
 **/

#ifndef CARMEN_READLOG_H
#define CARMEN_READLOG_H

#include <carmen/visual_odometry_messages.h>
#include <carmen/kinect_messages.h>
#include <carmen/visual_odometry_messages.h>
#include <carmen/simulator_ackerman_messages.h>
#include <carmen/robot_ackerman_messages.h>
#include <carmen/web_cam_messages.h>
#include <carmen/base_ackerman_messages.h>
#include <carmen/ultrasonic_filter_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CLF_READ_DOUBLE(str) strtod(*(str), (str))
#define CLF_READ_INT(str) (int)strtol(*(str), (str), 10)
#define CLF_READ_UNSIGNED_INT(str) (int)strtoul(*(str), (str), 10)
#define CLF_READ_CHAR(str) (char) ( ( (*str)++)[0] )
void CLF_READ_STRING(char *dst, char **string);

/** Index structure used to process a logfile. **/
typedef struct {
  int num_messages;     /**< Number of message in the file. **/
  int current_position; /**< Iterator to move through the file. **/
  off_t *offset;     /**< Array of indices to the messages. **/
} carmen_logfile_index_t, *carmen_logfile_index_p;

off_t carmen_logfile_uncompressed_length(carmen_FILE *infile);

void copy_host_string(char **host, char **string);

/** Builds the index structure used for parsing a carmen log file.
 * @param infile  A pointer to a CARMEN_FILE.
 * @returns A pointer to the newly created index structure.
 **/
carmen_logfile_index_p carmen_logfile_index_messages(carmen_FILE *infile);

/** Frees an index structure **/
void carmen_logfile_free_index(carmen_logfile_index_p* pindex);

/** End of file reached in the index structure. **/
int carmen_logfile_eof(carmen_logfile_index_p index);

/** Percentage of the how much data has been read **/
float carmen_logfile_percent_read(carmen_logfile_index_p index);

/** Reads a line from the log file using the index structre .
 * @param index A pointer to the index structure correspondinf to infile.
 * @param infile A Carmen file pointer to the log file.
 * @param message_num The index of the message to read.
 * @param max_line_length The size of line.
 * @param line A pointer to the address where the line is written to.
 * @returns Number of read bytes.
 **/
int carmen_logfile_read_line(carmen_logfile_index_p index, carmen_FILE *infile,
			     int message_num, int max_line_length, char *line);

/** Reads the next line from the log file using the index structre.
 * @param index A pointer to the index structure correspondinf to infile.
 * @param infile A Carmen file pointer.
 * @param max_line_length allocated memory in line.
 * @param line A pointer to the address where the line is written to.
 * @returns Number of read bytes.
 **/
int carmen_logfile_read_next_line(carmen_logfile_index_p index, carmen_FILE *infile,
				  int max_line_length, char *line);

int first_wordlength(char *str);

char *carmen_string_to_visual_odometry_message(char *string,
						carmen_visual_odometry_pose6d_message *odometry);

/** Converts the string to a truepos message.
 * @param string A string describing the message in the *old* carmen logfile format.
 * @param truepos A pointer to the (allocated) structure where the message should be written to.
 * @returns A pointer to the character up to which the string has been parsed.
 * In case there are no errors and the string is a line read from a carmen log
 * file, this string points to the relative timestamp (written by logger).
 **/
char *carmen_string_to_simulator_ackerman_truepos_message(char *string,
						 carmen_simulator_ackerman_truepos_message *truepos);

char *carmen_string_to_robot_ackerman_laser_message(char *string,
					   carmen_robot_ackerman_laser_message *laser);

/** Converts the string to an old laser_laser_message.
 * @param string A string describing the message in the *old* carmen logfile format.
 * @param laser A pointer to the (allocated) structure where the message should be written to.
 * @returns A pointer to the character up to which the string has been parsed.
 * In case there are no errors and the string is a line read from a carmen log
 * file, this string points to the relative timestamp (written by logger).
 **/
char *carmen_string_to_laser_laser_message_orig(char *string,
						carmen_laser_laser_message
						*laser);

/** Converts the string to a (new) laser_laser_message.
 * @param string A string describing the message in the carmen logfile format.
 * @param laser A pointer to the (allocated) structure where the message should be written to.
 * @returns A pointer to the character up to which the string has been parsed.
 * In case there are no errors and the string is a line read from a carmen log
 * file, this string points to the relative timestamp (written by logger).
 **/
char *carmen_string_to_laser_laser_message(char *string,
					   carmen_laser_laser_message *laser);


/** Converts the string to a gps-nmea-rmc message.
 * @param string A string describing the message in the carmen logfile format.
 * @param gps_msg A pointer to the (allocated) structure where the message should be written to.
 * @returns A pointer to the character up to which the string has been parsed.
 * In case there are no errors and the string is a line read from a carmen log
 * file, this string points to the relative timestamp (written by logger).
 **/
char *carmen_string_to_gps_gprmc_message(char *string,
					 carmen_gps_gprmc_message *gps_msg);

/** Converts the string to a gps-nmea-gga message.
 * @param string A string describing the message in the carmen logfile format.
 * @param gps_msg A pointer to the (allocated) structure where the message should be written to.
 * @returns A pointer to the character up to which the string has been parsed.
 * In case there are no errors and the string is a line read from a carmen log
 * file, this string points to the relative timestamp (written by logger).
 **/
char *carmen_string_to_gps_gpgga_message(char *string,
					 carmen_gps_gpgga_message *gps_msg);

/** Converts the string to a gps-nmea-hdt message.
 * @param string A string describing the message in the carmen logfile format.
 * @param gps_msg A pointer to the (allocated) structure where the message should be written to.
 * @returns A pointer to the character up to which the string has been parsed.
 * In case there are no errors and the string is a line read from a carmen log
 * file, this string points to the relative timestamp (written by logger).
 **/
char *carmen_string_to_gps_gphdt_message(char *string,
					 carmen_gps_gphdt_message *gps_msg);

char* carmen_string_to_bumblebee_basic_stereoimage_message(char* string, carmen_bumblebee_basic_stereoimage_message* msg);

char* carmen_string_to_ford_escape_estatus_message(char* string, carmen_ford_escape_status_message* msg);

char* carmen_string_to_globalpos_message(char* string, carmen_localize_ackerman_globalpos_message* msg);

char* carmen_string_and_file_to_bumblebee_basic_stereoimage_message(char* string, carmen_bumblebee_basic_stereoimage_message* msg);

/** Converts the string to a robot-ackerman-velocity message.
 * @param string A string describing the message in the carmen logfile format.
 * @param msg A pointer to the (allocated) structure where the message should be written to.
 * @returns A pointer to the character up to which the string has been parsed.
 * In case there are no errors and the string is a line read from a carmen log
 * file, this string points to the relative timestamp (written by logger).
 **/
char *carmen_string_to_robot_ackerman_velocity_message(char *string, carmen_robot_ackerman_velocity_message *msg);


/** Guesses the field of view for a SICK in case the configuration is underspecified.
 * @param number of laser beams
 * @returns the field of view if a SICK with that number of beams
 **/
double carmen_laser_guess_fov(int num_beams);

/** Guesses the angle increment (angle between two beams) of a SICK in case the configuration is underspecified.
 * @param number of laser beams
 * @returns the angle increment
 **/
double carmen_laser_guess_angle_increment(int num_beams);

char* carmen_string_to_pantilt_scanmark_message(char* string, carmen_pantilt_scanmark_message* scanmark);

char* carmen_string_to_pantilt_laserpos_message(char* string, carmen_pantilt_laserpos_message* laserpos);

char* carmen_string_to_imu_message(char* string, carmen_imu_message* msg);

char *carmen_string_to_kinect_depth_message(char *string, carmen_kinect_depth_message *kinect);

char *carmen_string_to_kinect_video_message(char *string, carmen_kinect_video_message *kinect);

char* carmen_string_to_velodyne_partial_scan_message(char* string, carmen_velodyne_partial_scan_message* msg);

char* carmen_string_and_file_to_velodyne_partial_scan_message(char* string, carmen_velodyne_partial_scan_message* msg);

char* carmen_string_to_variable_velodyne_scan_message(char* string, carmen_velodyne_variable_scan_message* msg);

char* carmen_string_and_file_to_variable_velodyne_scan_message(char* string, carmen_velodyne_variable_scan_message* msg);

char* carmen_string_to_velodyne_gps_message(char* string, carmen_velodyne_gps_message* msg);

char* carmen_string_to_xsens_euler_message(char* string, carmen_xsens_global_euler_message* msg);

char* carmen_string_to_xsens_quat_message(char* string, carmen_xsens_global_quat_message* msg);

char* carmen_string_to_xsens_matrix_message(char* string, carmen_xsens_global_matrix_message* msg);

char* carmen_string_to_xsens_mtig_message(char* string, carmen_xsens_mtig_message* msg);

char* carmen_string_to_web_cam_message(char* string, carmen_web_cam_message* msg);

char* carmen_string_to_base_ackerman_odometry_message(char* string, carmen_base_ackerman_odometry_message* odometry);

char* carmen_string_to_base_ackerman_motion_message(char* string, carmen_base_ackerman_motion_command_message* msg);

char* carmen_string_to_ultrasonic_message(char* string, carmen_ultrasonic_sonar_sensor_message* ultrasonic_msg);

char* carmen_string_to_laser_ldmrs_message(char *string, carmen_laser_ldmrs_message *laser);

char *carmen_string_to_laser_ldmrs_new_message(char *string, carmen_laser_ldmrs_new_message *laser);

char* carmen_string_to_laser_ldmrs_objects_message(char *string, carmen_laser_ldmrs_objects_message *laserObjects);

char *carmen_string_to_laser_ldmrs_objects_data_message(char *string, carmen_laser_ldmrs_objects_data_message *laserObjectsData);

#ifdef __cplusplus
}
#endif

#endif

// @}
