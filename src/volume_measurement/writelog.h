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

/** @addtogroup logger libwritelog **/
// @{

/**
 * \file writelog.h
 * \brief Library for writing log files.
 *
 * This library should be used to write logfiles.
 **/


#ifndef CARMEN_LOGWRITE_H
#define CARMEN_LOGWRITE_H

#include <carmen/carmen.h>
#include <carmen/carmen_stdio.h>
#include <carmen/web_cam_interface.h>
#include <carmen/ultrasonic_filter_messages.h>
#include <carmen/kinect_messages.h>
#include <carmen/xsens_messages.h>
#include <carmen/xsens_mtig_messages.h>
#include <carmen/robot_ackerman_messages.h>
#include <carmen/simulator_ackerman_messages.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/velodyne_messages.h>
#include <carmen/visual_odometry_messages.h>
#include <carmen/base_ackerman_messages.h>
#include <carmen/ford_escape_hybrid_messages.h>
#include <carmen/can_dump_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CARMEN_LOGFILE_HEADER "# CARMEN Logfile"

void carmen_logwrite_write_robot_name(char *robot_name,
				      carmen_FILE *outfile);

void carmen_logwrite_write_header(carmen_FILE *outfile);

/** Converts the corresponding message into a string in the carmen log format.
 * @param laser The message that should be written to the log file.
 * @param laser_num The number of the laser (1=front, 2=rear, 3=laser3, 4=laser4, ...)
 * @param outfile A carmen file pointer.
 * @param timestamp The relative timestamp (when was the message received). Note that this is not the timestamp within the message!
 **/
void carmen_logwrite_write_laser_laser(carmen_laser_laser_message *laser,
				       int laser_num, carmen_FILE *outfile,
					double timestamp);

void carmen_logwrite_write_robot_ackerman_laser(carmen_robot_ackerman_laser_message *laser,
				       int laser_num, carmen_FILE *outfile,
				       double timestamp);

/** Converts the given parameter into a string in the carmen log format.
 **/
void carmen_logwrite_write_param(char *module, char *variable, char *value,
				 double ipc_time, char *hostname,
				 carmen_FILE *outfile, double timestamp);

/** Converts the corresponding message into a string in the carmen log format.
 * @param sync_message The message that should be written to the log file.
 * @param outfile A carmen file pointer.
 **/
void carmen_logwrite_write_sync(carmen_logger_sync_message *sync_message,
				carmen_FILE *outfile);

/** Converts the corresponding message into a string in the carmen log format.
 * @param truepos The message that should be written to the log file.
 * @param outfile A carmen file pointer.
 * @param timestamp The relative timestamp (when was the message received). Note that this is not the timestamp within the message!
 **/
void carmen_logwrite_write_ackerman_truepos(carmen_simulator_ackerman_truepos_message *truepos,
				   carmen_FILE *outfile, double timestamp);

void carmen_logwrite_write_localize_ackerman(carmen_localize_ackerman_globalpos_message *msg,
				    carmen_FILE *outfile, double timestamp);

/** Converts the corresponding message into a string in the carmen log format.
 * @param gps_msg The message that should be written to the log file.
 * @param outfile A carmen file pointer.
 * @param timestamp The relative timestamp (when was the message received). Note that this is not the timestamp within the message!
 **/
void carmen_logger_write_gps_gpgga(carmen_gps_gpgga_message *gps_msg,
				   carmen_FILE *outfile,
				   double timestamp);

/** Converts the corresponding message into a string in the carmen log format.
 * @param gps_msg The message that should be written to the log file.
 * @param outfile A carmen file pointer.
 * @param timestamp The relative timestamp (when was the message received). Note that this is not the timestamp within the message!
 **/
void carmen_logger_write_gps_gphdt(carmen_gps_gphdt_message *gps_msg,
				   carmen_FILE *outfile,
				   double timestamp);

/** Converts the corresponding message into a string in the carmen log format.
 * @param gps_msg The message that should be written to the log file.
 * @param outfile A carmen file pointer.
 * @param timestamp The relative timestamp (when was the message received). Note that this is not the timestamp within the message!
 **/
void carmen_logger_write_gps_gprmc(carmen_gps_gprmc_message *gps_msg,
				   carmen_FILE *outfile,
				   double timestamp);

void carmen_logwrite_write_ultrasonic_sonar_sensor(carmen_ultrasonic_sonar_sensor_message *sonar,
				       carmen_FILE *outfile,
				       double timestamp);

void carmen_logwrite_write_pantilt_scanmark(carmen_pantilt_scanmark_message* scanmark,
				       carmen_FILE* outfile,
				       double timestamp);

void carmen_logwrite_write_pantilt_laserpos(carmen_pantilt_laserpos_message* laserpos,
				       carmen_FILE* outfile,
				       double timestamp);


void carmen_logwrite_write_imu(carmen_imu_message *imu,
			       carmen_FILE *outfile,
			       double timestamp);


void carmen_logwrite_write_robot_ackerman_vector_move(carmen_robot_ackerman_vector_move_message *msg,
					carmen_FILE *outfile,
					double timestamp);

void carmen_logwrite_write_robot_ackerman_velocity(carmen_robot_ackerman_velocity_message *msg,
					carmen_FILE *outfile,
					double timestamp);

void carmen_logwrite_write_robot_ackerman_follow_trajectory(carmen_robot_ackerman_follow_trajectory_message *msg,
					carmen_FILE *outfile,
					double timestamp);

/** Converts the corresponding message into a string in the carmen log format.
 * @param msg The message that should be written to the log file.
 * @param outfile A carmen file pointer.
 * @param timestamp The relative timestamp (when was the message received). Note that this is not the timestamp within the message!
 **/
void carmen_logwrite_write_logger_comment(carmen_logger_comment_message *msg,
              carmen_FILE *outfile,
              double timestamp);


void carmen_logwrite_write_kinect_depth(carmen_kinect_depth_message *kinect,
		int kinect_num, carmen_FILE *outfile,
		double timestamp);

void carmen_logwrite_write_kinect_video(carmen_kinect_video_message *kinect,
		int kinect_num, carmen_FILE *outfile,
		double timestamp);

void carmen_logwrite_write_velodyne_partial_scan(carmen_velodyne_partial_scan_message* msg, carmen_FILE* outfile, double timestamp);

void carmen_logwrite_write_to_file_velodyne(carmen_velodyne_partial_scan_message* msg, carmen_FILE *outfile, double timestamp, char *log_filename);

void carmen_logwrite_write_to_file_velodyne_variable(carmen_velodyne_variable_scan_message* msg, int velodyne_number, carmen_FILE *outfile, double timestamp, char *log_filename);

void carmen_logwrite_write_variable_velodyne_scan(carmen_velodyne_variable_scan_message* msg, int velodyne_number, carmen_FILE* outfile, double timestamp);

void carmen_logwrite_write_velodyne_gps(carmen_velodyne_gps_message* msg, carmen_FILE* outfile, double timestamp);

void carmen_logwrite_write_bumblebee_basic_steroimage(carmen_bumblebee_basic_stereoimage_message* msg, int bumblebee_num, carmen_FILE *outfile,
		double timestamp, int frequency);

void carmen_logwrite_write_to_file_bumblebee_basic_steroimage(carmen_bumblebee_basic_stereoimage_message* msg, int bumblebee_num, carmen_FILE *outfile,
		double timestamp, int frequency, char *log_filename);

void carmen_logwrite_write_xsens_euler(carmen_xsens_global_euler_message* msg,
			       carmen_FILE *outfile,
			       double timestamp);


void carmen_logwrite_write_xsens_quat(carmen_xsens_global_quat_message* msg,
			       carmen_FILE *outfile,
			       double timestamp);


void carmen_logwrite_write_xsens_matrix(carmen_xsens_global_matrix_message* msg,
			       carmen_FILE *outfile,
			       double timestamp);

void carmen_logwrite_write_xsens_mtig(	carmen_xsens_mtig_message* msg,
			       carmen_FILE *outfile,
			       double timestamp);

void
carmen_logwrite_write_web_cam_message (carmen_web_cam_message *msg,
		carmen_FILE *outfile,
		double timestamp);

void carmen_logwrite_write_visual_odometry(carmen_visual_odometry_pose6d_message *odometry,
		carmen_FILE *outfile, double timestamp);

void carmen_logwrite_write_odometry_ackerman(carmen_base_ackerman_odometry_message *odometry,
		carmen_FILE *outfile,
		double timestamp);

void carmen_logwrite_write_base_ackerman_velocity(carmen_base_ackerman_velocity_message *msg,
		carmen_FILE *outfile,
		double timestamp);

void carmen_logwrite_write_base_ackerman_motion(carmen_base_ackerman_motion_command_message *msg,
		carmen_FILE *outfile,
		double timestamp);

void carmen_logwrite_write_laser_ldmrs(carmen_laser_ldmrs_message *laser,
		int laser_num, carmen_FILE *outfile,
		double timestamp);

void carmen_logwrite_write_laser_ldmrs_new(carmen_laser_ldmrs_new_message *laser,
		int laser_num, carmen_FILE *outfile,
		double timestamp);

void carmen_logwrite_write_laser_ldmrs_objects(carmen_laser_ldmrs_objects_message *laser,
		int laser_num, carmen_FILE *outfile,
		double timestamp);

void carmen_logwrite_write_laser_ldmrs_objects_data(carmen_laser_ldmrs_objects_data_message *laser,
		int laser_num, carmen_FILE *outfile,
		double timestamp);

void carmen_logwrite_write_ford_escape_status_message(carmen_ford_escape_status_message *msg,
		carmen_FILE *outfile,
		double timestamp);

void carmen_logwrite_write_carmen_can_dump_can_line_message(
		carmen_can_dump_can_line_message *msg, carmen_FILE *outfile,
		double timestamp);

#ifdef __cplusplus
}
#endif

#endif
// @}
