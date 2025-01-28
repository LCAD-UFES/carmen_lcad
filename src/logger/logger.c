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

#include <carmen/carmen.h>
#include <sys/stat.h>
#include <string.h>
#include "writelog.h"
#include "logger.h"

char *log_filename = NULL;

carmen_FILE *outfile = NULL;
double logger_starttime;

static int log_visual_odometry = 1;
static int log_odometry = 1;
static int log_laser = 1;
static int log_sonar = 1;
static int log_velodyne = 1;
static int log_robot_laser = 1;
static int log_localize = 1;
static int log_simulator = 1;
static int log_params = 1;
static int log_gps = 1;
static int log_imu = 1;
static int log_pantilt = 1;
static int log_motioncmds = 0;
static int log_kinect = 1;
static int log_xsens = 1;
static int log_xsens_mtig = 1;
static int log_imu_pi = 1;
static int log_bumblebee = 1;
static int log_camera = 1;
static int log_web_cam = 1;
static int log_bumblebee_frames_to_save = 1;
static int log_bumblebee_save_to_file = 0;
static int log_velodyne_save_to_file = 0;
static int log_ford_escape_status = 0;
static int log_can_dump = 0;
char* log_path = 0;
char* suffix = NULL;
char* prefix = NULL;
int compress_image = 0;

void
get_logger_params(int argc, char** argv)
{
  carmen_param_t param_list[] = {
    {"logger", "odometry",    			CARMEN_PARAM_ONOFF, &log_odometry, 0, NULL},
    {"logger", "visual_odometry",		CARMEN_PARAM_ONOFF, &log_visual_odometry, 0, NULL},
    {"logger", "laser",       			CARMEN_PARAM_ONOFF, &log_laser, 0, NULL},
    {"logger", "velodyne",    			CARMEN_PARAM_ONOFF, &log_velodyne, 0, NULL},
    {"logger", "robot_laser", 			CARMEN_PARAM_ONOFF, &log_robot_laser, 0, NULL},
    {"logger", "localize",    			CARMEN_PARAM_ONOFF, &log_localize, 0, NULL},
    {"logger", "params",      			CARMEN_PARAM_ONOFF, &log_params, 0, NULL},
    {"logger", "simulator",   			CARMEN_PARAM_ONOFF, &log_simulator, 0, NULL},
    {"logger", "gps",         			CARMEN_PARAM_ONOFF, &log_gps, 0, NULL},
    {"logger", "imu",         			CARMEN_PARAM_ONOFF, &log_imu, 0, NULL},
    {"logger", "motioncmds",  			CARMEN_PARAM_ONOFF, &log_motioncmds, 0, NULL},
    {"logger", "kinect",      			CARMEN_PARAM_ONOFF, &log_kinect, 0, NULL},
    {"logger", "xsens",       			CARMEN_PARAM_ONOFF, &log_xsens, 0, NULL},
    {"logger", "xsens_mtig",  			CARMEN_PARAM_ONOFF, &log_xsens_mtig, 0, NULL},
	{"logger", "imu_pi",  				CARMEN_PARAM_ONOFF, &log_imu_pi, 0, NULL},
    {"logger", "bumblebee",   			CARMEN_PARAM_ONOFF, &log_bumblebee, 0, NULL},
    {"logger", "camera",   				CARMEN_PARAM_ONOFF, &log_camera, 0, NULL},
    {"logger", "web_cam",   			CARMEN_PARAM_ONOFF, &log_web_cam, 0, NULL},
    {"logger", "bumblebee_frames_to_save", CARMEN_PARAM_INT, &log_bumblebee_frames_to_save, 0, NULL},
    {"logger", "sonar",       			CARMEN_PARAM_ONOFF, &log_sonar, 0, NULL},
    {"logger", "velodyne_save_to_file",	CARMEN_PARAM_ONOFF, &log_velodyne_save_to_file, 0, NULL},
    {"logger", "bumblebee_save_to_file", CARMEN_PARAM_ONOFF, &log_bumblebee_save_to_file, 0, NULL},
    {"logger", "ford_escape_status", 	CARMEN_PARAM_ONOFF, &log_ford_escape_status, 0, NULL},
    {"logger", "can_dump", 				CARMEN_PARAM_ONOFF, &log_can_dump, 0, NULL},
  };
  carmen_param_install_params(argc, argv, param_list, sizeof(param_list)/sizeof(param_list[0]));

  carmen_param_allow_unfound_variables(1);
  carmen_param_t optional_commandline_param_list[] =
  {
	{(char *) "commandline", (char *) "automatic_file", CARMEN_PARAM_STRING, &log_path, 0, NULL},
	{(char *) "commandline", (char *) "compress_image", CARMEN_PARAM_ONOFF, &compress_image, 0, NULL},
	{(char *) "commandline", (char *) "prefix", CARMEN_PARAM_STRING, &prefix, 0, NULL},
	{(char *) "commandline", (char *) "suffix", CARMEN_PARAM_STRING, &suffix, 0, NULL},
  };
  carmen_param_install_params(argc, argv, optional_commandline_param_list, sizeof(optional_commandline_param_list) / sizeof(optional_commandline_param_list[0]));
  carmen_param_allow_unfound_variables(0);
}

void 
get_all_params(void)
{
  char **variables, **values, **modules;
  int list_length, index, num_modules, module_index;
  char *robot_name, *hostname;

  robot_name = carmen_param_get_robot();
  carmen_param_get_modules(&modules, &num_modules);
  carmen_logwrite_write_robot_name(robot_name, outfile);
  free(robot_name);
  carmen_param_get_paramserver_host(&hostname);
  for(module_index = 0; module_index < num_modules; module_index++) {
    if(carmen_param_get_all(modules[module_index], &variables, &values, NULL,
			    &list_length) < 0) {
      IPC_perror("Error retrieving all variables of module");
      exit(-1);
    }
    for(index = 0; index < list_length; index++) {
      carmen_logwrite_write_param(modules[module_index], variables[index],
				  values[index], carmen_get_time(),
				  hostname, outfile, carmen_get_time());
      free(variables[index]);
      free(values[index]);
    }
    free(variables);
    free(values);
    free(modules[module_index]);
  }
  free(hostname);
  free(modules);
}

void param_change_handler(carmen_param_variable_change_message *msg)
{
  carmen_logwrite_write_param(msg->module_name, msg->variable_name,
			      msg->value, msg->timestamp,
			      msg->host, outfile, carmen_get_time());
}

void carmen_simulator_ackerman_truepos_handler(carmen_simulator_ackerman_truepos_message
				      *truepos)
{
  //fprintf(stderr, "T");
  carmen_logwrite_write_ackerman_truepos(truepos, outfile,
				carmen_get_time() - logger_starttime);
}

void base_ackerman_odometry_handler(carmen_base_ackerman_odometry_message *odometry)
{
  //fprintf(stderr, "O");
  carmen_logwrite_write_odometry_ackerman(odometry, outfile,
				 carmen_get_time() - logger_starttime);
}

void visual_odometry_handler(carmen_visual_odometry_pose6d_message *message)
{
	carmen_logwrite_write_visual_odometry(message, outfile,
				carmen_get_time() - logger_starttime);
}

void ultrasonic_sonar_sensor_handler(carmen_ultrasonic_sonar_sensor_message *sonar)
{
  //fprintf(stderr, "S");
  carmen_logwrite_write_ultrasonic_sonar_sensor(sonar, outfile,
				 carmen_get_time() - logger_starttime);
}

void pantilt_scanmark_handler(carmen_pantilt_scanmark_message *scanmark)
{
  //fprintf(stderr, "M");
  carmen_logwrite_write_pantilt_scanmark(scanmark, outfile,
			    carmen_get_time() - logger_starttime);
}

void pantilt_laserpos_handler(carmen_pantilt_laserpos_message *laserpos)
{
  //fprintf(stderr, "P");
  carmen_logwrite_write_pantilt_laserpos(laserpos, outfile,
			    carmen_get_time() - logger_starttime);
}


void robot_frontlaser_ackerman_handler(carmen_robot_ackerman_laser_message *laser)
{
  //fprintf(stderr, "F");
  carmen_logwrite_write_robot_ackerman_laser(laser, 1, outfile,
				    carmen_get_time() - logger_starttime);
}

void robot_rearlaser_ackerman_handler(carmen_robot_ackerman_laser_message *laser)
{
  //fprintf(stderr, "R");
  carmen_logwrite_write_robot_ackerman_laser(laser, 2, outfile,
				    carmen_get_time() - logger_starttime);
}

void laser_laser1_handler(carmen_laser_laser_message *laser)
{
  //fprintf(stderr, "1");
  carmen_logwrite_write_laser_laser(laser, 1, outfile,
				    carmen_get_time() - logger_starttime);
}

void laser_laser2_handler(carmen_laser_laser_message *laser)
{
  //fprintf(stderr, "2");
  carmen_logwrite_write_laser_laser(laser, 2, outfile,
				    carmen_get_time() - logger_starttime);
}

void laser_laser3_handler(carmen_laser_laser_message *laser)
{
  //fprintf(stderr, "3");
  carmen_logwrite_write_laser_laser(laser, 3, outfile,
				    carmen_get_time() - logger_starttime);
}

void laser_laser4_handler(carmen_laser_laser_message *laser)
{
  //fprintf(stderr, "4");
  carmen_logwrite_write_laser_laser(laser, 4, outfile,
				    carmen_get_time() - logger_starttime);
}

void laser_laser5_handler(carmen_laser_laser_message *laser)
{
  //fprintf(stderr, "5");
  carmen_logwrite_write_laser_laser(laser, 5, outfile,
				    carmen_get_time() - logger_starttime);
}

void laser_ldmrs_handler(carmen_laser_ldmrs_message *laser)
{
  //fprintf(stderr, "1");
  carmen_logwrite_write_laser_ldmrs(laser, 1, outfile,
				    carmen_get_time() - logger_starttime);
}

void laser_ldmrs_new_handler(carmen_laser_ldmrs_new_message *laser)
{
  //fprintf(stderr, "1");
  carmen_logwrite_write_laser_ldmrs_new(laser, 1, outfile,
				    carmen_get_time() - logger_starttime);
}

void laser_ldmrs_objects_handler(carmen_laser_ldmrs_objects_message *laser)
{
  //fprintf(stderr, "1");
  carmen_logwrite_write_laser_ldmrs_objects(laser, 1, outfile,
				    carmen_get_time() - logger_starttime);
}

void laser_ldmrs_objects_data_handler(carmen_laser_ldmrs_objects_data_message *laser)
{
  //fprintf(stderr, "1");
  carmen_logwrite_write_laser_ldmrs_objects_data(laser, 1, outfile,
				    carmen_get_time() - logger_starttime);
}

void localize_ackerman_handler(carmen_localize_ackerman_globalpos_message *msg)
{
  //fprintf(stderr, "L");
  carmen_logwrite_write_localize_ackerman(msg, outfile, carmen_get_time() -
				 logger_starttime);
}

static void sync_handler(carmen_logger_sync_message *sync)
{
  carmen_logwrite_write_sync(sync, outfile);
}

void ipc_gps_gpgga_handler( carmen_gps_gpgga_message *gps_data)
{
  carmen_logger_write_gps_gpgga(gps_data, outfile, carmen_get_time() - logger_starttime);
}

void ipc_gps_gphdt_handler( carmen_gps_gphdt_message *gps_data)
{
  carmen_logger_write_gps_gphdt(gps_data, outfile, carmen_get_time() - logger_starttime);
}

void ipc_gps_gprmc_handler( carmen_gps_gprmc_message *gps_data)
{
  carmen_logger_write_gps_gprmc(gps_data, outfile, carmen_get_time() - logger_starttime);
}

void imu_handler(carmen_imu_message *msg)
{
  //fprintf(stderr, "i");
  carmen_logwrite_write_imu(msg, outfile, carmen_get_time() - logger_starttime);
}

void xsens_quat_handler(carmen_xsens_global_quat_message *msg)
{
  //fprintf(stderr, "xq");
  carmen_logwrite_write_xsens_quat(msg, outfile, carmen_get_time() - logger_starttime);
}

void pi_imu_handler(carmen_pi_imu_message_t *msg)
{
  //fprintf(stderr, "xq");
	carmen_logwrite_write_pi_imu( msg, outfile, carmen_get_time() - logger_starttime);
}

void xsens_euler_handler(carmen_xsens_global_euler_message *msg){
  //fprintf(stderr, "xe");
  carmen_logwrite_write_xsens_euler(msg, outfile, carmen_get_time() - logger_starttime);
}

void xsens_matrix_handler(carmen_xsens_global_matrix_message *msg){
  //fprintf(stderr, "xm");
  carmen_logwrite_write_xsens_matrix(msg, outfile, carmen_get_time() - logger_starttime);
}

void xsens_mtig_handler(carmen_xsens_mtig_message *msg){
	//fprintf(stderr, "xi");
	carmen_logwrite_write_xsens_mtig(msg, outfile, carmen_get_time() - logger_starttime);
}

void robot_ackerman_follow_trajectory_handler( carmen_robot_ackerman_follow_trajectory_message *msg)
{
	//fprintf(stderr, "t");
	carmen_logwrite_write_robot_ackerman_follow_trajectory(msg, outfile, carmen_get_time() - logger_starttime);
}

void robot_ackerman_vector_move_handler( carmen_robot_ackerman_vector_move_message *msg )
{
	//fprintf(stderr, "m");
	carmen_logwrite_write_robot_ackerman_vector_move(msg, outfile, carmen_get_time() - logger_starttime);
}

void robot_ackerman_velocity_handler( carmen_robot_ackerman_velocity_message *msg )
{
	//fprintf(stderr, "v");
	carmen_logwrite_write_robot_ackerman_velocity(msg, outfile, carmen_get_time() - logger_starttime);
}

void base_ackerman_velocity_handler( carmen_base_ackerman_velocity_message *msg )
{
	//fprintf(stderr, "b");
	carmen_logwrite_write_base_ackerman_velocity(msg, outfile, carmen_get_time() - logger_starttime);
}

void base_ackerman_motion_handler( carmen_base_ackerman_motion_command_message *msg )
{
	//fprintf(stderr, "b");
	carmen_logwrite_write_base_ackerman_motion(msg, outfile, carmen_get_time() - logger_starttime);
}

void logger_comment_handler( carmen_logger_comment_message *msg )
{
	//fprintf(stderr, "C");
	carmen_logwrite_write_logger_comment(msg, outfile, carmen_get_time() - logger_starttime);
}

void velodyne_partial_scan_handler( carmen_velodyne_partial_scan_message* msg)
{
	//fprintf(stderr, "V");
	if (log_velodyne_save_to_file)
		carmen_logwrite_write_to_file_velodyne(msg, outfile, carmen_get_time() - logger_starttime, log_filename);
	else
		carmen_logwrite_write_velodyne_partial_scan(msg, outfile, carmen_get_time() - logger_starttime);
}


void velodyne_variable_scan_handler0( carmen_velodyne_variable_scan_message* msg)
{
	//fprintf(stderr, "VV1");
	if (log_velodyne_save_to_file)
		carmen_logwrite_write_to_file_velodyne_variable(msg, 0, outfile, carmen_get_time() - logger_starttime, log_filename);
	else
		carmen_logwrite_write_variable_velodyne_scan(msg, 0, outfile, carmen_get_time() - logger_starttime);
}


void velodyne_variable_scan_handler1( carmen_velodyne_variable_scan_message* msg)
{
	//fprintf(stderr, "VV1");
	if (log_velodyne_save_to_file)
		carmen_logwrite_write_to_file_velodyne_variable(msg, 1, outfile, carmen_get_time() - logger_starttime, log_filename);
	else
		carmen_logwrite_write_variable_velodyne_scan(msg, 1, outfile, carmen_get_time() - logger_starttime);
}

void velodyne_variable_scan_handler2( carmen_velodyne_variable_scan_message* msg)
{
	//fprintf(stderr, "VV2");
	if (log_velodyne_save_to_file)
		carmen_logwrite_write_to_file_velodyne_variable(msg, 2, outfile, carmen_get_time() - logger_starttime, log_filename);
	else
		carmen_logwrite_write_variable_velodyne_scan(msg, 2, outfile, carmen_get_time() - logger_starttime);
}

void velodyne_variable_scan_handler3( carmen_velodyne_variable_scan_message* msg)
{
	//fprintf(stderr, "VV3");
	if (log_velodyne_save_to_file)
		carmen_logwrite_write_to_file_velodyne_variable(msg, 3, outfile, carmen_get_time() - logger_starttime, log_filename);
	else
		carmen_logwrite_write_variable_velodyne_scan(msg, 3, outfile, carmen_get_time() - logger_starttime);
}

void velodyne_variable_scan_handler4( carmen_velodyne_variable_scan_message* msg)
{
	//fprintf(stderr, "VV4");
	if (log_velodyne_save_to_file)
		carmen_logwrite_write_to_file_velodyne_variable(msg, 4, outfile, carmen_get_time() - logger_starttime, log_filename);
	else
		carmen_logwrite_write_variable_velodyne_scan(msg, 4, outfile, carmen_get_time() - logger_starttime);
}


void velodyne_variable_scan_handler5( carmen_velodyne_variable_scan_message* msg)
{
	//fprintf(stderr, "VV5");
	if (log_velodyne_save_to_file)
		carmen_logwrite_write_to_file_velodyne_variable(msg, 5, outfile, carmen_get_time() - logger_starttime, log_filename);
	else
		carmen_logwrite_write_variable_velodyne_scan(msg, 5, outfile, carmen_get_time() - logger_starttime);
}


void velodyne_variable_scan_handler6( carmen_velodyne_variable_scan_message* msg)
{
	//fprintf(stderr, "VV6");
	if (log_velodyne_save_to_file)
		carmen_logwrite_write_to_file_velodyne_variable(msg, 6, outfile, carmen_get_time() - logger_starttime, log_filename);
	else
		carmen_logwrite_write_variable_velodyne_scan(msg, 6, outfile, carmen_get_time() - logger_starttime);
}


void velodyne_variable_scan_handler7( carmen_velodyne_variable_scan_message* msg)
{
	//fprintf(stderr, "VV7");
	if (log_velodyne_save_to_file)
		carmen_logwrite_write_to_file_velodyne_variable(msg, 7, outfile, carmen_get_time() - logger_starttime, log_filename);
	else
		carmen_logwrite_write_variable_velodyne_scan(msg, 7, outfile, carmen_get_time() - logger_starttime);
}


void velodyne_variable_scan_handler8( carmen_velodyne_variable_scan_message* msg)
{
	//fprintf(stderr, "VV8");
	if (log_velodyne_save_to_file)
		carmen_logwrite_write_to_file_velodyne_variable(msg, 8, outfile, carmen_get_time() - logger_starttime, log_filename);
	else
		carmen_logwrite_write_variable_velodyne_scan(msg, 8, outfile, carmen_get_time() - logger_starttime);
}


void velodyne_variable_scan_handler9( carmen_velodyne_variable_scan_message* msg)
{
	//fprintf(stderr, "VV9");
	if (log_velodyne_save_to_file)
		carmen_logwrite_write_to_file_velodyne_variable(msg, 9, outfile, carmen_get_time() - logger_starttime, log_filename);
	else
		carmen_logwrite_write_variable_velodyne_scan(msg, 9, outfile, carmen_get_time() - logger_starttime);
}


void velodyne_variable_scan_handler10( carmen_velodyne_variable_scan_message* msg)
{
	//fprintf(stderr, "VV10");
	if (log_velodyne_save_to_file)
		carmen_logwrite_write_to_file_velodyne_variable(msg, 10, outfile, carmen_get_time() - logger_starttime, log_filename);
	else
		carmen_logwrite_write_variable_velodyne_scan(msg, 10, outfile, carmen_get_time() - logger_starttime);
}


void velodyne_variable_scan_handler11( carmen_velodyne_variable_scan_message* msg)
{
	//fprintf(stderr, "VV11");
	if (log_velodyne_save_to_file)
		carmen_logwrite_write_to_file_velodyne_variable(msg, 11, outfile, carmen_get_time() - logger_starttime, log_filename);
	else
		carmen_logwrite_write_variable_velodyne_scan(msg, 11, outfile, carmen_get_time() - logger_starttime);
}


void velodyne_variable_scan_handler12( carmen_velodyne_variable_scan_message* msg)
{
	//fprintf(stderr, "VV12");
	if (log_velodyne_save_to_file)
		carmen_logwrite_write_to_file_velodyne_variable(msg, 12, outfile, carmen_get_time() - logger_starttime, log_filename);
	else
		carmen_logwrite_write_variable_velodyne_scan(msg, 12, outfile, carmen_get_time() - logger_starttime);
}


void velodyne_variable_scan_handler13( carmen_velodyne_variable_scan_message* msg)
{
	//fprintf(stderr, "VV13");
	if (log_velodyne_save_to_file)
		carmen_logwrite_write_to_file_velodyne_variable(msg, 13, outfile, carmen_get_time() - logger_starttime, log_filename);
	else
		carmen_logwrite_write_variable_velodyne_scan(msg, 13, outfile, carmen_get_time() - logger_starttime);
}


void velodyne_variable_scan_handler14( carmen_velodyne_variable_scan_message* msg)
{
	//fprintf(stderr, "VV14");
	if (log_velodyne_save_to_file)
		carmen_logwrite_write_to_file_velodyne_variable(msg, 14, outfile, carmen_get_time() - logger_starttime, log_filename);
	else
		carmen_logwrite_write_variable_velodyne_scan(msg, 14, outfile, carmen_get_time() - logger_starttime);
}


void velodyne_variable_scan_handler15( carmen_velodyne_variable_scan_message* msg)
{
	//fprintf(stderr, "VV15");
	if (log_velodyne_save_to_file)
		carmen_logwrite_write_to_file_velodyne_variable(msg, 15, outfile, carmen_get_time() - logger_starttime, log_filename);
	else
		carmen_logwrite_write_variable_velodyne_scan(msg, 15, outfile, carmen_get_time() - logger_starttime);
}


void velodyne_gps_handler(carmen_velodyne_gps_message* msg)
{
	//fprintf(stderr, "VG");
	carmen_logwrite_write_velodyne_gps(msg, outfile, carmen_get_time() - logger_starttime);
}

void bumblebee1_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b1s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 1, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, log_filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 1, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}

void bumblebee2_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b2s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 2, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, log_filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 2, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}

void bumblebee3_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b3s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 3, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, log_filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 3, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}

void bumblebee4_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b4s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 4, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, log_filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 4, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}

void bumblebee5_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b5s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 5, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, log_filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 5, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}

void bumblebee6_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b6s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 6, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, log_filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 6, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}

void bumblebee7_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b7s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 7, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, log_filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 7, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}

void bumblebee8_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b8s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 8, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, log_filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 8, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}

void bumblebee9_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b9s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 9, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, log_filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 9, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}

void bumblebee10_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b10s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 10, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, log_filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 10, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}

void bumblebee11_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b11s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 11, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, log_filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 11, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}

void bumblebee12_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b12s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 12, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, log_filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 12, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}

void bumblebee13_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b13s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 13, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, log_filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 13, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}


void
camera1_handler(camera_message *message)
{
	camera_drivers_write_camera_message_to_log(1, compress_image, message, outfile, log_filename, carmen_get_time() - logger_starttime);
}

void
camera2_handler(camera_message *message)
{
	camera_drivers_write_camera_message_to_log(2, compress_image, message, outfile, log_filename, carmen_get_time() - logger_starttime);
}

void
camera3_handler(camera_message *message)
{
	camera_drivers_write_camera_message_to_log(3, compress_image, message, outfile, log_filename, carmen_get_time() - logger_starttime);
}

void
camera4_handler(camera_message *message)
{
	camera_drivers_write_camera_message_to_log(4, compress_image, message, outfile, log_filename, carmen_get_time() - logger_starttime);
}

void
camera5_handler(camera_message *message)
{
	camera_drivers_write_camera_message_to_log(5, compress_image, message, outfile, log_filename, carmen_get_time() - logger_starttime);
}

void
camera6_handler(camera_message *message)
{
	camera_drivers_write_camera_message_to_log(6, compress_image, message, outfile, log_filename, carmen_get_time() - logger_starttime);
}

void
camera7_handler(camera_message *message)
{
	camera_drivers_write_camera_message_to_log(7, compress_image, message, outfile, log_filename, carmen_get_time() - logger_starttime);
}

void
camera8_handler(camera_message *message)
{
	camera_drivers_write_camera_message_to_log(8, compress_image, message, outfile, log_filename, carmen_get_time() - logger_starttime);
}

void
camera9_handler(camera_message *message)
{
	camera_drivers_write_camera_message_to_log(9, compress_image, message, outfile, log_filename, carmen_get_time() - logger_starttime);
}

void
camera10_handler(camera_message *message)
{
	camera_drivers_write_camera_message_to_log(10, compress_image, message, outfile, log_filename, carmen_get_time() - logger_starttime);
}

void
camera11_handler(camera_message *message)
{
	camera_drivers_write_camera_message_to_log(11, compress_image, message, outfile, log_filename, carmen_get_time() - logger_starttime);
}

void
camera12_handler(camera_message *message)
{
	camera_drivers_write_camera_message_to_log(12, compress_image, message, outfile, log_filename, carmen_get_time() - logger_starttime);
}

void
camera13_handler(camera_message *message)
{
	camera_drivers_write_camera_message_to_log(13, compress_image, message, outfile, log_filename, carmen_get_time() - logger_starttime);
}

void
camera14_handler(camera_message *message)
{
	camera_drivers_write_camera_message_to_log(14, compress_image, message, outfile, log_filename, carmen_get_time() - logger_starttime);
}

void
camera15_handler(camera_message *message)
{
	camera_drivers_write_camera_message_to_log(15, compress_image, message, outfile, log_filename, carmen_get_time() - logger_starttime);
}

void
camera16_handler(camera_message *message)
{
	camera_drivers_write_camera_message_to_log(16, compress_image, message, outfile, log_filename, carmen_get_time() - logger_starttime);
}

void
camera17_handler(camera_message *message)
{
	camera_drivers_write_camera_message_to_log(17, compress_image, message, outfile, log_filename, carmen_get_time() - logger_starttime);
}

void
camera18_handler(camera_message *message)
{
	camera_drivers_write_camera_message_to_log(18, compress_image, message, outfile, log_filename, carmen_get_time() - logger_starttime);
}

void
camera19_handler(camera_message *message)
{
	camera_drivers_write_camera_message_to_log(19, compress_image, message, outfile, log_filename, carmen_get_time() - logger_starttime);
}

void
camera20_handler(camera_message *message)
{
	camera_drivers_write_camera_message_to_log(20, compress_image, message, outfile, log_filename, carmen_get_time() - logger_starttime);
}


void
ipc_kinect_0_depth_handler(carmen_kinect_depth_message *message)
{
	//fprintf(stderr, "k0d");
	carmen_logwrite_write_kinect_depth(message, 0, outfile, carmen_get_time() - logger_starttime);
}

void
ipc_kinect_1_depth_handler(carmen_kinect_depth_message *message)
{
	//fprintf(stderr, "k1d");
	carmen_logwrite_write_kinect_depth(message, 1, outfile, carmen_get_time() - logger_starttime);
}

void
ipc_kinect_0_video_handler(carmen_kinect_video_message *message)
{
	//fprintf(stderr, "k0v");
	carmen_logwrite_write_kinect_video(message, 0, outfile, carmen_get_time() - logger_starttime);
}

void
ipc_kinect_1_video_handler(carmen_kinect_video_message *message)
{
	//fprintf(stderr, "k1v");
	carmen_logwrite_write_kinect_video(message, 1, outfile, carmen_get_time() - logger_starttime);
}

void
carmen_web_cam_message_handler(carmen_web_cam_message *message)
{
	carmen_logwrite_write_web_cam_message(message, outfile, carmen_get_time() - logger_starttime);
}

void
ford_escape_status_message_handler(carmen_ford_escape_status_message *message)
{
	carmen_logwrite_write_ford_escape_status_message(message, outfile, carmen_get_time() - logger_starttime);
}

void
can_dump_message_handler(carmen_can_dump_can_line_message *message)
{
	carmen_logwrite_write_carmen_can_dump_can_line_message(message, outfile, carmen_get_time() - logger_starttime);
}

void
register_ipc_messages(void)
{
  carmen_subscribe_message(CARMEN_LOGGER_SYNC_NAME, CARMEN_LOGGER_SYNC_FMT,
			   NULL, sizeof(carmen_logger_sync_message),
			   (carmen_handler_t)sync_handler,
			   CARMEN_SUBSCRIBE_LATEST);

  carmen_subscribe_message(CARMEN_PARAM_VARIABLE_CHANGE_NAME,
			   CARMEN_PARAM_VARIABLE_CHANGE_FMT,
			   NULL, sizeof(carmen_param_variable_change_message),
			   (carmen_handler_t)param_change_handler,
			   CARMEN_SUBSCRIBE_LATEST);
}


void
shutdown_module(int sig)
{
  if(sig == SIGINT) {
    carmen_fclose(outfile);
    carmen_ipc_disconnect();
    fprintf(stderr, "\nDisconnecting.\n");
    exit(0);
  }
}


void 
_increase_file(char *filename, char *outfile)
{
    int inc = 1;
    char token[1024], _token[1024], ext[8], _inc[4], __inc[4];
    // file extension
    char *_ext = strrchr(filename, '.');
    strcpy(ext, _ext);
    int i = 0;
    for (; i < (int) (strlen(filename) - strlen(ext)); i++)
        _token[i] = filename[i];
    _token[i] = '\0';

    // already is incremented
    int hp = 0;
    int j = 0;
    i = strlen(_token) - 1;
    for (; (i > 0) && isdigit(_token[i]) && (j < 3); i--, j++) // only 3-digit
        __inc[j] = _token[i];
    __inc[j] = '\0';
    if (_token[i] == '-')
        hp = 1;
    else
        __inc[0] = '\0';
    int n = strlen(__inc);
    for (j = 0; j < n; j++)
        _inc[j] = __inc[n - j - 1];
    inc = atoi(_inc) + 1;

    if (hp)
    {
        token[i--] = '\0';
        for (; i >= 0; i--)
            token[i] = _token[i];
    }
    else
        strcpy(token, _token);

    sprintf(outfile, "%s-%d%s", token, inc, ext);
}

void 
increase_file(char *filename)
{
    struct stat buf;
    char f1[1024], f2[1024];

    if (stat(filename, &buf) == -1)
    {
        return;
    }

    strcpy(f2, filename);
    while (stat(f2, &buf) != -1)
    {
        memset(f1, 0, sizeof(f1));
        strcpy(f1, f2);
        memset(f2, 0, sizeof(f2));
        _increase_file(f1, f2);
    }
	carmen_warn("\nWARNING: log file %s already exists, renaming to %s\n", filename, f2);
    strcpy(filename, f2);
}

void
get_log_file_name(int argc, char **argv)
{
  if (log_path)
  {
	  log_filename = (char*) malloc (64 * sizeof(char));
	  char date[48]; 
	  carmen_get_date(date);

	  if (prefix && suffix)
		  sprintf(log_filename, "%s/log_%s_%s_%s.txt", log_path, prefix, date, suffix);
	  else if (prefix)
		  sprintf(log_filename, "%s/log_%s_%s.txt", log_path, prefix, date);
	  else if (suffix)
		  sprintf(log_filename, "%s/log_%s_%s.txt", log_path, date, suffix);
	  else
		  sprintf(log_filename, "%s/log_%s.txt", log_path, date);
  }
  else
  {
  	if (argc < 2)
	  carmen_die("Usage: %s <logfile>\n", argv[0]);

    log_filename = argv[1];
  }

  increase_file(log_filename);
}


int 
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	get_logger_params(argc, argv);

	get_log_file_name(argc, argv);

	outfile = carmen_fopen(log_filename, "w");

	if (outfile == NULL)
		carmen_die("Error: Could not open file %s for writing.\n", log_filename);

	carmen_logwrite_write_header(outfile);

	if (!(log_odometry && log_laser && log_robot_laser))
		carmen_warn("\nWARNING: You are neither logging laser nor odometry messages!\n");

	if (log_params)
		get_all_params();

	register_ipc_messages();

	if (log_odometry)
		carmen_robot_ackerman_subscribe_velocity_message(NULL, (carmen_handler_t) robot_ackerman_velocity_handler, CARMEN_SUBSCRIBE_ALL);

	if (log_pantilt)
	{
		carmen_pantilt_subscribe_scanmark_message(NULL, (carmen_handler_t) pantilt_scanmark_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_pantilt_subscribe_laserpos_message(NULL, (carmen_handler_t) pantilt_laserpos_handler, CARMEN_SUBSCRIBE_ALL);
	}

	if (log_robot_laser)
	{
		carmen_robot_ackerman_subscribe_frontlaser_message(NULL, (carmen_handler_t) robot_frontlaser_ackerman_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_robot_ackerman_subscribe_rearlaser_message(NULL, (carmen_handler_t) robot_rearlaser_ackerman_handler, CARMEN_SUBSCRIBE_ALL);
	}

	if (log_laser)
	{
		carmen_laser_subscribe_laser1_message(NULL, (carmen_handler_t) laser_laser1_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_laser_subscribe_laser2_message(NULL, (carmen_handler_t) laser_laser2_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_laser_subscribe_laser3_message(NULL, (carmen_handler_t) laser_laser3_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_laser_subscribe_laser4_message(NULL, (carmen_handler_t) laser_laser4_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_laser_subscribe_laser5_message(NULL, (carmen_handler_t) laser_laser5_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_laser_subscribe_ldmrs_message(NULL, (carmen_handler_t) laser_ldmrs_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_laser_subscribe_ldmrs_new_message(NULL, (carmen_handler_t) laser_ldmrs_new_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_laser_subscribe_ldmrs_objects_message(NULL, (carmen_handler_t) laser_ldmrs_objects_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_laser_subscribe_ldmrs_objects_data_message(NULL, (carmen_handler_t) laser_ldmrs_objects_data_handler, CARMEN_SUBSCRIBE_ALL);
	}

	if (log_velodyne)
	{
		carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) velodyne_variable_scan_handler0, CARMEN_SUBSCRIBE_ALL, 0);
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) velodyne_variable_scan_handler1, CARMEN_SUBSCRIBE_ALL, 1);
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) velodyne_variable_scan_handler2, CARMEN_SUBSCRIBE_ALL, 2);
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) velodyne_variable_scan_handler3, CARMEN_SUBSCRIBE_ALL, 3);
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) velodyne_variable_scan_handler4 , CARMEN_SUBSCRIBE_ALL, 4 );
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) velodyne_variable_scan_handler5 , CARMEN_SUBSCRIBE_ALL, 5 );
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) velodyne_variable_scan_handler6 , CARMEN_SUBSCRIBE_ALL, 6 );
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) velodyne_variable_scan_handler7 , CARMEN_SUBSCRIBE_ALL, 7 );
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) velodyne_variable_scan_handler8 , CARMEN_SUBSCRIBE_ALL, 8 );
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) velodyne_variable_scan_handler9 , CARMEN_SUBSCRIBE_ALL, 9 );
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) velodyne_variable_scan_handler10, CARMEN_SUBSCRIBE_ALL, 10);
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) velodyne_variable_scan_handler11, CARMEN_SUBSCRIBE_ALL, 11);
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) velodyne_variable_scan_handler12, CARMEN_SUBSCRIBE_ALL, 12);
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) velodyne_variable_scan_handler13, CARMEN_SUBSCRIBE_ALL, 13);
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) velodyne_variable_scan_handler14, CARMEN_SUBSCRIBE_ALL, 14);
		carmen_velodyne_subscribe_variable_scan_message(NULL, (carmen_handler_t) velodyne_variable_scan_handler15, CARMEN_SUBSCRIBE_ALL, 15);
		carmen_velodyne_subscribe_gps_message(NULL, (carmen_handler_t) velodyne_gps_handler, CARMEN_SUBSCRIBE_ALL);
	}

	if (log_localize)
		{
			carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_handler, CARMEN_SUBSCRIBE_ALL);
		}

	if (log_simulator)
		carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t) carmen_simulator_ackerman_truepos_handler, CARMEN_SUBSCRIBE_ALL);

	if (log_imu)
		carmen_imu_subscribe_imu_message(NULL, (carmen_handler_t) imu_handler, CARMEN_SUBSCRIBE_ALL);

	if (log_gps)
	{
		carmen_gps_subscribe_nmea_message(NULL, (carmen_handler_t) ipc_gps_gpgga_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_gps_subscribe_nmea_hdt_message(NULL, (carmen_handler_t) ipc_gps_gphdt_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_gps_subscribe_nmea_rmc_message(NULL, (carmen_handler_t) ipc_gps_gprmc_handler, CARMEN_SUBSCRIBE_ALL);
	}

	if (log_motioncmds)
	{
		carmen_robot_ackerman_subscribe_vector_move_message(NULL, (carmen_handler_t) robot_ackerman_vector_move_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_robot_ackerman_subscribe_follow_trajectory_message(NULL, (carmen_handler_t) robot_ackerman_follow_trajectory_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_base_ackerman_subscribe_motion_command(NULL, (carmen_handler_t) base_ackerman_motion_handler, CARMEN_SUBSCRIBE_ALL);
	}

	if (log_bumblebee)
	{
		carmen_bumblebee_basic_subscribe_stereoimage(1, NULL, (carmen_handler_t) bumblebee1_basic_stereoimage_handler, CARMEN_SUBSCRIBE_ALL); // TODO nao deveriam ser todas CARMEN_SUBSCRIBE_LATEST???
		carmen_bumblebee_basic_subscribe_stereoimage(2, NULL, (carmen_handler_t) bumblebee2_basic_stereoimage_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_bumblebee_basic_subscribe_stereoimage(3, NULL, (carmen_handler_t) bumblebee3_basic_stereoimage_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_bumblebee_basic_subscribe_stereoimage(4, NULL, (carmen_handler_t) bumblebee4_basic_stereoimage_handler, CARMEN_SUBSCRIBE_LATEST);
		carmen_bumblebee_basic_subscribe_stereoimage(5, NULL, (carmen_handler_t) bumblebee5_basic_stereoimage_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_bumblebee_basic_subscribe_stereoimage(6, NULL, (carmen_handler_t) bumblebee6_basic_stereoimage_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_bumblebee_basic_subscribe_stereoimage(7, NULL, (carmen_handler_t) bumblebee7_basic_stereoimage_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_bumblebee_basic_subscribe_stereoimage(8, NULL, (carmen_handler_t) bumblebee8_basic_stereoimage_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_bumblebee_basic_subscribe_stereoimage(9, NULL, (carmen_handler_t) bumblebee9_basic_stereoimage_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_bumblebee_basic_subscribe_stereoimage(10, NULL, (carmen_handler_t) bumblebee10_basic_stereoimage_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_bumblebee_basic_subscribe_stereoimage(11, NULL, (carmen_handler_t) bumblebee11_basic_stereoimage_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_bumblebee_basic_subscribe_stereoimage(12, NULL, (carmen_handler_t) bumblebee12_basic_stereoimage_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_bumblebee_basic_subscribe_stereoimage(13, NULL, (carmen_handler_t) bumblebee13_basic_stereoimage_handler, CARMEN_SUBSCRIBE_ALL);
	}

	if (log_camera)
	{
		camera_drivers_subscribe_message(1, NULL, (carmen_handler_t) camera1_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(2, NULL, (carmen_handler_t) camera2_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(3, NULL, (carmen_handler_t) camera3_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(4, NULL, (carmen_handler_t) camera4_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(5, NULL, (carmen_handler_t) camera5_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(6, NULL, (carmen_handler_t) camera6_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(7, NULL, (carmen_handler_t) camera7_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(8, NULL, (carmen_handler_t) camera8_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(9, NULL, (carmen_handler_t) camera9_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(10, NULL, (carmen_handler_t) camera10_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(11, NULL, (carmen_handler_t) camera11_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(12, NULL, (carmen_handler_t) camera12_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(13, NULL, (carmen_handler_t) camera13_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(14, NULL, (carmen_handler_t) camera14_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(15, NULL, (carmen_handler_t) camera15_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(16, NULL, (carmen_handler_t) camera16_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(17, NULL, (carmen_handler_t) camera17_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(18, NULL, (carmen_handler_t) camera18_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(19, NULL, (carmen_handler_t) camera19_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(20, NULL, (carmen_handler_t) camera20_handler, CARMEN_SUBSCRIBE_LATEST);
	}

	if (log_kinect)
	{
		int num_kinect_devices = 0;

		carmen_param_t kinect_num_devs[] = { { "kinect", "num_kinect_devices", CARMEN_PARAM_INT, &num_kinect_devices, 0, NULL } };

		carmen_param_install_params(argc, argv, kinect_num_devs, sizeof(kinect_num_devs) / sizeof(kinect_num_devs[0]));

		if (num_kinect_devices > 0)
		{
			carmen_kinect_subscribe_depth_message(0, NULL, (carmen_handler_t) ipc_kinect_0_depth_handler, CARMEN_SUBSCRIBE_ALL);
			carmen_kinect_subscribe_video_message(0, NULL, (carmen_handler_t) ipc_kinect_0_video_handler, CARMEN_SUBSCRIBE_ALL);
		}

		if (num_kinect_devices > 1)
		{
			carmen_kinect_subscribe_depth_message(1, NULL, (carmen_handler_t) ipc_kinect_1_depth_handler, CARMEN_SUBSCRIBE_ALL);
			carmen_kinect_subscribe_video_message(1, NULL, (carmen_handler_t) ipc_kinect_1_video_handler, CARMEN_SUBSCRIBE_ALL);
		}
	}

	if (log_xsens)
	{
		carmen_xsens_subscribe_xsens_global_matrix_message(NULL, (carmen_handler_t) xsens_matrix_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_xsens_subscribe_xsens_global_euler_message(NULL, (carmen_handler_t) xsens_euler_handler, CARMEN_SUBSCRIBE_ALL);
		carmen_xsens_subscribe_xsens_global_quat_message(NULL, (carmen_handler_t) xsens_quat_handler, CARMEN_SUBSCRIBE_ALL);
	}

	if (log_xsens_mtig)
		carmen_xsens_mtig_subscribe_message(NULL, (carmen_handler_t) xsens_mtig_handler, CARMEN_SUBSCRIBE_ALL);

	if (log_web_cam)
		carmen_web_cam_subscribe_message(NULL, (carmen_handler_t) carmen_web_cam_message_handler, CARMEN_SUBSCRIBE_ALL);

	carmen_logger_subscribe_comment_message(NULL, (carmen_handler_t) logger_comment_handler, CARMEN_SUBSCRIBE_ALL);

	if (log_visual_odometry)
		carmen_visual_odometry_subscribe_pose6d_message(NULL, (carmen_handler_t) visual_odometry_handler, CARMEN_SUBSCRIBE_ALL);

	if (log_imu_pi)
		carmen_pi_imu_subscribe(NULL, (carmen_handler_t) pi_imu_handler, CARMEN_SUBSCRIBE_ALL);

	if (log_sonar)
		carmen_ultrasonic_sonar_sensor_subscribe(NULL, (carmen_handler_t) ultrasonic_sonar_sensor_handler, CARMEN_SUBSCRIBE_ALL);

	if (log_ford_escape_status)
		carmen_ford_escape_subscribe_status_message(NULL, (carmen_handler_t) ford_escape_status_message_handler, CARMEN_SUBSCRIBE_ALL);

	if (log_can_dump)
		carmen_can_dump_subscribe_can_line_message(NULL, (carmen_handler_t) can_dump_message_handler, CARMEN_SUBSCRIBE_ALL);

	signal(SIGINT, shutdown_module);
	logger_starttime = carmen_get_time();
	carmen_ipc_dispatch();

	return (0);
}
