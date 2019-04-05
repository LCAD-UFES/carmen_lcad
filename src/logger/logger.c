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
#include <carmen/ultrasonic_filter_interface.h>
#include <carmen/visual_odometry_interface.h>
#include "logger.h"
#include "writelog.h"

char filename[1024];
char filename_without_path[1024];

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
static int log_web_cam = 1;
static int log_bumblebee_frames_to_save = 1;
static int log_bumblebee_save_to_file = 0;
static int log_velodyne_save_to_file = 0;
static int log_ford_escape_status = 0;
static int log_can_dump = 0;

void get_logger_params(int argc, char** argv) {

  int num_items;

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
    {"logger", "web_cam",   			CARMEN_PARAM_ONOFF, &log_web_cam, 0, NULL},
    {"logger", "bumblebee_frames_to_save", CARMEN_PARAM_INT, &log_bumblebee_frames_to_save, 0, NULL},
    {"logger", "sonar",       			CARMEN_PARAM_ONOFF, &log_sonar, 0, NULL},
    {"logger", "velodyne_save_to_file",	CARMEN_PARAM_ONOFF, &log_velodyne_save_to_file, 0, NULL},
    {"logger", "bumblebee_save_to_file", CARMEN_PARAM_ONOFF, &log_bumblebee_save_to_file, 0, NULL},
    {"logger", "ford_escape_status", 	CARMEN_PARAM_ONOFF, &log_ford_escape_status, 0, NULL},
    {"logger", "can_dump", 				CARMEN_PARAM_ONOFF, &log_can_dump, 0, NULL},
  };

  num_items = sizeof(param_list)/sizeof(param_list[0]);
  carmen_param_install_params(argc, argv, param_list, num_items);
}

void get_all_params(void)
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
		carmen_logwrite_write_to_file_velodyne(msg, outfile, carmen_get_time() - logger_starttime, filename);
	else
		carmen_logwrite_write_velodyne_partial_scan(msg, outfile, carmen_get_time() - logger_starttime);

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
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 1, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 1, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}

void bumblebee2_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b2s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 2, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 2, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}

void bumblebee3_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b3s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 3, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 3, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}

void bumblebee4_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b4s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 4, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 4, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}

void bumblebee5_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b5s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 5, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 5, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}

void bumblebee6_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b6s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 6, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 6, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}

void bumblebee7_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b7s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 7, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 7, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}

void bumblebee8_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b8s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 8, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 8, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
}

void bumblebee9_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message* message)
{
	//fprintf(stderr, "b9s");
	if (log_bumblebee_save_to_file)
		carmen_logwrite_write_to_file_bumblebee_basic_steroimage(message, 9, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save, filename);
	else
		carmen_logwrite_write_bumblebee_basic_steroimage(message, 9, outfile, carmen_get_time() - logger_starttime, log_bumblebee_frames_to_save);
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

void shutdown_module(int sig)
{
  if(sig == SIGINT) {
    carmen_fclose(outfile);
    carmen_ipc_disconnect();
    fprintf(stderr, "\nDisconnecting.\n");
    exit(0);
  }
}

int main(int argc, char **argv)
{
  int p;
  char key;

  /* initialize connection to IPC network */
  carmen_ipc_initialize(argc, argv);
  carmen_param_check_version(argv[0]);

  /* open logfile, check if file overwrites something */
  if(argc < 2)
    carmen_die("usage: %s <logfile>\n", argv[0]);

  strcpy(filename, argv[1]);

  for (p = strlen(filename) - 1; p >= 0; p--)
	  if (filename[p] == '/')
		  break;

  strcpy(filename_without_path, filename + p);

  outfile = carmen_fopen(filename, "r");
  if (outfile != NULL) {
    fprintf(stderr, "Overwrite %s? ", filename);
    scanf("%c", &key);
    if (toupper(key) != 'Y')
      exit(-1);
    carmen_fclose(outfile);
  }
  outfile = carmen_fopen(filename, "w");
  if(outfile == NULL)
    carmen_die("Error: Could not open file %s for writing.\n", filename);
  carmen_logwrite_write_header(outfile);


  get_logger_params(argc, argv);

  if  ( !(log_odometry && log_laser && log_robot_laser ) )
    carmen_warn("\nWARNING: You are neither logging laser nor odometry messages!\n");



  if (log_params)
    get_all_params();

  register_ipc_messages();


  if (log_odometry)
  {
		  carmen_robot_ackerman_subscribe_velocity_message( NULL,
				  (carmen_handler_t) robot_ackerman_velocity_handler,
				  CARMEN_SUBSCRIBE_ALL );
  }

    if (log_pantilt) {
    carmen_pantilt_subscribe_scanmark_message (NULL, (carmen_handler_t)
					       pantilt_scanmark_handler,
					       CARMEN_SUBSCRIBE_ALL);

    carmen_pantilt_subscribe_laserpos_message (NULL, (carmen_handler_t)
					       pantilt_laserpos_handler,
					       CARMEN_SUBSCRIBE_ALL);
  }

  if (log_robot_laser) {
		  carmen_robot_ackerman_subscribe_frontlaser_message(NULL, (carmen_handler_t)
				  robot_frontlaser_ackerman_handler,
				  CARMEN_SUBSCRIBE_ALL);
		  carmen_robot_ackerman_subscribe_rearlaser_message(NULL, (carmen_handler_t)
				  robot_rearlaser_ackerman_handler,
				  CARMEN_SUBSCRIBE_ALL);
  }


  if (log_laser) {
    carmen_laser_subscribe_laser1_message(NULL, (carmen_handler_t)
					  laser_laser1_handler,
					  CARMEN_SUBSCRIBE_ALL);
    carmen_laser_subscribe_laser2_message(NULL, (carmen_handler_t)
					  laser_laser2_handler,
					  CARMEN_SUBSCRIBE_ALL);
    carmen_laser_subscribe_laser3_message(NULL, (carmen_handler_t)
					  laser_laser3_handler,
					  CARMEN_SUBSCRIBE_ALL);
    carmen_laser_subscribe_laser4_message(NULL, (carmen_handler_t)
					  laser_laser4_handler,
					  CARMEN_SUBSCRIBE_ALL);
    carmen_laser_subscribe_laser5_message(NULL, (carmen_handler_t)
					  laser_laser5_handler,
					  CARMEN_SUBSCRIBE_ALL);
    carmen_laser_subscribe_ldmrs_message(NULL, (carmen_handler_t)
					  laser_ldmrs_handler,
					  CARMEN_SUBSCRIBE_ALL);
    carmen_laser_subscribe_ldmrs_new_message(NULL, (carmen_handler_t)
					  laser_ldmrs_new_handler,
					  CARMEN_SUBSCRIBE_ALL);
    carmen_laser_subscribe_ldmrs_objects_message(NULL, (carmen_handler_t)
					  laser_ldmrs_objects_handler,
					  CARMEN_SUBSCRIBE_ALL);
    carmen_laser_subscribe_ldmrs_objects_data_message(NULL, (carmen_handler_t)
					  laser_ldmrs_objects_data_handler,
					  CARMEN_SUBSCRIBE_ALL);
  }

  if (log_velodyne){
	  carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t)
  			velodyne_partial_scan_handler,
  			CARMEN_SUBSCRIBE_ALL);

	  carmen_velodyne_subscribe_gps_message(NULL, (carmen_handler_t)
				 velodyne_gps_handler,
				 CARMEN_SUBSCRIBE_ALL);
  }

  if (log_localize) {
		  carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t)
				  localize_ackerman_handler,
				  CARMEN_SUBSCRIBE_ALL);
  }


  if (log_simulator) {

		  carmen_simulator_ackerman_subscribe_truepos_message(NULL, (carmen_handler_t)
				  carmen_simulator_ackerman_truepos_handler,
				  CARMEN_SUBSCRIBE_ALL);
  }


  if (log_imu) {

    carmen_imu_subscribe_imu_message(NULL, (carmen_handler_t)
				     imu_handler,
				     CARMEN_SUBSCRIBE_ALL);
  }


  if (log_gps) {
    carmen_gps_subscribe_nmea_message( NULL,
				       (carmen_handler_t) ipc_gps_gpgga_handler,
				       CARMEN_SUBSCRIBE_ALL );

    carmen_gps_subscribe_nmea_hdt_message( NULL,
				       (carmen_handler_t) ipc_gps_gphdt_handler,
				       CARMEN_SUBSCRIBE_ALL );

    carmen_gps_subscribe_nmea_rmc_message( NULL,
					   (carmen_handler_t) ipc_gps_gprmc_handler,
					   CARMEN_SUBSCRIBE_ALL );
  }

  if (log_motioncmds) {
		  carmen_robot_ackerman_subscribe_vector_move_message( NULL,
				  (carmen_handler_t) robot_ackerman_vector_move_handler,
				  CARMEN_SUBSCRIBE_ALL );
		  carmen_robot_ackerman_subscribe_follow_trajectory_message( NULL,
				  (carmen_handler_t) robot_ackerman_follow_trajectory_handler,
				  CARMEN_SUBSCRIBE_ALL );
		  carmen_base_ackerman_subscribe_motion_command( NULL,
				  (carmen_handler_t) base_ackerman_motion_handler,
				  CARMEN_SUBSCRIBE_ALL );
  }

  if (log_bumblebee)
  {
	carmen_bumblebee_basic_subscribe_stereoimage(1, NULL,
			       (carmen_handler_t) bumblebee1_basic_stereoimage_handler,
			       CARMEN_SUBSCRIBE_ALL);

	carmen_bumblebee_basic_subscribe_stereoimage(2, NULL,
			       (carmen_handler_t) bumblebee2_basic_stereoimage_handler,
			       CARMEN_SUBSCRIBE_ALL);

	carmen_bumblebee_basic_subscribe_stereoimage(3, NULL,
			(carmen_handler_t) bumblebee3_basic_stereoimage_handler,
			CARMEN_SUBSCRIBE_ALL);

	carmen_bumblebee_basic_subscribe_stereoimage(4, NULL,
			(carmen_handler_t) bumblebee4_basic_stereoimage_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_bumblebee_basic_subscribe_stereoimage(5, NULL,
			(carmen_handler_t) bumblebee5_basic_stereoimage_handler,
			CARMEN_SUBSCRIBE_ALL);

	carmen_bumblebee_basic_subscribe_stereoimage(6, NULL,
			(carmen_handler_t) bumblebee6_basic_stereoimage_handler,
			CARMEN_SUBSCRIBE_ALL);

	carmen_bumblebee_basic_subscribe_stereoimage(7, NULL,
			(carmen_handler_t) bumblebee7_basic_stereoimage_handler,
			CARMEN_SUBSCRIBE_ALL);

	carmen_bumblebee_basic_subscribe_stereoimage(8, NULL,
			(carmen_handler_t) bumblebee8_basic_stereoimage_handler,
			CARMEN_SUBSCRIBE_ALL);

	carmen_bumblebee_basic_subscribe_stereoimage(9, NULL,
			(carmen_handler_t) bumblebee9_basic_stereoimage_handler,
			CARMEN_SUBSCRIBE_ALL);
  }

  if (log_kinect)
  {
	  int num_kinect_devices = 0;

	  carmen_param_t kinect_num_devs[] = {
			  {"kinect", "num_kinect_devices", CARMEN_PARAM_INT, &num_kinect_devices, 0, NULL}
	  };

	  carmen_param_install_params(argc, argv, kinect_num_devs,
			  sizeof(kinect_num_devs) / sizeof(kinect_num_devs[0]));

	  if(num_kinect_devices > 0) {
		  carmen_kinect_subscribe_depth_message(0, NULL,
				  (carmen_handler_t) ipc_kinect_0_depth_handler,
				  CARMEN_SUBSCRIBE_ALL );

		  carmen_kinect_subscribe_video_message(0, NULL,
				  (carmen_handler_t) ipc_kinect_0_video_handler,
				  CARMEN_SUBSCRIBE_ALL );
	  }

	  if(num_kinect_devices > 1) {
		  carmen_kinect_subscribe_depth_message(1, NULL,
				  (carmen_handler_t) ipc_kinect_1_depth_handler,
				  CARMEN_SUBSCRIBE_ALL );

		  carmen_kinect_subscribe_video_message(1, NULL,
				  (carmen_handler_t) ipc_kinect_1_video_handler,
				  CARMEN_SUBSCRIBE_ALL );
	  }
  }

  if(log_xsens){


    carmen_xsens_subscribe_xsens_global_matrix_message(NULL,
					    (carmen_handler_t) xsens_matrix_handler,
					    CARMEN_SUBSCRIBE_ALL );

    carmen_xsens_subscribe_xsens_global_euler_message(NULL,
					    (carmen_handler_t) xsens_euler_handler,
					    CARMEN_SUBSCRIBE_ALL );

    carmen_xsens_subscribe_xsens_global_quat_message(NULL,
					    (carmen_handler_t) xsens_quat_handler,
					    CARMEN_SUBSCRIBE_ALL );
  }

  if(log_xsens_mtig){

	carmen_xsens_mtig_subscribe_message(	NULL,
		      		(carmen_handler_t) xsens_mtig_handler,
		            CARMEN_SUBSCRIBE_ALL);

  }

  if (log_web_cam)
  {
	  carmen_web_cam_subscribe_message (NULL,
		  (carmen_handler_t) carmen_web_cam_message_handler,
		  CARMEN_SUBSCRIBE_ALL);
  }

  carmen_logger_subscribe_comment_message( NULL,
		  (carmen_handler_t) logger_comment_handler,
		  CARMEN_SUBSCRIBE_ALL );

  if (log_visual_odometry)
  {
	  carmen_visual_odometry_subscribe_pose6d_message(NULL,
		 (carmen_handler_t) visual_odometry_handler,
		 CARMEN_SUBSCRIBE_ALL);

  }

  if (log_imu_pi)
  {
	  carmen_pi_imu_subscribe(NULL,
			  (carmen_handler_t) pi_imu_handler,
			  CARMEN_SUBSCRIBE_ALL);
  }

  if (log_sonar)
  {
	  carmen_ultrasonic_sonar_sensor_subscribe(NULL,
		 (carmen_handler_t) ultrasonic_sonar_sensor_handler,
		 CARMEN_SUBSCRIBE_ALL);
  }

  if (log_ford_escape_status)
  {
	  carmen_ford_escape_subscribe_status_message(NULL,
		 (carmen_handler_t) ford_escape_status_message_handler,
		 CARMEN_SUBSCRIBE_ALL);
  }

  if (log_can_dump)
  {
	  carmen_can_dump_subscribe_can_line_message(NULL,
		 (carmen_handler_t) can_dump_message_handler,
		 CARMEN_SUBSCRIBE_ALL);
  }

  signal(SIGINT, shutdown_module);
  logger_starttime = carmen_get_time();
  carmen_ipc_dispatch();
  return 0;
}
