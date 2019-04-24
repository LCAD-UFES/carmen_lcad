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
#include "sick.h"
#include "laser.h"
#include "laser_ipc.h"
#include "laser_messages.h"

sick_laser_t laser1, laser2, laser3, laser4, laser5;

carmen_laser_laser_config_t laser1_config, laser2_config, laser3_config, laser4_config, laser5_config;

int use_laser1 = 0, use_laser2 = 0;
int use_laser3 = 0, use_laser4 = 0;
int use_laser5 = 0;
int quit_signal = 0;

void set_default_parameters(sick_laser_p laser, int laser_num)
{
  laser->settings.type = LMS;
  laser->settings.range_res = CM;
  laser->settings.range_dist = SICK_RANGE80M;
  laser->settings.laser_num = laser_num;
  strcpy(laser->settings.device_name, "/dev/ttyS0");
  laser->settings.detect_baudrate = TRUE;
  laser->settings.use_highspeed = FALSE;
  laser->settings.start_baudrate = 9600;
  laser->settings.set_baudrate = 38400;
  laser->settings.databits = 8;
  laser->settings.parity = N;
  laser->settings.stopbits = 1;
  laser->settings.hwf = 0;
  laser->settings.swf = 0;
  laser->settings.angle_range = 180;
  laser->settings.angle_resolution = RES_1_00_DEGREE;
  laser->settings.laser_flipped = 0;
  laser->settings.use_remission = 0;
}

void check_parameter_settings(sick_laser_p laser)
{
  /*********************** TYPE CHECKING **************************/
  if(laser->settings.type == PLS) {
    strncpy((char *)laser->settings.password, (const char *)PLS_PASSWORD, 8);
    laser->settings.parity = E;
  } 
  if(laser->settings.type == LMS) {
    strncpy((char *)laser->settings.password, (const char *)LMS_PASSWORD, 8);
    laser->settings.parity = N;
  } 
  
  /*********************** START BAUDRATE **************************/
  if(laser->settings.detect_baudrate)
    laser->settings.start_baudrate = 9600;
  else if(laser->settings.start_baudrate != 9600 &&
	  laser->settings.start_baudrate != 19200 &&
	  laser->settings.start_baudrate != 38400 &&
	  laser->settings.start_baudrate != 500000) {
    fprintf(stderr, "ERROR: start baudrate = %d is not valid!\n",
	    laser->settings.start_baudrate);
    exit(1);
  }

  /*********************** SET BAUDRATE **************************/
  if(laser->settings.set_baudrate != 9600 &&
     laser->settings.set_baudrate != 19200 &&
     laser->settings.set_baudrate != 38400 &&
     laser->settings.set_baudrate != 500000) {
    fprintf(stderr, "ERROR: set baudrate = %d is not valid!\n",
	    laser->settings.set_baudrate);
    exit(1);
  } 
  else if(laser->settings.set_baudrate == 500000)
    laser->settings.use_highspeed = TRUE;
  
  /*********************** NUM VALUES **************************/
  if(laser->settings.angle_range != 180 && 
     laser->settings.angle_range != 100) {
    fprintf(stderr, "ERROR: angle range = %d is not valid!\n",
	    laser->settings.angle_range);
    exit(1);
  }
  
  /************************** ANGLE RANGE ************************/
  if(laser->settings.angle_range == 100) {
    if(laser->settings.angle_resolution == RES_1_00_DEGREE)
      laser->settings.num_values = 101;
    else if(laser->settings.angle_resolution == RES_0_50_DEGREE)
      laser->settings.num_values = 201;
    else if(laser->settings.type == LMS)
      laser->settings.num_values = 401;
    else
      fprintf(stderr, "ERROR: ang-res=0.25 is not valid for this laser!\n");
  }
  else {
    if(laser->settings.angle_resolution == RES_1_00_DEGREE)
      laser->settings.num_values = 181;
    else if(laser->settings.angle_resolution == RES_0_50_DEGREE)
      laser->settings.num_values = 361;
    else {
      fprintf(stderr, "ERROR: ang-res=0.25 and ang-range=180 is not valid!\n");
      exit(1);
    }
  }
  if(laser->settings.type == PLS) {
    if(laser->settings.angle_range == 100) {
      fprintf(stderr, "ERROR: type = PLS and ang-range=100 is not valid!\n");
      exit(1);
    } 
  }


  /********************** REMISSION RANGE ************************/
  /* remission values - start */
  if (laser->settings.use_remission == 1 && 
      laser->settings.type != LMS) {
    fprintf(stderr, "ERROR: remission values are only available using LMS laser!\n");
    exit(1);
  }
  if (laser->settings.use_remission == 1 && 
      laser->settings.angle_resolution != RES_1_00_DEGREE) {
    fprintf(stderr, "ERROR: remission values are only available with 1.0 degree resolution!\n");
    exit(1);
  }

  if(laser->settings.use_remission == 1) 
    laser->settings.rem_values = laser->settings.num_values;
  else 
    laser->settings.rem_values = 0;
  /* remission values - stop */
}

void interpret_params(sick_laser_p laser, char *dev, char *type, double res, char *rem, double fov)
{
  strcpy(laser->settings.device_name, dev);
  if(strcmp(type, "LMS") == 0)
    laser->settings.type = LMS;
  else if(strcmp(type, "PLS") == 0)
    laser->settings.type = PLS;
  
  if (fabs(fov-M_PI) < 0.1 || fabs(fov-100.0/180.0*M_PI) < 0.1)
    carmen_die("The parameter laser_laserX_fov in the ini file must\nbe specified in degrees not in radians!\n");
  
  laser->settings.angle_range = carmen_round(fov);
  
  if ( laser->settings.angle_range != 180 && 
       laser->settings.angle_range != 100 )
    carmen_die("The laser driver only provides 180 deg and 100 deg field of view!\n");

  if(res == 0.25) {
    laser->settings.angle_resolution = RES_0_25_DEGREE;
    laser->settings.angle_range = 100;
  }
  else if(res == 0.5)
    laser->settings.angle_resolution = RES_0_50_DEGREE;
  else
    laser->settings.angle_resolution = RES_1_00_DEGREE;

  /* remission values - start */
  if(strcmp(rem, "direct") == 0) {
    laser->settings.use_remission = 1;
    laser->settings.range_dist = SICK_REMISSION_DIRECT;
  }
  else if(strcmp(rem, "normalized") == 0) {
    laser->settings.use_remission = 1;
    laser->settings.range_dist = SICK_REMISSION_NORM;
  }
  else if(strcmp(rem, "no") == 0) {
    laser->settings.use_remission = 0;
  }
  else if(strcmp(rem, "off") == 0) {
    laser->settings.use_remission = 0;
    carmen_warn("Warning: please set the value of the parameter \nlaser_use_remission to \"no\" and do not use \"off\".\nAssuming \"no\" for now.\n\n");
  }
  else carmen_die("ERROR: Parameter laser_use_remission for laser %d has invalid value: %s\nPossible values are: direct, normalized and off.\n", laser->settings.laser_num, rem);
  /* remission values - stop */

  check_parameter_settings(laser);
}

void read_parameters(int argc, char **argv)
{
  char *dev1, *dev2, *dev3, *dev4, *dev5;
  char *str1, *str2, *str3, *str4, *str5;
  double res1, res2, res3, res4, res5;
  double fov1, fov2, fov3, fov4, fov5;
  char *rem1, *rem2, *rem3, *rem4, *rem5;

  carmen_param_t laser_devs[] = {
    {"laser", "front_laser_dev", CARMEN_PARAM_STRING, &dev1, 0, NULL},
    {"laser", "rear_laser_dev", CARMEN_PARAM_STRING, &dev2, 0, NULL},
    {"laser", "laser3_dev", CARMEN_PARAM_STRING, &dev3, 0, NULL},
    {"laser", "laser4_dev", CARMEN_PARAM_STRING, &dev4, 0, NULL},
    {"laser", "laser5_dev", CARMEN_PARAM_STRING, &dev5, 0, NULL}};
  carmen_param_t laser1_params[] = {
    {"laser", "front_laser_type", CARMEN_PARAM_STRING, &str1, 0, NULL},
    {"laser", "front_laser_resolution", CARMEN_PARAM_DOUBLE, &res1, 0, NULL},
    {"laser", "front_laser_use_remission", CARMEN_PARAM_STRING, &rem1, 0, NULL},
    {"laser", "front_laser_fov", CARMEN_PARAM_DOUBLE,  &fov1, 0, NULL},
    {"laser", "front_laser_baud", CARMEN_PARAM_INT,  
     &laser1.settings.set_baudrate, 0, NULL},
    {"laser", "front_laser_flipped", CARMEN_PARAM_INT, 
     &laser1.settings.laser_flipped, 0, NULL}};
  carmen_param_t laser2_params[] = {
    {"laser", "rear_laser_type", CARMEN_PARAM_STRING, &str2, 0, NULL},
    {"laser", "rear_laser_resolution", CARMEN_PARAM_DOUBLE, &res2, 0, NULL},
    {"laser", "rear_laser_use_remission", CARMEN_PARAM_STRING, &rem2, 0, NULL},
    {"laser", "rear_laser_fov", CARMEN_PARAM_DOUBLE, &fov2, 0, NULL},
    {"laser", "rear_laser_baud", CARMEN_PARAM_INT, 
     &laser2.settings.set_baudrate, 0, NULL},
    {"laser", "rear_laser_flipped", CARMEN_PARAM_INT, 
     &laser2.settings.laser_flipped, 0, NULL}};
  carmen_param_t laser3_params[] = {
    {"laser", "laser3_type", CARMEN_PARAM_STRING, &str3, 0, NULL},
    {"laser", "laser3_resolution", CARMEN_PARAM_DOUBLE, &res3, 0, NULL},
    {"laser", "laser3_use_remission", CARMEN_PARAM_STRING, &rem3, 0, NULL},
    {"laser", "laser3_fov", CARMEN_PARAM_DOUBLE, &fov3, 0, NULL},
    {"laser", "laser3_baud", CARMEN_PARAM_INT, 
     &laser3.settings.set_baudrate, 0, NULL},
    {"laser", "laser3_flipped", CARMEN_PARAM_INT, 
     &laser3.settings.laser_flipped, 0, NULL}};
  carmen_param_t laser4_params[] = {
    {"laser", "laser4_type", CARMEN_PARAM_STRING, &str4, 0, NULL},
    {"laser", "laser4_resolution", CARMEN_PARAM_DOUBLE, &res4, 0, NULL},
    {"laser", "laser4_use_remission", CARMEN_PARAM_STRING, &rem4, 0, NULL},
    {"laser", "laser4_fov", CARMEN_PARAM_DOUBLE, &fov4, 0, NULL},
    {"laser", "laser4_baud", CARMEN_PARAM_INT, 
     &laser4.settings.set_baudrate, 0, NULL},
    {"laser", "laser4_flipped", CARMEN_PARAM_INT, 
     &laser4.settings.laser_flipped, 0, NULL}};
  carmen_param_t laser5_params[] = {
    {"laser", "laser5_type", CARMEN_PARAM_STRING, &str5, 0, NULL},
    {"laser", "laser5_resolution", CARMEN_PARAM_DOUBLE, &res5, 0, NULL},
    {"laser", "laser5_use_remission", CARMEN_PARAM_STRING, &rem5, 0, NULL},
    {"laser", "laser5_fov", CARMEN_PARAM_DOUBLE, &fov5, 0, NULL},
    {"laser", "laser5_baud", CARMEN_PARAM_INT, 
     &laser5.settings.set_baudrate, 0, NULL},
    {"laser", "laser5_flipped", CARMEN_PARAM_INT, 
     &laser5.settings.laser_flipped, 0, NULL}};


  carmen_param_install_params(argc, argv, laser_devs, 
			      sizeof(laser_devs) / sizeof(laser_devs[0]));

  if(strncmp(dev1, "none", 4) != 0) {
    use_laser1 = 1;
    carmen_param_install_params(argc, argv, laser1_params,
				sizeof(laser1_params) / 
				sizeof(laser1_params[0]));
    interpret_params(&laser1, dev1, str1, res1, rem1, fov1);
  }
  if(strncmp(dev2, "none", 4) != 0) {
    use_laser2 = 1;
    carmen_param_install_params(argc, argv, laser2_params,
				sizeof(laser2_params) / 
				sizeof(laser2_params[0]));
    interpret_params(&laser2, dev2, str2, res2, rem2, fov2);
  }
  if(strncmp(dev3, "none", 4) != 0) {
    use_laser3 = 1;
    carmen_param_install_params(argc, argv, laser3_params,
				sizeof(laser3_params) / 
				sizeof(laser3_params[0]));
    interpret_params(&laser3, dev3, str3, res3, rem3, fov3);
  }
  if(strncmp(dev4, "none", 4) != 0) {
    use_laser4 = 1;
    carmen_param_install_params(argc, argv, laser4_params,
				sizeof(laser4_params) / 
				sizeof(laser4_params[0]));
    interpret_params(&laser4, dev4, str4, res4, rem4, fov4);
  }
  if(strncmp(dev5, "none", 4) != 0) {
    use_laser5 = 1;
    carmen_param_install_params(argc, argv, laser5_params,
				sizeof(laser5_params) / 
				sizeof(laser5_params[0]));
    interpret_params(&laser5, dev5, str5, res5, rem5, fov5);
  }
}

void  set_laser_config_structure(sick_laser_p laser,
				 carmen_laser_laser_config_t* config) {

  if (laser->settings.type == LMS) {
    config->laser_type = SICK_LMS;
    config->maximum_range = 81.90;

    if (laser->settings.range_res == MM)
      config->accuracy = 0.035;  /* 5cm in cm mode, 35mm in mm mode */
    else
      config->accuracy = 0.05;   /* 5cm in cm mode, 35mm in mm mode */
  }
  else if (laser->settings.type == PLS) {
    config->laser_type = SICK_PLS;
    config->maximum_range = 50.0;
    config->accuracy = 0.15; /* I need to look up this value in the SICK specs */
  }
  else { 
    // if unknown, assume LMS
    config->laser_type = SICK_LMS;
    config->maximum_range = 81.90;

    if (laser->settings.range_res == MM)
      config->accuracy = 0.035;  /* 5cm in cm mode, 35mm in mm mode */
    else
      config->accuracy = 0.05;   /* 5cm in cm mode, 35mm in mm mode */
  }

  if (laser->settings.num_values == 181 ) {
    config->angular_resolution = carmen_degrees_to_radians(1.0); 
    config->fov  = M_PI;
    config->start_angle = -0.5*config->fov;
  } 
  else if (laser->settings.num_values == 361 ) {
    config->angular_resolution = carmen_degrees_to_radians(0.5); 
    config->fov  = M_PI;
    config->start_angle = -0.5*config->fov;
  } 
  else if (laser->settings.num_values == 180 ) {
    config->angular_resolution = carmen_degrees_to_radians(1.0);
    config->fov  = M_PI - config->angular_resolution;
    config->start_angle = -0.5*M_PI;
  } 
  else if (laser->settings.num_values == 360 ) {
    config->angular_resolution = carmen_degrees_to_radians(0.5);
    config->fov  = M_PI - config->angular_resolution;
    config->start_angle = -0.5*M_PI;
  } 
  else   if (laser->settings.num_values == 401 ) {
    config->angular_resolution = carmen_degrees_to_radians(0.25);
    config->fov  = carmen_degrees_to_radians(100.0);
    config->start_angle = -0.5*carmen_degrees_to_radians(100.0);
  } 
  else   if (laser->settings.num_values == 201 ) {
    config->angular_resolution = carmen_degrees_to_radians(0.5);
    config->fov  = carmen_degrees_to_radians(100.0);
    config->start_angle = -0.5*carmen_degrees_to_radians(100.0);
  } 
  else   if (laser->settings.num_values == 101 ) {
    config->angular_resolution = carmen_degrees_to_radians(1.0);
    config->fov  = carmen_degrees_to_radians(100.0);
    config->start_angle = -0.5*carmen_degrees_to_radians(100.0);
  } 
  else   if (laser->settings.num_values == 400 ) {
    config->angular_resolution = carmen_degrees_to_radians(0.25);
    config->fov  = carmen_degrees_to_radians(100.0) - config->angular_resolution;
    config->start_angle = -0.5*carmen_degrees_to_radians(100.0);
  } 
  else   if (laser->settings.num_values == 200 ) {
    config->angular_resolution = carmen_degrees_to_radians(0.5);
    config->fov  = carmen_degrees_to_radians(100.0) - config->angular_resolution;
    config->start_angle = -0.5*carmen_degrees_to_radians(100.0);
  } 
  else   if (laser->settings.num_values == 100 ) {
    config->angular_resolution = carmen_degrees_to_radians(1.0);
    config->fov  = carmen_degrees_to_radians(100.0) - config->angular_resolution;
    config->start_angle = -0.5*carmen_degrees_to_radians(100.0);
  } 
  else {
    config->fov  = M_PI;
    config->start_angle = -0.5*config->fov;
    config->angular_resolution = config->fov/((double)laser->settings.laser_num-1.0);

    carmen_warn("Unkown laser config for a SICK with %d beams\n", laser->settings.num_values);
    carmen_warn("Guessing: (fov=%.3f, start=%.2f, res=%.2f)\n", config->fov, config->start_angle, config->angular_resolution);

  }

  if (laser->settings.use_remission == 1 && 
      laser->settings.range_dist == SICK_REMISSION_DIRECT) 
    config->remission_mode = REMISSION_DIRECT;
  else   if (laser->settings.use_remission == 1 && 
      laser->settings.range_dist == SICK_REMISSION_NORM) 
    config->remission_mode = REMISSION_NORMALIZED;
  else
    config->remission_mode = REMISSION_NONE;

}

int carmen_laser_start(int argc, char **argv)
{
  /* initialize laser messages */
  ipc_initialize_messages();

  /* get laser parameters */
  set_default_parameters(&laser1, CARMEN_FRONT_LASER_NUM);  
  set_default_parameters(&laser2, CARMEN_REAR_LASER_NUM);
  set_default_parameters(&laser3, CARMEN_LASER3_NUM);
  set_default_parameters(&laser4, CARMEN_LASER4_NUM);
  set_default_parameters(&laser5, CARMEN_LASER5_NUM);
  read_parameters(argc, argv);
  

  /* start lasers, and start publishing scans */
  if(use_laser1) {
    set_laser_config_structure(&laser1, &laser1_config);
    sick_start_laser(&laser1);
  }
  if(use_laser2) {
    set_laser_config_structure(&laser2, &laser2_config);
    sick_start_laser(&laser2);
  }
  if(use_laser3) {
    set_laser_config_structure(&laser3, &laser3_config);
    sick_start_laser(&laser3);
  }
  if(use_laser4) {
    set_laser_config_structure(&laser4, &laser4_config);
    sick_start_laser(&laser4);
  }
  if(use_laser5) {
    set_laser_config_structure(&laser5, &laser5_config);
    sick_start_laser(&laser5);
  }
  return 0;
}

void carmen_laser_shutdown(int signo __attribute__ ((unused)))
{
  if(use_laser1)
    sick_stop_laser(&laser1);
  if(use_laser2)
    sick_stop_laser(&laser2);
  if(use_laser3)
    sick_stop_laser(&laser3);
  if(use_laser4)
    sick_stop_laser(&laser4);
  if(use_laser5)
    sick_stop_laser(&laser5);
}

int carmen_laser_run(void)
{
  static int first = 1;
  static double last_update;
  static double last_alive = 0;
  double current_time;
  int print_stats;
  static int laser1_stalled = 0, laser2_stalled = 0, laser3_stalled = 0,
    laser4_stalled = 0,    laser5_stalled = 0;;

  if(first) {
    last_update = carmen_get_time();
    first = 0;
  }
  current_time = carmen_get_time();
  print_stats = (current_time - last_update > 1.0);
  if(use_laser1) {
    sick_handle_laser(&laser1);
    if(laser1.new_reading)
      publish_laser_message(&laser1, &laser1_config);   
    laser1_stalled = (current_time - laser1.timestamp > 1.0);
    //**
	//fprintf(stderr, "time: %.1f",current_time - laser1.timestamp);
	//**
    if(print_stats) 
      fprintf(stderr, "L1: %s(%.1f%% full) ", laser1_stalled ?
	      "STALLED " : " ", (laser1.buffer_position - 
				 laser1.processed_mark) / 
	      (float)LASER_BUFFER_SIZE * 100.0);
  }
  if(use_laser2) {
    sick_handle_laser(&laser2);
    if(laser2.new_reading)
      publish_laser_message(&laser2, &laser2_config);
    laser2_stalled = (current_time - laser2.timestamp > 1.0);
    if(print_stats) 
      fprintf(stderr, "L2: %s(%.1f%% full) ", laser2_stalled ?
	      "STALLED " : " ", (laser2.buffer_position - 
				 laser2.processed_mark) / 
	      (float)LASER_BUFFER_SIZE * 100.0);
  }
  if(use_laser3) {
    sick_handle_laser(&laser3);
    if(laser3.new_reading)
      publish_laser_message(&laser3, &laser3_config);   
    laser3_stalled = (current_time - laser3.timestamp > 1.0);
    if(print_stats) 
      fprintf(stderr, "L3: %s(%.1f%% full) ", laser3_stalled ?
	      "STALLED " : " ", laser3.buffer_position / 
	      (float)LASER_BUFFER_SIZE * 100.0);
  }
  if(use_laser4) {
    sick_handle_laser(&laser4);
    if(laser4.new_reading)
      publish_laser_message(&laser4, &laser4_config);   
    laser4_stalled = (current_time - laser4.timestamp > 1.0);
    if(print_stats) 
      fprintf(stderr, "L4: %s(%.1f%% full) ", laser4_stalled ?
	      "STALLED " : " ", laser4.buffer_position / 
	      (float)LASER_BUFFER_SIZE * 100.0);
  }
  if(use_laser5) {
    sick_handle_laser(&laser5);
    if(laser5.new_reading)
      publish_laser_message(&laser5, &laser5_config);   
    laser5_stalled = (current_time - laser5.timestamp > 1.0);
    if(print_stats) 
      fprintf(stderr, "L5: %s(%.1f%% full) ", laser5_stalled ?
	      "STALLED " : " ", laser5.buffer_position / 
	      (float)LASER_BUFFER_SIZE * 100.0);
  }
  if(print_stats) {
    fprintf(stderr, "\n");
    last_update = current_time;
  }
  if(current_time - last_alive > 1.0) {
    publish_laser_alive(laser1_stalled, laser2_stalled, laser3_stalled,
			laser4_stalled, laser5_stalled);
    last_alive = current_time;
  }

  carmen_publish_heartbeat("laser");

  return 0;
}

void shutdown_laser(int x)
{
  carmen_laser_shutdown(x);
  carmen_ipc_disconnect();
  exit(-1);
}

