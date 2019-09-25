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

/** @addtogroup base **/
// @{

/** \file base_messages.h
 * \brief Definition of the messages for this module.
 *
 * This file specifies the messages for this modules used to transmit
 * data via ipc to other modules.
 **/


#ifndef CARMEN_BASE_MESSAGES_H
#define CARMEN_BASE_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

// These messages are generic format messages

#define CARMEN_BASE_RESET_OCCURRED_NAME "carmen_base_reset_occurred"
typedef carmen_default_message carmen_base_reset_occurred_message;
#define CARMEN_BASE_RESET_COMMAND_NAME "carmen_base_reset_command"
typedef carmen_default_message carmen_base_reset_command_message;

typedef struct {
  double x, y, theta;
  double tv, rv;
  double acceleration;
  double timestamp;
  char *host;
} carmen_base_odometry_message;

#define      CARMEN_BASE_ODOMETRY_NAME       "carmen_base_odometry"
#define      CARMEN_BASE_ODOMETRY_FMT        "{double,double,double,double,double,double,double,string}"

typedef struct {
  double tv, rv;
  double timestamp;
  char *host;
} carmen_base_velocity_message;

#define      CARMEN_BASE_VELOCITY_NAME       "carmen_base_velocity"
#define      CARMEN_BASE_VELOCITY_FMT        "{double,double,double,string}"

typedef struct {
  double xv;
  double yv;
  double rv;
  double timestamp;
  char *host;
} carmen_base_holonomic_velocity_message;

#define      CARMEN_BASE_HOLONOMIC_VELOCITY_NAME       "carmen_base_holonomic_velocity"
#define      CARMEN_BASE_HOLONOMIC_VELOCITY_FMT        "{double,double,double,double,string}"

typedef struct {
  int num_sonars;
  double cone_angle;                    //width of sonar cone
  double *range;
  carmen_point_t* sonar_offsets;
  double timestamp;
  char *host;
} carmen_base_sonar_message;

#define      CARMEN_BASE_SONAR_NAME          "carmen_base_sonar"
#define      CARMEN_BASE_SONAR_FMT           "{int,double,<double:1>,<{double,double,double}:1>,double,string}"

typedef struct {
  int rate;
  int num_sonars;
  int *order;
  carmen_point_t *sonar_offsets;
  double timestamp;
  char *host;
} carmen_base_sonar_conf_message;

#define      CARMEN_BASE_SONAR_CONF_NAME          "carmen_base_sonar_conf"
#define      CARMEN_BASE_SONAR_CONF_FMT           "{int,int,<int:2>,<{double,double,double}:2>,double,string}"

typedef struct {
  int num_bumpers;
  unsigned char *state;
  carmen_position_t *bumper_offsets;
  double timestamp;
  char *host;
} carmen_base_bumper_message;

#define      CARMEN_BASE_BUMPER_NAME          "carmen_base_bumper"
#define      CARMEN_BASE_BUMPER_FMT           "{int,<ubyte:1>,<{double,double}:1>,double,string}"

typedef struct {
  int num_irs;
  double *range;
  double timestamp;
  char *host;
} carmen_base_ir_message;

#define      CARMEN_BASE_IR_NAME          "carmen_base_ir"
#define      CARMEN_BASE_IR_FMT           "{int,<double:1>,double,string}"

typedef struct {
  int history;
  int num_irs;
  int order[32];
  double timestamp;
  char *host;
} carmen_base_ir_conf_message;

#define      CARMEN_BASE_IR_CONF_NAME          "carmen_base_ir_conf"
#define      CARMEN_BASE_IR_CONF_FMT           "{int,int,[int:32],double,string}"

typedef struct {
  unsigned char *data;
  int size;
  double timestamp;
  char *host;
} carmen_base_binary_data_message;

#define CARMEN_BASE_BINARY_COMMAND_NAME "carmen_base_binary_command"
#define CARMEN_BASE_BINARY_COMMAND_FMT "{<char:2>,int,double,string}"

#define CARMEN_BASE_BINARY_DATA_NAME "carmen_base_binary_data"
#define CARMEN_BASE_BINARY_DATA_FMT "{<char:2>,int,double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}

