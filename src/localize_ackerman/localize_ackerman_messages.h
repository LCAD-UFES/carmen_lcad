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

/** @addtogroup localize **/
// @{

/** \file localize_messages.h
 * \brief Definition of the messages for this module.
 *
 * This file specifies the messages for this modules used to transmit
 * data via ipc to other modules.
 **/


#ifndef CARMEN_LOCALIZE_ACKERMAN_MESSAGES_H
#define CARMEN_LOCALIZE_ACKERMAN_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Initialization message for localize */

#define CARMEN_INITIALIZE_UNIFORM     1
#define CARMEN_INITIALIZE_GAUSSIAN    2

typedef struct {
  int distribution;
  int num_modes;
  carmen_point_t *mean, *std;
  double timestamp;
  char *host;
} carmen_localize_ackerman_initialize_message;

#define CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_NAME  "carmen_localize_ackerman_initialize"
#define CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_FMT   "{int,int,<{double,double,double}:2>,<{double,double,double}:2>,double,string}"

  /* initialize by map placename */

typedef struct {
  char *placename;
  double timestamp;
  char *host;
} carmen_localize_ackerman_initialize_placename_message;

#define CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_PLACENAME_NAME "carmen_localize_ackerman_initialize_placename"
#define CARMEN_LOCALIZE_ACKERMAN_INITIALIZE_PLACENAME_FMT "{string,double,string}"

/* Contains the mean and standard deviation of the position of the robot */

typedef struct {
  carmen_point_t globalpos, globalpos_std, odometrypos;
  carmen_pose_3D_t pose;
  carmen_vector_3D_t velocity;
  double v, phi;
  double globalpos_xy_cov;
  int converged;
  int semi_trailer_engaged;
  int semi_trailer_type;
  double beta;
  double timestamp;
  char *host;
} carmen_localize_ackerman_globalpos_message;

#define CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME "carmen_localize_ackerman_globalpos"
#define CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_FMT  "{{double,double,double},{double,double,double},{double,double,double},{{double,double,double},{double,double,double}},{double,double,double},double,double,double,int,int,int,double,double,string}"

/* particle message */

typedef struct {
  double x, y, theta, v, phi, phi_bias;
  double weight;
} carmen_localize_ackerman_particle_ipc_t, *carmen_localize_ackerman_particle_ipc_p;

typedef struct {
  int num_particles;
  carmen_localize_ackerman_particle_ipc_p particles;
  carmen_point_t globalpos, globalpos_std;
  double globalpos_xy_cov;
  double timestamp;
  char *host;
} carmen_localize_ackerman_particle_message;

#define CARMEN_LOCALIZE_ACKERMAN_PARTICLE_NAME "carmen_localize_ackerman_particle"
#define CARMEN_LOCALIZE_ACKERMAN_PARTICLE_FMT  "{int,<{double,double,double,double,double,double,double}:1>,{double,double,double},{double,double,double},double,double,string}"

#define CARMEN_LOCALIZE_ACKERMAN_PARTICLE_PREDICTION_NAME "carmen_localize_ackerman_particle_prediction"
#define CARMEN_LOCALIZE_ACKERMAN_PARTICLE_CORRECTION_NAME "carmen_localize_ackerman_particle_correction"

/* sensor message in localize coordinates */

typedef struct {
  carmen_laser_laser_config_t config;
  int num_readings, laser_skip;
  double *range;
  char *mask;
  carmen_point_t pose;
  int num_laser;
  double timestamp;
  char *host;
} carmen_localize_ackerman_sensor_message;

#define CARMEN_LOCALIZE_ACKERMAN_SENSOR_NAME "carmen_localize_ackerman_sensor"
#define CARMEN_LOCALIZE_ACKERMAN_SENSOR_FMT  "{{int,double,double,double,double,double,int},int,int,<double:2>,<char:2>,{double,double,double},int,double,string}"

typedef struct {
  int map_is_global_likelihood;
  double timestamp;
  char *host;
} carmen_localize_ackerman_map_query_message;
  
#define CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_NAME "carmen_localize_ackerman_map_query"
#define CARMEN_LOCALIZE_ACKERMAN_MAP_QUERY_FMT "{int,double,string}"
  
typedef struct {
  unsigned char *data;    
  int size;
  carmen_map_config_t config;
  int compressed;
  int map_is_global_likelihood;
  double timestamp;
  char *host;
} carmen_localize_ackerman_map_message;  
  
#define CARMEN_LOCALIZE_ACKERMAN_MAP_NAME "carmen_localize_ackerman_map"
#define CARMEN_LOCALIZE_ACKERMAN_MAP_FMT  "{<char:2>,int,{int,int,double,[byte:64],string, double, double},int,int,double,string}"

#define CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_QUERY_NAME "carmen_localize_ackerman_globalpos_query"
typedef carmen_default_message carmen_localize_ackerman_globalpos_query_message;

#ifdef __cplusplus
}
#endif

#endif
// @}
