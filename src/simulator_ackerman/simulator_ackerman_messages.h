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

/** @addtogroup simulator **/
// @{

/** \file simulator_messages.h
 * \brief Definition of the messages for this module.
 *
 * This file specifies the messages for this modules used to transmit
 * data via ipc to other modules.
 **/

#ifndef SIMULATOR_ACKERMAN_MESSAGES_H
#define SIMULATOR_ACKERMAN_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

#define CARMEN_SIMULATOR_ACKERMAN_CLEAR_OBJECTS_NAME   "carmen_simulator_ackerman_clear_objects"
typedef carmen_default_message carmen_simulator_ackerman_clear_objects_message;
#define CARMEN_SIMULATOR_ACKERMAN_NEXT_TICK_NAME       "carmen_simulator_ackerman_next_tick"
typedef carmen_default_message carmen_simulator_ackerman_next_tick_message;
#define CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_QUERY_NAME   "carmen_simulator_ackerman_truepos_query"
typedef carmen_default_message carmen_simulator_ackerman_truepos_query_message;
#define CARMEN_SIMULATOR_ACKERMAN_OBJECTS_QUERY_NAME   "carmen_simulator_ackerman_objects_query"
typedef carmen_default_message carmen_simulator_ackerman_objects_query_message;

typedef enum {CARMEN_SIMULATOR_ACKERMAN_RANDOM_OBJECT, CARMEN_SIMULATOR_ACKERMAN_LINE_FOLLOWER,
	      CARMEN_SIMULATOR_ACKERMAN_OTHER_ROBOT} carmen_simulator_ackerman_object_t;

typedef struct {
  carmen_point_t pose;
  double timestamp;
  char *host;
} carmen_simulator_ackerman_set_truepose_message;

#define CARMEN_SIMULATOR_ACKERMAN_SET_TRUEPOSE_NAME "carmen_simulator_ackerman_set_truepose"
#define CARMEN_SIMULATOR_ACKERMAN_SET_TRUEPOSE_FMT  "{{double,double,double},double,string}"

typedef struct {
  carmen_point_t truepose;
  carmen_point_t odometrypose;
  double v;
  double phi;
  double timestamp;
  char *host;
} carmen_simulator_ackerman_truepos_message;

#define CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME       "carmen_simulator_ackerman_truepos"
#define CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_FMT        "{{double,double,double},{double,double,double},double,double,double,string}"

#define CARMEN_SIMULATOR_ACKERMAN_EXTERNAL_TRUEPOSE_NAME "carmen_simulator_ackerman_external_truepose"
#define CARMEN_SIMULATOR_ACKERMAN_EXTERNAL_TRUEPOSE_FMT  "{{double,double,double},{double,double,double},double,double,double,string}"

typedef struct {
  char *other_central;
  double timestamp;
  char *host;
} carmen_simulator_ackerman_connect_robots_message;

#define CARMEN_SIMULATOR_ACKERMAN_CONNECT_ROBOTS_NAME "carmen_simulator_ackerman_connect_robots"
#define CARMEN_SIMULATOR_ACKERMAN_CONNECT_ROBOTS_FMT  "{string,double,string}"

typedef struct {
  carmen_point_t pose;
  double speed;
  carmen_simulator_ackerman_object_t type;
  double timestamp;
  char *host;
} carmen_simulator_ackerman_set_object_message;

#define CARMEN_SIMULATOR_ACKERMAN_SET_OBJECT_NAME "carmen_simulator_ackerman_set_object"
#define CARMEN_SIMULATOR_ACKERMAN_SET_OBJECT_FMT  "{{double,double,double},double,int,double,string}"

typedef struct {
  int num_objects;
  carmen_traj_point_t *objects_list;//TODO change this?
  double timestamp;
  char *host;
} carmen_simulator_ackerman_objects_message;

#define CARMEN_SIMULATOR_ACKERMAN_OBJECTS_NAME "carmen_simulator_ackerman_objects"
#define CARMEN_SIMULATOR_ACKERMAN_OBJECTS_FMT  "{int,<{double,double,double,double,double}:1>,double,string}"

#ifdef __cplusplus
}
#endif

#endif


// @}
