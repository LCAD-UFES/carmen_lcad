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

/** \file segway_messages.h
 * \brief Definition of the messages for this module.
 *
 * This file specifies the messages for this modules used to transmit
 * data via ipc to other modules.
 **/

#ifndef SEGWAY_MESSAGES_H
#define SEGWAY_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  double timestamp;
  char *host;
  double pitch, pitch_rate;
  double roll, roll_rate;
  double lw_velocity, rw_velocity;
  double tv, rv;
  double x, y, theta;
} carmen_segway_pose_message;

#define CARMEN_SEGWAY_POSE_NAME "carmen_segway_pose"
#define CARMEN_SEGWAY_POSE_FMT "{double,string,double,double,double,double,double,double,double,double,double,double,double}"

typedef struct {
  double timestamp;
  char *host;
  int percent;
} carmen_segway_battery_message;

#define CARMEN_SEGWAY_BATTERY_NAME "carmen_segway_battery"
#define CARMEN_SEGWAY_BATTERY_FMT  "{double,string,int}"

#define CARMEN_SEGWAY_KILL_NAME "carmen_segway_kill"
#define CARMEN_SEGWAY_KILL_FMT  ""

#ifdef __cplusplus
}
#endif

#endif
