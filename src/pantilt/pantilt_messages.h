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


/** @addtogroup pantilt **/
// @{

/** \file pantilt_messages.h
 * \brief Definition of the messages for this module.
 *
 * This file specifies the messages for this modules used to transmit
 * data via ipc to other modules.
 **/

#ifndef CARMEN_PANTILT_MESSAGES_H
#define CARMEN_PANTILT_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  double  pan;
  double  tilt;
  double  timestamp;
  char*   host;
} carmen_pantilt_status_message;

#define      CARMEN_PANTILT_STATUS_MESSAGE_NAME  "pantilt_status_message"
#define      CARMEN_PANTILT_STATUS_MESSAGE_FMT   "{double, double, double, string}"


typedef struct {
  double  pan;
  double  tilt;
  double  timestamp;
  char*   host;
} carmen_pantilt_move_message;

#define      CARMEN_PANTILT_MOVE_MESSAGE_NAME  "pantilt_move_message"
#define      CARMEN_PANTILT_MOVE_MESSAGE_FMT   "{double,double,double,string}"

typedef struct {
  double pan;
  double timestamp;
  char*  host;
} carmen_pantilt_move_pan_message;

#define      CARMEN_PANTILT_MOVE_PAN_MESSAGE_NAME  "pantilt_move_pan_message"
#define      CARMEN_PANTILT_MOVE_PAN_MESSAGE_FMT   "{double,double,string}"

typedef struct {
  double  tilt;
  double  timestamp;
  char*   host;
 } carmen_pantilt_move_tilt_message;

#define      CARMEN_PANTILT_MOVE_TILT_MESSAGE_NAME  "pantilt_move_tilt_message"
#define      CARMEN_PANTILT_MOVE_TILT_MESSAGE_FMT   "{double,double,string}"


typedef struct {
  double pan_vel;
  double tilt_vel;
  double timestamp;
  char* host;
} carmen_pantilt_set_velocity_message;
#define CARMEN_PANTILT_SET_VEL_NAME       "pantilt_set_vel"
#define CARMEN_PANTILT_SET_VEL_FMT        "{ double, double, double, string }"

typedef struct {
  double pan_acc;
  double tilt_acc;
  double timestamp;
  char* host;
} carmen_pantilt_set_acceleration_message;
#define CARMEN_PANTILT_SET_ACC_NAME       "pantilt_set_acc"
#define CARMEN_PANTILT_SET_ACC_FMT        "{ double, double, double, string }"



#define CARMEN_PANTILT_RESET_NAME         "pantilt_reset"
#define CARMEN_PANTILT_PAN_HOME_NAME   "pantilt_pan_home"
#define CARMEN_PANTILT_TILT_HOME_NAME  "pantilt_tilt_home"
#define CARMEN_PANTILT_HALT_NAME          "pantilt_halt"
typedef carmen_default_message carmen_pantilt_reset_message;
typedef carmen_default_message carmen_pantilt_pan_home_message;
typedef carmen_default_message carmen_pantilt_tilt_home_message;
typedef carmen_default_message carmen_pantilt_halt_message;


  /*
   * scanmarks are sent before and after a 3D-scan is taken.
   * This way a scan can easily be extracted from incoming data, logfiles etc.
   *
   * type:    0 -> begin of scan
   *          1 -> end of scan
   * laserid: id of laser that is used for scanning
   *
   */

#define SCANMARK_START 0
#define SCANMARK_STOP  1


typedef struct {
  int  type;
  int  laserid;
  double  timestamp;
  char*   host;
} carmen_pantilt_scanmark_message;

#define      CARMEN_PANTILT_SCANMARK_MESSAGE_NAME  "pantilt_scanmark_message"
#define      CARMEN_PANTILT_SCANMARK_MESSAGE_FMT   "{int, int, double, string}"



  /*
   * laserpos messages state the 3D-position of a laser
   * relative to the robot's center. This is can be used for 
   * 3D-scanning.
   *
   * x,y,z,phi,theta,psi: 
   * 3D-pos. of starting point of laser beams
   *
   * id: id of laser
   *
   */
  
  typedef struct {
    int id;
    double x;
    double y;
    double z;
    double phi;
    double theta;
    double psi;
    double timestamp;
    char *host;
  } carmen_pantilt_laserpos_message;
  
#define      CARMEN_PANTILT_LASERPOS_MESSAGE_NAME  "pantilt_laserpos_message"
#define      CARMEN_PANTILT_LASERPOS_MESSAGE_FMT  "{int,double,double,double,double,double,double,double,string}"
  
  
#ifdef __cplusplus
}
#endif

#endif

// @}
