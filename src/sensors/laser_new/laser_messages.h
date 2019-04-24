/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, and Sebastian Thrun
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

/** @addtogroup laser **/
// @{

/** \file laser_messages.h
 * \brief Definition of the messages for the module laser.
 *
 * This file specifies the messages for this modules used to transmit
 * data via ipc to other modules.
 **/

#ifndef CARMEN_LASER_MESSAGES_H
#define CARMEN_LASER_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

/** supported laser types     **/
typedef enum {
	SICK_LMS                  = 0, 
	SICK_PLS                  = 1, 
	HOKUYO_URG                = 2, 
	SIMULATED_LASER           = 3, 
	SICK_S300                 = 4, 
	LASER_EMULATED_USING_KINECT		= 5,
	LASER_EMULATED_USING_BUMBLEBEE	= 6,
	UMKNOWN_PROXIMITY_SENSOR  = 99
} carmen_laser_laser_type_t;

/** Possible remission values **/
typedef enum {
	REMISSION_NONE       = 0, 
	REMISSION_DIRECT     = 1, 
	REMISSION_NORMALIZED = 2
} carmen_laser_remission_type_t;

/** The laser message of the laser module (rawlaser) **/
typedef struct {
  carmen_laser_laser_type_t  laser_type;  /**< what kind of laser is this **/
  double start_angle;                     /**< angle of the first beam relative **/
                                          /**< to to the center of the laser **/
  double fov;                             /**< field of view of the laser **/
  double angular_resolution;              /**< angular resolution of the laser **/
  double maximum_range;                   /**< the maximum valid range of a measurement  **/
  double accuracy;                        /**< error in the range measurements **/
  carmen_laser_remission_type_t remission_mode;  /* if and what kind of remission values are used */

} carmen_laser_laser_config_t;


typedef struct {
  int id;
  carmen_laser_laser_config_t config;
  int num_readings;
  double *range;
  int num_remissions;
  double *remission;
  double timestamp;
  char *host;
} carmen_laser_laser_message;

#define CARMEN_LASER_LASER_FMT  "{int,{int,double,double,double,double,double,int},int,<double:3>,int,<double:5>,double,string}"

#define      CARMEN_LASER_LASER1_NAME            "carmen_laser_laser1"
#define      CARMEN_LASER_LASER2_NAME            "carmen_laser_laser2"
#define      CARMEN_LASER_LASER3_NAME            "carmen_laser_laser3"
#define      CARMEN_LASER_LASER4_NAME            "carmen_laser_laser4"
#define      CARMEN_LASER_LASER5_NAME            "carmen_laser_laser5"
#define      CARMEN_LASER_LASER6_NAME            "carmen_laser_laser6" //para o front laser do eduardo
#define      CARMEN_LASER_LASER9_NAME            "carmen_laser_laser9" //para o rear laser

#define      CARMEN_LASER_LASER1_FMT               CARMEN_LASER_LASER_FMT
#define      CARMEN_LASER_LASER2_FMT               CARMEN_LASER_LASER_FMT
#define      CARMEN_LASER_LASER3_FMT               CARMEN_LASER_LASER_FMT
#define      CARMEN_LASER_LASER4_FMT               CARMEN_LASER_LASER_FMT
#define      CARMEN_LASER_LASER5_FMT               CARMEN_LASER_LASER_FMT

#define      CARMEN_LASER_FRONTLASER_NAME        CARMEN_LASER_LASER1_NAME
#define      CARMEN_LASER_REARLASER_NAME         CARMEN_LASER_LASER9_NAME
#define      CARMEN_LASER_FRONTLASER_2_NAME        CARMEN_LASER_LASER6_NAME
#define      CARMEN_LASER_FRONTLASER_FMT           CARMEN_LASER_LASER_FMT
#define      CARMEN_LASER_REARLASER_FMT            CARMEN_LASER_LASER_FMT


typedef struct {
  int frontlaser_stalled;
  int rearlaser_stalled;
  int laser3_stalled;
  int laser4_stalled;
} carmen_laser_alive_message;
  
#define      CARMEN_LASER_ALIVE_NAME            "carmen_laser_alive"
#define      CARMEN_LASER_ALIVE_FMT             "{int,int,int,int}"

  
#ifdef __cplusplus
}
#endif

#endif

// @}
