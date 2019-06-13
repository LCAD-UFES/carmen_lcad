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

/** @addtogroup robot **/
// @{

/** \file robot_messages.h
 * \brief Definition of the messages for this module.
 *
 * This file specifies the messages for this modules used to transmit
 * data via ipc to other modules.
 **/


#ifndef CARMEN_ROBOT_ACKERMAN_MESSAGES_H
#define CARMEN_ROBOT_ACKERMAN_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/laser_messages.h>


  /** Sonar measurement message **/
typedef struct {
  int num_sonars;
  double cone_angle;               /**< width of sonar cone **/
  double *ranges;
  carmen_point_t *sonar_offsets;
  carmen_point_t robot_pose;
  double v, phi;
  double timestamp;
  char *host;                         /**< The host from which this message was sent **/
} carmen_robot_ackerman_sonar_message;
#define      CARMEN_ROBOT_ACKERMAN_SONAR_NAME            "robot_ackerman_sonar"
#define      CARMEN_ROBOT_ACKERMAN_SONAR_FMT             "{int,double,<double:1>,<{double,double,double}:1>,{double,double,double},double,double,double,string}"


  /** Bumper measurement message **/
typedef struct { 
  int num_bumpers;
  char *state;
  carmen_point_t robot_pose; //position of the center of the robot
  double v, phi;
  double timestamp;
  char *host;                          /**< The host from which this message was sent **/
} carmen_robot_ackerman_bumper_message;
#define CARMEN_ROBOT_ACKERMAN_BUMPER_NAME "carmen_robot_ackerman_bumper"
#define CARMEN_ROBOT_ACKERMAN_BUMPER_FMT "{int, <char:1>, {double,double,double}, double, double, double, string}"

  
#ifndef COMPILE_WITHOUT_LASER_SUPPORT
  /** Laser measurement message with interpolated pose estimate of the robot **/
typedef struct {
  int id;
  carmen_laser_laser_config_t config;   /**< Configuration of the laser sensor **/
  int num_readings;                     /**< Number of beams in this message **/
  double *range;                         /**< Array of proximity measurements **/
  char *tooclose;                       /**< Is the robot too close to an obstacle?  **/
  int num_remissions;                   /**< Number of remission values (0 = no remission available **/
  double *remission;                    /**< Array of remission measurements **/
  carmen_point_t laser_pose;           /**< Position of the center of the laser **/
  carmen_point_t robot_pose;           /**< Position of the center of the robot **/
  double v, phi;                       /**< Translational and rotational velocities **/
  double forward_safety_dist, side_safety_dist;
  double turn_axis;
  double timestamp;                    /**< Timestamp when the laser data was recorded (received by the devide driver) **/
  char *host;                          /**< The host from which this message was sent **/
} carmen_robot_ackerman_laser_message;
#define      CARMEN_ROBOT_ACKERMAN_LASER_FMT             "{int,{int,double,double,double,double,double,int},int,<double:3>,<char:3>,int,<double:6>,{double,double,double},{double,double,double},double,double,double,double,double,double,string}"
#define      CARMEN_ROBOT_ACKERMAN_FRONTLASER_NAME       "carmen_robot_ackerman_frontlaser"
#define      CARMEN_ROBOT_ACKERMAN_FRONTLASER_FMT        CARMEN_ROBOT_ACKERMAN_LASER_FMT

#define      CARMEN_ROBOT_ACKERMAN_REARLASER_NAME        "carmen_robot_ackerman_rearlaser"
#define      CARMEN_ROBOT_ACKERMAN_REARLASER_FMT         CARMEN_ROBOT_ACKERMAN_LASER_FMT
#endif


typedef struct {
  double vector_distance;
  double vector_angle;
  double timestamp;
  char *host;                   /**< The host from which this message was sent **/
} carmen_robot_ackerman_vector_status_message;
#define CARMEN_ROBOT_ACKERMAN_VECTOR_STATUS_NAME "carmen_robot_ackerman_vector_status"
#define CARMEN_ROBOT_ACKERMAN_VECTOR_STATUS_FMT "{double, double, double, string}"
  

typedef struct {
  double v;
  double phi;
  double timestamp;
  char *host;                   /**< The host from which this message was sent **/
} carmen_robot_ackerman_velocity_message;
#define      CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME         "carmen_robot_ackerman_vel"
#define      CARMEN_ROBOT_ACKERMAN_VELOCITY_FMT          "{double,double,double,string}"


// Message redefined to insert a module between the obstacle_avoider and the ford_escape_hybrid
#define      CARMEN_ROBOT_ACKERMAN_VELOCITY_2_NAME         "carmen_robot_ackerman_vel_2"
#define      CARMEN_ROBOT_ACKERMAN_VELOCITY_2_FMT          "{double,double,double,string}"


typedef struct {
  int trajectory_length;
  carmen_ackerman_traj_point_t *trajectory;
  carmen_ackerman_traj_point_t robot_position;
  double timestamp;
  char *host;                 /**< The host from which this message was sent **/
} carmen_robot_ackerman_follow_trajectory_message;
#define      CARMEN_ROBOT_ACKERMAN_FOLLOW_TRAJECTORY_NAME         "carmen_robot_ackerman_follow_trajectory"
#define      CARMEN_ROBOT_ACKERMAN_FOLLOW_TRAJECTORY_FMT          "{int,<{double,double,double,double,double}:1>,{double,double,double,double,double},double,string}"


typedef struct {
  double distance;
  double theta;
  double timestamp;
  char *host;               /**< The host from which this message was sent **/
} carmen_robot_ackerman_vector_move_message;
#define      CARMEN_ROBOT_ACKERMAN_VECTOR_MOVE_NAME         "carmen_robot_ackerman_vector_move"
#define      CARMEN_ROBOT_ACKERMAN_VECTOR_MOVE_FMT          "{double,double,double,string}"


typedef struct 
{
	int num_motion_commands;
	carmen_ackerman_motion_command_p motion_command;
	double timestamp;
	char *host;                 /**< The host from which this message was sent **/
} carmen_robot_ackerman_motion_command_message;
#define      CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_NAME		"carmen_robot_ackerman_motion_command"
#define      CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_FMT		"{int,<{double,double,double,double,double,double}:1>,double,string}"

// This message has the same structure and format of the traditional motion command message,
// but a different name. It is used to send motion commands to some module, but not to the
// rest of control hierarchy and interfaces.
#define      CARMEN_ROBOT_ACKERMAN_TEACHER_MOTION_COMMAND_NAME		"carmen_robot_ackerman_teacher_motion_command"
#define      CARMEN_ROBOT_ACKERMAN_TEACHER_MOTION_COMMAND_FMT		"{int,<{double,double,double,double,double,double}:1>,double,string}"

#define      CARMEN_OBSTACLE_AVOIDER_PATH_NAME			"carmen_obstacle_avoider_path"
#define      CARMEN_OBSTACLE_AVOIDER_PATH_FMT			CARMEN_NAVIGATOR_ACKERMAN_PLAN_FMT

#define      CARMEN_OBSTACLE_AVOIDER_MOTION_PLANNER_PATH_NAME	"carmen_obstacle_avoider_motion_planner_path"
#define      CARMEN_OBSTACLE_AVOIDER_MOTION_PLANNER_PATH_FMT	CARMEN_NAVIGATOR_ACKERMAN_PLAN_FMT


#ifdef __cplusplus
}
#endif

#endif

// @}
