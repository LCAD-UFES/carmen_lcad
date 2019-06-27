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

#ifndef BASE_ACKERMAN_MESSAGES_H
#define BASE_ACKERMAN_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	double x, y, theta;
	double v, phi;
	double timestamp;
	char *host;
} carmen_base_ackerman_odometry_message;

#define	CARMEN_BASE_ACKERMAN_ODOMETRY_NAME	"carmen_base_ackerman_odometry"
#define	CARMEN_BASE_ACKERMAN_ODOMETRY_FMT	"{double,double,double,double,double,double,string}"

// Message redefined to insert a module between the obstacle_avoider and the ford_escape_hybrid
#define	CARMEN_BASE_ACKERMAN_ODOMETRY_2_NAME	"carmen_base_ackerman_odometry_2"
#define	CARMEN_BASE_ACKERMAN_ODOMETRY_2_FMT	"{double,double,double,double,double,double,string}"

typedef struct {
	double v, phi;
	double timestamp;
	char *host;
} carmen_base_ackerman_velocity_message;

#define	CARMEN_BASE_ACKERMAN_VELOCITY_NAME	"carmen_base_ackerman_velocity"
#define	CARMEN_BASE_ACKERMAN_VELOCITY_FMT	"{double,double,double,string}"

typedef struct
{
	int num_motion_commands;
	carmen_ackerman_motion_command_p motion_command;
	double timestamp;
	char *host;                 /**< The host from which this message was sent **/
} carmen_base_ackerman_motion_command_message;

#define      CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME         "carmen_base_ackerman_motion_command"
#define      CARMEN_BASE_ACKERMAN_MOTION_COMMAND_FMT          "{int,<{double,double,double,double,double,double}:1>,double,string}"

// Message redefined to insert a module between the obstacle_avoider and the ford_escape_hybrid
#define      CARMEN_BASE_ACKERMAN_MOTION_COMMAND_2_NAME         "carmen_base_ackerman_motion_command_2"
#define      CARMEN_BASE_ACKERMAN_MOTION_COMMAND_2_FMT          "{int,<{double,double,double,double,double,double}:1>,double,string}"


#ifdef __cplusplus
}
#endif

#endif


// @}
