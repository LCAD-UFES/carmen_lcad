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

#ifndef TASK_MANAGER_MESSAGES_H
#define TASK_MANAGER_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

#define ENGAGED		1
#define DISENGAGED	0


typedef struct
{
	int geometry;
	double timestamp;
	char *host;
} carmen_task_manager_set_collision_geometry_message;

#define	CARMEN_TASK_MANAGER_SET_GEOMETRY_MESSAGE_NAME	"carmen_task_manager_set_collision_geometry"
#define	CARMEN_TASK_MANAGER_SET_GEOMETRY_MESSAGE_FMT	"{int,double,string}"

typedef struct
{
	int desired_engage_state;
	double timestamp;
	char *host;
} carmen_task_manager_desired_engage_state_message;

#define	CARMEN_TASK_MANAGER_DESIRED_ENGAGE_STATE_MESSAGE_NAME	"carmen_task_manager_desired_engage_state"
#define	CARMEN_TASK_MANAGER_DESIRED_ENGAGE_STATE_MESSAGE_FMT	"{int,double,string}"

typedef struct
{
	int semi_trailer_type;
	double timestamp;
	char *host;
} carmen_task_manager_set_semi_trailer_type_message;

#define	CARMEN_TASK_MANAGER_SET_SEMI_TRAILER_TYPE_MESSAGE_NAME	"carmen_task_manager_set_semi_trailer_type"
#define	CARMEN_TASK_MANAGER_SET_SEMI_TRAILER_TYPE_MESSAGE_FMT	"{int,double,string}"

#ifdef __cplusplus
}
#endif

#endif


// @}
