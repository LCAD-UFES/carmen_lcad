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

/** \file arm_messages.h
 * \brief Definition of the messages for this module.
 *
 * This file specifies the messages for this modules used to transmit
 * data via ipc to other modules.
 **/


#ifndef CARMEN_ARM_MESSAGES_H
#define CARMEN_ARM_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif


//arm type defines
#define CARMEN_ARM_HAS_ANGULAR_VEL_STATES      1
#define CARMEN_ARM_HAS_CURRENT_STATES          2
#define CARMEN_ARM_HAS_GRIPPER                 4

#define CARMEN_ARM_QUERY_NAME "carmen_arm_query"
typedef carmen_default_message carmen_arm_query_message;
#define CARMEN_ARM_RESET_OCCURRED_NAME "carmen_arm_reset_occurred"
typedef carmen_default_message carmen_arm_reset_occurred_message;
#define CARMEN_ARM_RESET_COMMAND_NAME "carmen_arm_reset_command"
typedef carmen_default_message carmen_arm_reset_command_message;

typedef struct {
  int num_joints;
  double* joint_angles;
  double timestamp;
  char *host;
} carmen_arm_command_message;

#define CARMEN_ARM_COMMAND_NAME  "carmen_arm_command"
#define CARMEN_ARM_COMMAND_FMT "{int,<double:1>,double,string}"

typedef struct {
  int flags;
  int num_joints;
  double* joint_angles;
  int num_currents;
  double* joint_currents;
  int num_vels;
  double* joint_angular_vels;
  int gripper_closed;
  double timestamp;
  char *host;
} carmen_arm_state_message;

#define CARMEN_ARM_STATE_NAME "carmen_arm_state"
#define CARMEN_ARM_STATE_FMT "{int,int,<double:2>,int,<double:4>,int,<double:6>,int,double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
