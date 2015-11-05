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


/** @addtogroup proccontrol **/
// @{

/** \file proccontrol_messages.h
 * \brief Definition of the messages for this module.
 *
 * This file specifies the messages for this modules used to transmit
 * data via ipc to other modules.
 **/

#ifndef CARMEN_PROCCONTROL_MESSAGES_H
#define CARMEN_PROCCONTROL_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  char *group_name;
  int requested_state;
  double timestamp;
  char *host;
} carmen_proccontrol_groupset_message;

#define     CARMEN_PROCCONTROL_GROUPSET_NAME    "carmen_proccontrol_groupset"
#define     CARMEN_PROCCONTROL_GROUPSET_FMT     "{string,int,double,string}"

typedef struct {
  char *module_name;
  int requested_state;
  double timestamp;
  char *host;
} carmen_proccontrol_moduleset_message;

#define     CARMEN_PROCCONTROL_MODULESET_NAME   "carmen_proccontrol_moduleset"
#define     CARMEN_PROCCONTROL_MODULESET_FMT    "{string,int,double,string}"

typedef struct {
  char *group_name, *module_name;
  int active, requested_state, pid;
} carmen_proccontrol_process_t, *carmen_proccontrol_process_p;

typedef struct {
  int num_processes;
  carmen_proccontrol_process_p process;
  double timestamp;
  char *host;
} carmen_proccontrol_pidtable_message;

#define     CARMEN_PROCCONTROL_PIDTABLE_NAME     "carmen_proccontrol_pidtable"
#define     CARMEN_PROCCONTROL_PIDTABLE_FMT      "{int,<{string,string,int,int,int}:1>,double,string}"

typedef struct {
  int pid;
  char *output;
  double timestamp;
  char *host;
} carmen_proccontrol_output_message;

#define     CARMEN_PROCCONTROL_OUTPUT_NAME       "carmen_proccontrol_output"
#define     CARMEN_PROCCONTROL_OUTPUT_FMT        "{int,string,double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
