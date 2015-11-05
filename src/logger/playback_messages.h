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

/** @addtogroup logger  **/
// @{

/** \file playback_messages.h
 * \brief Definition of the messages for  playback.
 *
 * This file specifies the messages for this modules used to transmit
 * data via ipc to other modules.
 **/

#ifndef    CARMEN_PLAYBACK_MESSAGES_H
#define    CARMEN_PLAYBACK_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

#define    CARMEN_PLAYBACK_COMMAND_PLAY         	0
#define    CARMEN_PLAYBACK_COMMAND_STOP         	1
#define    CARMEN_PLAYBACK_COMMAND_RESET        	2
#define    CARMEN_PLAYBACK_COMMAND_FORWARD      	3
#define    CARMEN_PLAYBACK_COMMAND_REWIND       	4
#define    CARMEN_PLAYBACK_COMMAND_FWD_SINGLE   	5
#define    CARMEN_PLAYBACK_COMMAND_RWD_SINGLE   	6
#define    CARMEN_PLAYBACK_COMMAND_SET_SPEED    	7
#define    CARMEN_PLAYBACK_COMMAND_SET_INITIAL_TIME    	8

typedef struct {
  int cmd, arg;
  float speed;
} carmen_playback_command_message;

#define CARMEN_PLAYBACK_COMMAND_NAME     "carmen_playback_command"
#define CARMEN_PLAYBACK_COMMAND_FMT      "{int,int,float}"

typedef struct
{
	int message_number;
	double message_timestamp;
	double message_timestamp_difference;
	double playback_speed;
} carmen_playback_info_message;

#define CARMEN_PLAYBACK_INFO_MESSAGE_NAME	"carmen_playback_info_message"
#define CARMEN_PLAYBACK_INFO_MESSAGE_FMT	"{int,double,double,double}"

#ifdef __cplusplus
}
#endif

#endif

// @}

