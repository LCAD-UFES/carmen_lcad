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

/** @addtogroup logger libplayback_interface  **/
// @{

/** \file playback_interface.h
 * \brief Definition of the interface for playback.
 *
 * This file specifies the intreface for this modules used to transmit
 * data via ipc to other modules.
 **/

#ifndef CARMEN_PLAYBACK_INTERFACE_H
#define CARMEN_PLAYBACK_INTERFACE_H

#include "carmen/playback_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

/**  Playback a command with playback **/
void carmen_playback_command(int pCommand, int pArgument, float speed);

void carmen_subscribe_playback_info_message (carmen_playback_info_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);
void carmen_unsubscribe_playback_info_message (carmen_handler_t handler);
void carmen_playback_define_messages ();


#ifdef __cplusplus
}
#endif

#endif

// @}
