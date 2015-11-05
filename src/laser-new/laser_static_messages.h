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

/** \file laser_static_messages.h
 * \brief Definition of the static messages for the module laser (internal use only)
 *
 * 
 * Keep the same stucture as in laser_messages.h
 **/

#ifndef CARMEN_LASER_STATIC_MESSAGES_H
#define CARMEN_LASER_STATIC_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "laser_messages.h"

  /* Keep the same stucture as in laser_messages.h (exept of the static array) */

#define CARMEN_LASER_LASER_STATIC_MESSAGE_MAXREADINGS 2048
typedef struct {
  int id;
  carmen_laser_laser_config_t config;
  int num_readings;
  float range[CARMEN_LASER_LASER_STATIC_MESSAGE_MAXREADINGS];
  int num_remissions;
  float remission[CARMEN_LASER_LASER_STATIC_MESSAGE_MAXREADINGS];
  double timestamp;
  char *host;
} carmen_laser_laser_static_message;

 
#ifdef __cplusplus
}
#endif

#endif

// @}
