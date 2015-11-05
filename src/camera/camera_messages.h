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

/** @addtogroup camera **/
// @{

/** \file camera_messages.h
 * \brief Definition of the messages for this module.
 *
 * This file specifies the messages for this modules used to transmit
 * data via ipc to other modules.
 **/


#ifndef CARMEN_CAMERA_MESSAGES_H
#define CARMEN_CAMERA_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

  /** This message is published by all the camera_xxxxxcam modules. */

typedef struct {
  int width;                    /**<The x dimension of the image in pixels. */
  int height;                   /**<The y dimension of the image in pixels. */
  int bytes_per_pixel;          /**<Usually 3 (RGB). */ 
  int image_size;               /**<width*height*bytes_per_pixel. */ 
  char *image;                
  double timestamp;
  char *host;
} carmen_camera_image_message;

#define      CARMEN_CAMERA_IMAGE_NAME       "carmen_camera_image"
#define      CARMEN_CAMERA_IMAGE_FMT        "{int,int,int,int,<char:4>,double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
