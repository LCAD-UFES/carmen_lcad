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


/** @addtogroup camera libcamera_hw_interface **/
// @{

/** \file camera_hw_interface.h
 * \brief Definition of the interface for the camera hardware.
 *
 * Definition of the interface for the camera hardware.
 **/

 
#ifndef CARMEN_CAMERA_HW_INTERFACE_H
#define CARMEN_CAMERA_HW_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif
  
  typedef struct {
    int width;
    int height;
    int bytes_per_pixel;
    int image_size;
    char *image;
    int is_new;
    double timestamp;
  } carmen_camera_image_t;
  
  carmen_camera_image_t *carmen_camera_start(int argc, char **argv);
  void carmen_camera_grab_image(carmen_camera_image_t *image);
  void carmen_camera_shutdown(void);

#ifdef __cplusplus
}
#endif

#endif
// @}
