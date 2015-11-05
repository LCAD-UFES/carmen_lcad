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

#ifndef MAP_ACKERMAN_MODIFY_H
#define MAP_ACKERMAN_MODIFY_H

#include "navigator_ackerman.h"

#ifdef __cplusplus
extern "C" {
#endif

  void map_modify_update(carmen_robot_ackerman_laser_message *laser_msg, 
			 carmen_navigator_config_t *navigator_config,
			 carmen_world_point_p world_point, 
			 carmen_map_p true_map, carmen_map_p modify_map);
  void map_modify_clear(carmen_map_p true_map, carmen_map_p modify_map);

#ifdef __cplusplus
}
#endif

#endif
