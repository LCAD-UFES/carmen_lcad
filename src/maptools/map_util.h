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

#ifndef CARMEN_MAP_UTIL_H
#define CARMEN_MAP_UTIL_H

void carmen_map_util_change_resolution(carmen_map_p map, double new_resolution);
void carmen_minimize_gridmap(carmen_map_p map, int *x_offset, int *y_offset);
void carmen_minimize_offlimits(carmen_offlimits_list_t *offlimits_list, 
			       double x_offset, double y_offset);
void carmen_minimize_places(carmen_map_placelist_t *place_list, 
			    double x_offset, double y_offset,
			    double width, double height);

void carmen_map_free_hmap(carmen_hmap_p hmap);

void carmen_rotate_gridmap(carmen_map_p map, int rotation);
void carmen_rotate_offlimits(carmen_map_config_t config,
			     carmen_offlimits_list_t *offlimits_list,
			     int rotation);
void carmen_rotate_places(carmen_map_config_t config, 
			  carmen_map_placelist_p place_list, 
			  int rotation);

#endif
