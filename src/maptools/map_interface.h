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

/** @addtogroup maptools libmap_interface **/
// @{

/** \file map_interface.h
 * \brief Library of function for mapserver clients.
 *
 * Library of function for mapserver clients.
 **/

#ifndef CARMEN_MAP_INTERFACE_H
#define CARMEN_MAP_INTERFACE_H

#include <carmen/map.h>
#include <carmen/map_messages.h>

#ifdef __cplusplus
extern "C" {
#endif


/* request hmap (hierarchical map) from server */
int carmen_map_get_hmap(carmen_hmap_p hmap);

/* subscribe to hmap messages output by the map server
void carmen_map_subscribe_hmap_update_message(carmen_hmap_p hmap, 
                                              carmen_handler_t handler, 
					      carmen_subscribe_t subscribe_how);
*/

/* change map zone within an hmap */
int carmen_map_change_map_zone(char *zone_name);

/* subscribe to map zone messages */
void carmen_map_subscribe_map_zone_message(char **zone_name,
					   carmen_handler_t handler,
					   carmen_subscribe_t subscribe_how);

/* request a map from the server */
//int carmen_map_get_gridmap(carmen_map_p map);
//int carmen_map_get_gridmap_by_name(char *name, carmen_map_p map);

/* subscribe to map messages output by the map server */
void carmen_map_subscribe_gridmap_update_message(carmen_map_t *map, 
						 carmen_handler_t handler, 
						 carmen_subscribe_t 
						 subscribe_how);

/* request a map from the server */
int carmen_map_get_placelist(carmen_map_placelist_p placelist);
int carmen_map_get_placelist_by_name(char *name, carmen_map_placelist_p placelist);

int carmen_map_get_offlimits(carmen_offlimits_p *offlimits, int *list_length);
int carmen_map_get_offlimits_by_name(char *name, carmen_offlimits_p *offlimits, int *list_length);

int carmen_map_apply_offlimits_chunk_to_map(carmen_offlimits_p offlimits, 
					    int list_length, carmen_map_p map);

int
carmen_map_get_global_offset(carmen_global_offset_t *global_offset);
int carmen_map_get_global_offset_by_name(char *name, 
					 carmen_global_offset_t *global_offset);

/* conversions between map_point_t's and other points */
/* conversions between map_point_t's and other points */
int carmen_map_to_world(carmen_map_point_p map_point, carmen_world_point_p world_point);
int carmen_world_to_map(carmen_world_point_p world_point, carmen_map_point_p map_point); 

int carmen_point_to_map(carmen_point_p point, carmen_map_point_p map_point, carmen_map_p map);

/* distance between two map_points in grid cells */
double carmen_distance_map(carmen_map_point_p p1, carmen_map_point_p p2);
double carmen_distance_world(carmen_world_point_p p1, carmen_world_point_p p2);

int carmen_map_cmp(carmen_map_point_p p1, carmen_map_point_p p2);
int carmen_world_cmp(carmen_world_point_p p1, carmen_world_point_p p2);

int carmen_map_to_trajectory(carmen_map_point_p carmen_map_point, carmen_traj_point_p traj_point);
int carmen_map_to_ackerman_trajectory(carmen_map_point_p carmen_map_point, carmen_robot_and_trailer_traj_point_t *traj_point);

int carmen_trajectory_to_map(carmen_traj_point_p traj_point, carmen_map_point_p carmen_map_point,
				  carmen_map_p map);
int carmen_ackerman_trajectory_to_map(carmen_robot_and_trailer_traj_point_t *traj_point, carmen_map_point_p carmen_map_point,
				  carmen_map_p map);

void carmen_map_copy(carmen_map_p new_map, carmen_map_p map);
carmen_map_p carmen_map_clone(carmen_map_p map);
void carmen_map_destroy(carmen_map_p *map);

void carmen_map_interface_set_superimposed_map(carmen_map_p map);
carmen_map_p carmen_map_interface_get_superimposed_map();

carmen_map_t *carmen_map_interface_create_new_empty_map(carmen_map_config_t *reference_config);
carmen_map_t *carmen_map_interface_create_new_empty_log_odds_map(carmen_map_config_t *reference_config);

#ifdef __cplusplus
}
#endif

#endif
// @}
