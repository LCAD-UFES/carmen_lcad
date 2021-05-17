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

#ifndef CARMEN_MAP_H
#define CARMEN_MAP_H

#ifdef __cplusplus
extern "C" {
#endif

#define          CARMEN_NAMED_POSITION_TYPE          0
#define          CARMEN_NAMED_POSE_TYPE              1
#define          CARMEN_LOCALIZATION_INIT_TYPE       2

#define          CARMEN_OFFLIMITS_POINT_ID           0
#define          CARMEN_OFFLIMITS_LINE_ID            1
#define          CARMEN_OFFLIMITS_RECT_ID            2

#define          CARMEN_HMAP_LINK_DOOR               1
#define          CARMEN_HMAP_LINK_ELEVATOR           2


typedef struct {
  int x_size;
  int y_size;
  double resolution;
  char origin[64];
  char *map_name;
  double x_origin;
  double y_origin;
} carmen_map_config_t, *carmen_map_config_p;

typedef struct {
  carmen_map_config_t config;
  double* complete_map;
  double** map;
} carmen_map_t, *carmen_map_p;

typedef struct {
  int type, size;
  char name[22];
  double x, y, theta;
  double x_std, y_std, theta_std;
} carmen_place_t, *carmen_place_p;

typedef struct {
  double x, y, theta;
} carmen_global_offset_t, *carmen_global_offset_p;

typedef struct {
  carmen_place_p places;
  int num_places;
} carmen_map_placelist_t, *carmen_map_placelist_p;

typedef struct {
  carmen_map_config_t config;
  double *complete_data;
  double ***dist;
} carmen_exp_dist_t, *carmen_exp_dist_p;

typedef struct {
  int x, y;
  carmen_map_p map;
} carmen_map_point_t, *carmen_map_point_p;

typedef struct {
  carmen_point_t pose;
  carmen_map_p map;
} carmen_world_point_t, *carmen_world_point_p;

typedef struct {
  carmen_robot_and_trailer_pose_t pose;
  carmen_map_p map;
} carmen_world_robot_and_trailer_pose_t;


typedef struct {
  int type;
  int x1, y1, x2, y2;
} carmen_offlimits_t, *carmen_offlimits_p;

typedef struct {
  carmen_offlimits_p offlimits;
  int list_length;
} carmen_offlimits_list_t, *carmen_offlimits_list_p;

typedef struct {
  double x, y, theta;
  int num_readings;
  double *range;
} carmen_laser_scan_t, *carmen_laser_scan_p;

/* Heirarchical Map Interface Node (either an elevator or a door connecting two map zones) */
typedef struct {
  int type;                // door or elevator
  int degree;              /* num map zones connected by this link.
			      for doors, this should always equal 2 */
  int *keys;               /* indices into hmap.zone_names[]
			      for elevators, zones are stacked from lowest to highest */
  int num_points;          // 2*degree for doors, 1*degree for elevators
  carmen_point_p points;   // indexed by points[key * (num_points / degree) + point]
} carmen_hmap_link_t, *carmen_hmap_link_p;

typedef struct {
  int num_zones;
  char **zone_names;
  int num_links;
  carmen_hmap_link_p links;
} carmen_hmap_t, *carmen_hmap_p;

typedef struct _compact_map
{
	int number_of_known_points_on_the_map;
	int *coord_x;
	int *coord_y;
	double *value;
	carmen_map_config_t config;
} carmen_compact_map_t;

typedef struct _imaging_map
{
	unsigned char *complete_map;
	unsigned char **map;
	int *frequency;
	int n_channels;
	carmen_map_config_t config;
} carmen_imaging_map_t, *carmen_imaging_map_p;


#ifdef __cplusplus
}
#endif

#endif
