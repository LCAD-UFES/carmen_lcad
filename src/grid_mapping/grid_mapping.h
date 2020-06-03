/*
 * grid_mapping.h
 *
 *  Created on: 15/08/2012
 *      Author: romulo
 */

#ifndef CARMEN_GRID_MAPPING_H
#define CARMEN_GRID_MAPPING_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <carmen/carmen.h>


/**
 * If there is a map for the given pose, read the map in the file system and put in map
 * Otherwise initialize map with -1 (unknown)
 * Integrate the old_map with the new map
 *
 * @map_path path for the map folder
 * @pose desired global pose
 * @old_map
 * @map  the returned map
 *
 */

void carmen_grid_mapping_change_blocks_map_by_origin(carmen_point_t pose, carmen_map_t *map, carmen_map_t *new_map);

//int carmen_grid_mapping_get_map_by_origin(char *map_path, carmen_point_t pose, carmen_map_t *map, carmen_map_t *new_map);

int carmen_grid_mapping_is_map_changed(carmen_position_t *map_origin, double x_origin, double y_origin);

void carmen_grid_mapping_switch_maps(carmen_map_t *current_map, carmen_map_t *new_map);

void carmen_grid_mapping_swap_maps_and_clear_old_map(carmen_map_t *current_map, carmen_map_t *new_map);

int carmen_grid_mapping_update_map_buffer(carmen_map_t *map, char map_type);

void carmen_grid_mapping_set_unknown_value(carmen_map_t *map, char map_type);

void carmen_grid_mapping_create_new_map(carmen_map_t *map, int gridmap_size_x, int gridmap_size_y, double gridmap_resolution, char map_type);

void carmen_grid_mapping_initialize_map(carmen_map_t *map, int gridmap_size, double gridmap_resolution, char map_type);

int carmen_grid_mapping_get_buffered_map(double x_origin, double y_origin, carmen_map_t *new_map, char map_type);

int carmen_grid_mapping_get_block_map_by_origin(char *map_path, char map_type, carmen_point_t pose, carmen_map_t *new_map);

int carmen_grid_mapping_get_block_map_by_origin_x_y(char *map_path, char map_type, double x_origin, double y_origin, carmen_map_t *new_map);
int carmen_grid_mapping_get_block_map_by_origin_x_y_verbose(char *map_path, char map_type, double x_origin, double y_origin, carmen_map_t *new_map, int verbose);

int carmen_grid_mapping_save_block_map_by_origin(char *map_path, char map_type, carmen_map_t *map);

void carmen_grid_mapping_init_parameters(double resolution, double size);

int carmen_grid_mapping_get_quadrant(carmen_point_t map_pose);

/**
 * Get the map origin in meters
 */
void carmen_grid_mapping_get_map_origin(carmen_point_t *global_pose, double *x_origin, double *y_origin);


/**
 * Save the map in the given map folder path
 *
 * @return if the map was successfully saved return 1, otherwise return 0
 */
int carmen_grid_mapping_save_map_by_origin(char *map_path, carmen_map_t *map);

int carmen_grid_mapping_save_map(char *map_file_name, carmen_map_t *map);

int carmen_grid_mapping_read_complete_map(char *map_path, carmen_map_t *map);

carmen_map_t *carmen_grid_mapping_copy_map(carmen_map_t *map, carmen_map_t *map_from);

#ifdef __cplusplus
}
#endif

#endif
