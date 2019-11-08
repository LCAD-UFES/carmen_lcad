/*
 * complete_map_to_block_map.c
 *
 *  Created on: 20/04/2013
 *      Author: romulo
 */



#include <dirent.h>
#include <stdio.h>
#include <carmen/carmen.h>
#include "grid_mapping.h"

static char *map_path = "/home/romulo/carmen/data/map_volta_da_ufes-20121003-01";
static double map_resolution = 0.2;
static double block_size_in_meters = 150;
//
//static void
//build_complete_map(carmen_map_t *block_map, carmen_map_t *complete_map)
//{
//	int local_x, local_y;
//	double val;
//
//	for (int x = 0; x < block_map->config.x_size; x++)
//	{
//		for (int y = 0; y < block_map->config.y_size; y++)
//		{
//			val = block_map->map[x][y];
//
//			if (val == -1.0)
//				continue;
//
//			local_x = ((((double)x * block_map->config.resolution) + block_map->config.x_origin) - complete_map->config.x_origin) / complete_map->config.resolution;
//			local_y = ((((double)y * block_map->config.resolution) + block_map->config.y_origin) - complete_map->config.y_origin) / complete_map->config.resolution;
//
//
//			if (local_x < 0 || local_x > complete_map->config.x_size ||
//					local_y < 0 || local_y > complete_map->config.y_size)
//				continue;
//
//			if (val > 0.5 || complete_map->map[local_x][local_y] == -1)
//			{
//				complete_map->map[local_x][local_y] = val;
//				continue;
//			}
//		}
//	}
//}

static void
read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] = {
			{(char *)"command_line",	 (char *)"map_path",		CARMEN_PARAM_STRING, &map_path,	 		1, NULL},
			{(char *)"command_line",	 (char *)"map_resolution",	CARMEN_PARAM_DOUBLE, &map_resolution,	1, NULL},
			{(char *)"command_line",	 (char *)"block_size_in_meters",	CARMEN_PARAM_DOUBLE, &block_size_in_meters,	1, NULL},

	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}

int
main(int argc, char **argv)
{
	carmen_map_t block_map, complete_map, count_map;

	carmen_ipc_initialize(argc, argv);
	read_parameters(argc, argv);

	carmen_grid_mapping_init_parameters(map_resolution, block_size_in_meters);

	carmen_grid_mapping_read_complete_map(map_path, &complete_map);

	carmen_grid_mapping_initialize_map(&block_map, block_size_in_meters/map_resolution, map_resolution, 'm');
	carmen_grid_mapping_initialize_map(&count_map, block_size_in_meters/map_resolution, map_resolution, 'm');

	for (double x = 0; x < complete_map.config.x_size * complete_map.config.resolution; x += block_size_in_meters/3.0)
	{
		for (double y = 0; y < complete_map.config.y_size * complete_map.config.resolution; y += block_size_in_meters/3.0)
		{
			count_map.config.x_origin = block_map.config.x_origin = floor(complete_map.config.x_origin + x);
			count_map.config.y_origin = block_map.config.y_origin = floor(complete_map.config.y_origin + y);

			if ((block_map.config.x_origin + block_size_in_meters) >
					floor(complete_map.config.x_origin + ((complete_map.config.x_size >= block_map.config.x_size)? complete_map.config.x_size: block_map.config.x_size) * complete_map.config.resolution) ||
				(block_map.config.y_origin + block_size_in_meters) >
					floor(complete_map.config.y_origin + ((complete_map.config.y_size >= block_map.config.y_size)? complete_map.config.y_size: block_map.config.y_size) * complete_map.config.resolution))
				continue;

			for (int x_index = 0; x_index < block_map.config.x_size; x_index++)
			{
				for (int y_index = 0; y_index < block_map.config.y_size; y_index++)
				{
					if ((x_index < complete_map.config.x_size) && (y_index < complete_map.config.y_size))
						block_map.map[x_index][y_index] = complete_map.map[(int)(x / complete_map.config.resolution) + x_index][(int)(y / complete_map.config.resolution) + y_index];
					else
						block_map.map[x_index][y_index] = -1;
					count_map.map[x_index][y_index] = 1.0;
				}
			}
			carmen_grid_mapping_save_block_map_by_origin(map_path, 'm',&block_map);
			//carmen_grid_mapping_save_block_map_by_origin(map_path, 's',&block_map);
			//carmen_grid_mapping_save_block_map_by_origin(map_path, 'c',&count_map);
		}
	}
	printf("%d %d\n", complete_map.config.x_size, complete_map.config.y_size);

	return 1;
}

