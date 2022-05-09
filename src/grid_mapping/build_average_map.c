/*
 * complete_map_to_bitmap.c
 *
 *  Created on: 30/08/2016
 *      Author: Thomas
 */



#include <dirent.h>
#include <stdio.h>
#include <carmen/carmen.h>
#include "grid_mapping.h"

static char *map_path = ".";
char new_complete_map_name[1024];


static char *map_name = "%s/complete_map.map";
static char *map_name_info = "%s/complete_map.info";

static void
read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] = {
			{(char *)"command_line",	 (char *)"map_path",	CARMEN_PARAM_STRING, &map_path,	 1, NULL},
	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}

int
carmen_grid_mapping_read_complete_map_type(char *map_path, carmen_map_t *map, char *map_type)
{
	FILE *file;
	char global_map_path[1000], aux[1000];
	int rtr;

	if (map_type[0] == 'u'){
		map_name = "%scomplete_map_sum.map";
		map_name_info = "%scomplete_map_sum.info";
	}else if(map_type[0] == 'o'){
		map_name = "%scomplete_map_count.map";
		map_name_info = "%scomplete_map_count.info";
	}else if(map_type[0] == 'e'){
		map_name = "%scomplete_map_mean.map";
		map_name_info = "%scomplete_map_mean.info";
	}

	sprintf(global_map_path, map_name, map_path);
	printf("Reading map %s\n", global_map_path);
	rtr = carmen_map_read_gridmap_chunk(global_map_path, map);

	sprintf(global_map_path, map_name_info, map_path);

	file = fopen(global_map_path, "r");

	if (file == NULL)
		fprintf(stderr, "Error: complete map not found!\n");

	fscanf(file, "%s\n", aux);
	map->config.x_origin = atof(aux);
	fscanf(file, "%s\n", aux);
	map->config.y_origin = atof(aux);
	fclose(file);

	return rtr;
}


int
main(int argc, char **argv)
{
	carmen_map_t complete_map_sum;
	carmen_map_t complete_map_count;
	carmen_map_t complete_map_average;

	carmen_ipc_initialize(argc, argv);
	read_parameters(argc, argv);

	//carmen_grid_mapping_init_parameters(map_resolution, block_size_in_meters);

	carmen_grid_mapping_read_complete_map_type(map_path, &complete_map_sum, "u");
	carmen_grid_mapping_read_complete_map_type(map_path, &complete_map_count, "o");
//	carmen_grid_mapping_read_complete_map_type(map_path, &complete_map_average, "e");

	complete_map_average.config.resolution = complete_map_sum.config.resolution;
	complete_map_average.config.x_origin = complete_map_sum.config.x_origin;
	complete_map_average.config.y_origin = complete_map_sum.config.y_origin;
	complete_map_average.config.x_size = complete_map_sum.config.x_size;
	complete_map_average.config.y_size = complete_map_sum.config.y_size;
	complete_map_average.complete_map = (double*) malloc(sizeof(double) * complete_map_average.config.x_size * complete_map_average.config.y_size);
	complete_map_average.map = (double**)malloc(sizeof(double*) * complete_map_average.config.x_size);

	for (int x = 0; x < complete_map_average.config.x_size; x++)
	{
		complete_map_average.map[x] = &complete_map_average.complete_map[x * complete_map_average.config.y_size];

		//initializing map with unknown
		for (int y = 0; y < complete_map_average.config.y_size; y++)
		{
			complete_map_average.map[x][y] = -1.0;
		}
	}

	double mean = 0.0;
	for (int x = 0; x < complete_map_sum.config.x_size; x++)
	{
		for (int y = 0; y < complete_map_sum.config.y_size; y++)
		{
			if (complete_map_count.map[x][y] > 0.0)
			{
				mean = complete_map_sum.map[x][y] / complete_map_count.map[x][y];

				complete_map_average.map[x][y] = mean;

//				if(map1.map[x][y] < 0.0 && map2.map[x][y] < 0.0 && map3.map[x][y] < 0.0)
//					continue;
//
//				//			mean = carmen_prob_models_log_odds_to_probabilistic(get_log_odds(map1.map[x][y]) +
//				//					get_log_odds(map2.map[x][y]) + get_log_odds(map3.map[x][y]));
//				int count = (map1.map[x][y] < 0.0 ? 0 : 1) +
//						(map2.map[x][y] < 0.0 ? 0 : 1) +
//						(map3.map[x][y] < 0.0 ? 0 : 1);
//
//				mean = ((map1.map[x][y] < 0.0 ? 0.0 : map1.map[x][y]) +
//						(map2.map[x][y] < 0.0 ? 0.0 : map2.map[x][y]) +
//						(map3.map[x][y] < 0.0 ? 0.0 : map3.map[x][y])) / count;

//				if (mean < 0.5)
//					complete_map_average.map[x][y] = 0.0;
//				else if (mean > 0.6)
//					complete_map_average.map[x][y] = 1.0;
//				else
//					complete_map_average.map[x][y] = mean;
//				complete_map_average.map[x][y] = mean > 0.5 ? mean : 0.0;
//				complete_map_average.map[x][y] = ((mean < 0.68) && (mean > 0.0)) ? 0.0  : mean;
			}
		}
	}

	sprintf(new_complete_map_name, "%scomplete_map.map", map_path);
	printf("Saving map %s\n", new_complete_map_name);
	carmen_grid_mapping_save_map(new_complete_map_name, &complete_map_average);

	return (1);
}
