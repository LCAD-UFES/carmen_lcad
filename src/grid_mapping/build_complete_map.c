/*
 * build_complete_map.c
 *
 *  Created on: 03/02/2013
 *      Author: romulo
 */

#include <dirent.h>
#include <stdio.h>
#include <carmen/carmen.h>
#include "grid_mapping.h"

static char *map_path = "/home/romulo/carmen/data/map_volta_da_ufes-20121003-01";
static double map_resolution = 0.2;
char *map_type = "mmmmmmm";
static char *map_name = "%s/complete_map.map";
static char *map_name_info = "%s/complete_map.info";


static void
get_map_origin_by_filename(char *filename, double *x_origin, double *y_origin)
{
	char map_name[1000], *div_char, *aux;

	*x_origin = *y_origin = 0.0;

	if (filename == NULL)
		return;

	div_char = strrchr(filename, '/');

	if (div_char == NULL)
	{
		return;
	}

	div_char++;

	strcpy(map_name, div_char);

	div_char = strrchr(map_name, '_');

	if (div_char != NULL && (map_name[0] == 'm' || map_name[0] == 'g' || map_name[0] == 's' || map_name[0] == '2' || map_name[0] == 'c'
			|| map_name[0] == 'u' || map_name[0] == 'o' || map_name[0] == 'e' || (map_name[0] >= 'h' && map_name[0] <= 'l')))
	{
		aux = strrchr(map_name, '.');

		*aux = '\0';
		*div_char = '\0';
		div_char++;

		if((isdigit(*(map_name + 1)) || *(map_name + 1) == '-') && (isdigit(*div_char) || *div_char == '-'))
		{
			*x_origin = atof(map_name + 1);
			*y_origin = atof(div_char);
		}
	}
}

static void
build_complete_map(carmen_map_t *block_map, carmen_map_t *complete_map)
{
	int local_x, local_y;
	double val;

	for (int x = 0; x < block_map->config.x_size; x++)
	{
		for (int y = 0; y < block_map->config.y_size; y++)
		{
			val = block_map->map[x][y];

			if (val == -1.0)
				continue;

			local_x = x + (int)((double)(block_map->config.x_origin - complete_map->config.x_origin) / complete_map->config.resolution);//((((double)x * block_map->config.resolution) + block_map->config.x_origin) - complete_map->config.x_origin) / complete_map->config.resolution;
			local_y = y + (int)((double)(block_map->config.y_origin - complete_map->config.y_origin) / complete_map->config.resolution);//((((double)y * block_map->config.resolution) + block_map->config.y_origin) - complete_map->config.y_origin) / complete_map->config.resolution;


			if (local_x < 0 || local_x >= complete_map->config.x_size ||
					local_y < 0 || local_y >= complete_map->config.y_size)
				continue;

		//	if (val > 0.5 || complete_map->map[local_x][local_y] == -1.0)
			//{
				complete_map->map[local_x][local_y] = val;
				//continue;
			//}
		}
	}
}

static void
read_parameters(int argc, char **argv)
{
	carmen_param_allow_unfound_variables(1);

	carmen_param_t param_list[] = 
	{
			{(char *)"commandline",	 (char *)"map_path",		CARMEN_PARAM_STRING, &map_path,	 	1, NULL},
			{(char *)"commandline",	 (char *)"map_resolution",	CARMEN_PARAM_DOUBLE, &map_resolution,	1, NULL},
			{(char *)"commandline",	 (char *)"map_type",		CARMEN_PARAM_STRING, &map_type,		1, NULL},

	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}

int
main(int argc, char **argv)
{
	carmen_point_t min_pose, max_pose;
	FILE *file;

	char full_path[1000] = "", global_map_path[1000];
	DIR *dp;
	struct dirent *dirp;
	int i;
	carmen_map_t block_map, complete_map;
	double x_origin, y_origin, size = 0.0;
	char complete_map_name[1024];
	
//	map_type = (char *) malloc(1024);
//	strcpy(map_type, "mmmmmmm");

	carmen_ipc_initialize(argc, argv);
	read_parameters(argc, argv);

	min_pose.x = DBL_MAX;
	min_pose.y = DBL_MAX;

	max_pose.x = -DBL_MAX;
	max_pose.y = -DBL_MAX;

	dp  = opendir(map_path);
	sprintf(complete_map_name, "rm %s/complete_map.*", map_path);
//	system(complete_map_name);

	for (dirp = readdir(dp), i = 0; dirp != NULL; dirp = readdir(dp))
	{
		if (dirp->d_name[0] != map_type[0])
			continue;

		strcat(full_path, map_path);
		strcat(full_path, "/");//mudar para / no linux
		strcat(full_path, dirp->d_name);

		get_map_origin_by_filename(full_path, &x_origin, &y_origin);


		if (i == 0)
		{
			carmen_map_read_gridmap_chunk(full_path, &block_map);
			size = floor((double)block_map.config.x_size * block_map.config.resolution);

			free(block_map.complete_map);
			free(block_map.map);
		}

		if (x_origin < min_pose.x)
			min_pose.x = x_origin;

		if ((x_origin + size) > max_pose.x)
			max_pose.x = x_origin + size;

		if (y_origin < min_pose.y)
			min_pose.y = y_origin;

		if ((y_origin + size) > max_pose.y)
			max_pose.y = y_origin + size;

		full_path[0] = '\0';

		i++;
	}

	closedir(dp);

	complete_map.config.resolution = map_resolution;
	complete_map.config.x_origin = min_pose.x;
	complete_map.config.y_origin = min_pose.y;
	complete_map.config.x_size = (max_pose.x - min_pose.x) / complete_map.config.resolution;
	complete_map.config.y_size = (max_pose.y - min_pose.y) / complete_map.config.resolution;
	complete_map.complete_map = (double *) malloc(sizeof(double) * complete_map.config.x_size * complete_map.config.y_size);
	carmen_test_alloc(complete_map.complete_map);
	complete_map.map = (double **)malloc(sizeof(double*) * complete_map.config.x_size);
	carmen_test_alloc(complete_map.map);


	for (int x = 0; x < complete_map.config.x_size; x++)
	{
		complete_map.map[x] = &complete_map.complete_map[x * complete_map.config.y_size];

		//initializing map with unknown
		for (int y = 0; y < complete_map.config.y_size; y++)
		{
			complete_map.map[x][y] = -1.0;
		}
	}

	dp  = opendir(map_path);

	for(dirp = readdir(dp), i = 0; dirp != NULL; dirp = readdir(dp))
	{
		if (dirp->d_name[0] != map_type[0])
			continue;

		strcat(full_path, map_path);
		strcat(full_path, "/");
		strcat(full_path, dirp->d_name);

		carmen_map_read_gridmap_chunk(full_path, &block_map);
//		printf("block map read = %s \n",full_path);
		get_map_origin_by_filename(full_path, &block_map.config.x_origin, &block_map.config.y_origin);
		build_complete_map(&block_map, &complete_map);
		free(block_map.complete_map);
		free(block_map.map);

		full_path[0] = '\0';
	}

	closedir(dp);

	if (map_type[0] == 'u'){
		map_name = "%s/complete_map_sum.map";
		map_name_info = "%s/complete_map_sum.info";
	}else if(map_type[0] == 'o'){
		map_name = "%s/complete_map_count.map";
		map_name_info = "%s/complete_map_count.info";
	}else if(map_type[0] == 'e'){
		map_name = "%s/complete_map_mean.map";
		map_name_info = "%s/complete_map_mean.info";
	}else if(map_type[0] == 's'){
		map_name = "%s/complete_map_remission.map";
		map_name_info = "%s/complete_map_remission.info";
	}else if(map_type[0] == 'm'){
		map_name = "%s/complete_map.map";
		map_name_info = "%s/complete_map.info";
	}

	sprintf(global_map_path, map_name, map_path);
	carmen_grid_mapping_save_map(global_map_path, &complete_map);

	sprintf(global_map_path, map_name_info, map_path);
	file = fopen(global_map_path, "w");

	fprintf(file, "%f\n%f\n", complete_map.config.x_origin, complete_map.config.y_origin);

	fclose(file);

	return 1;
}
