/*
 * grid_mapping.c
 *
 *  Created on: 15/08/2012
 *      Author: romulo
 */
#include "grid_mapping.h"
#include <carmen/road_mapper.h>
#include <string.h>


#define LOCAL_MAP_SIZE (150)
#define GLOBAL_MAP_SIZE (1800)

static double global_gridmap_resolution = 0.6;
static double local_gridmap_resolution = 0.2;
static double local_gridmap_size = LOCAL_MAP_SIZE;
static int local_gridmap_count = LOCAL_MAP_SIZE / 0.2;
static int local_gridmap_count_3 = (LOCAL_MAP_SIZE/3) / 0.2;
static int global_gridmap_count = GLOBAL_MAP_SIZE / 0.6;
static int map_quadrant;

static int file_warnings = 1;

#define MAP_BUFFER_SIZE 18

static carmen_map_t *map_buffer[MAP_BUFFER_SIZE + 1];
static int map_buffer_index = 0;


void
carmen_grid_mapping_set_unknown_value(carmen_map_t *map, char map_type)
{
	int x, y;
	road_prob cell = { .off_road = MAX_PROB, .solid_marking = 0, .broken_marking = 0, .lane_center = 0 };
	double unknown = -1;

	if (map_type == 'r')
		unknown = *((double*) &cell);

	for (x = 0; x < map->config.x_size; x++)
	{
		map->map[x] = &(map->complete_map[x * map->config.y_size]);

		//initializing map with unknown
		for (y = 0; y < map->config.y_size; y++)
		{
			map->map[x][y] = unknown;
		}
	}
}


void
carmen_grid_mapping_create_new_map(carmen_map_t *map,
		int gridmap_size_x, int gridmap_size_y, double gridmap_resolution, char map_type)
{
	map->config.resolution = gridmap_resolution;
	map->config.x_size = gridmap_size_x;
	map->config.y_size = gridmap_size_y;
	map->config.map_name = NULL;

	map->complete_map = (double*) malloc(sizeof(double) * gridmap_size_x * gridmap_size_y);
	carmen_test_alloc(map->complete_map);
	map->map = (double**) malloc(sizeof(double*) * gridmap_size_x);
	carmen_test_alloc(map->map);
	//map->config.map_name = (char *)calloc(strlen(map_filename)+1, sizeof(char));
	//carmen_test_alloc(map->config.map_name);
	//strcpy(map->config.map_name, map_filename);

	carmen_grid_mapping_set_unknown_value(map, map_type);
}


void
carmen_grid_mapping_initialize_map(carmen_map_t *map,
		int gridmap_size, double gridmap_resolution, char map_type)
{
	carmen_grid_mapping_create_new_map(map, gridmap_size, gridmap_size, gridmap_resolution, map_type);
}


void
carmen_grid_mapping_init_parameters(double resolution, double size)
{
	int i;
	
	local_gridmap_size = size;
	local_gridmap_resolution = resolution;
	local_gridmap_count = size / resolution;
	local_gridmap_count_3 = (size / 3.0) / resolution;

	for(i = 0; i < MAP_BUFFER_SIZE; i++)
	{
		map_buffer[i] = (carmen_map_t*) calloc(sizeof(carmen_map_t), 1);
		map_buffer[i]->complete_map = NULL;
		map_buffer[i]->map = NULL;
		map_buffer[i]->config.map_name = NULL;
	}
}


void
carmen_grid_mapping_init_global_map_parameters(double resolution, double size)
{
	global_gridmap_resolution = resolution;
	global_gridmap_count = size / resolution;
}


int
carmen_grid_mapping_get_quadrant(carmen_point_t map_pose)
{
	if (map_pose.y < (local_gridmap_count_3) )
	{
		if (map_pose.x >= 0 && map_pose.x < (local_gridmap_count_3))
		{
			return 0;
		}
		else if (map_pose.x < ((local_gridmap_count_3) * 2))
		{
			return 1;
		}
		else if (map_pose.x < local_gridmap_count)
		{
			return 2;
		}
	}
	else if (map_pose.y < ((local_gridmap_count_3) * 2))
	{
		if (map_pose.x >= 0 && map_pose.x < (local_gridmap_count_3))
		{
			return 3;
		}
		else if (map_pose.x < ((local_gridmap_count_3) * 2))
		{
			return 4;
		}
		else if (map_pose.x < local_gridmap_count)
		{
			return 5;
		}
	}
	else if (map_pose.y <= local_gridmap_count)
	{
		if (map_pose.x >= 0 && map_pose.x < (local_gridmap_count_3))
		{
			return 6;
		}
		else if (map_pose.x < ((local_gridmap_count_3) * 2))
		{
			return 7;
		}
		else if (map_pose.x < local_gridmap_count)
		{
			return 8;
		}
	}
	return -1;
}


static void
copy_chunk_map(double *map, int desloc_map, double *old_map, int desloc_old_map, int n, int size)
{
	int i;
	
	for (i = 0; i < n; i++)
	{
		memcpy((void*)(map + (i * local_gridmap_count + desloc_map)), (void*)(old_map + (i * local_gridmap_count + desloc_old_map)), sizeof(double) * size);
	}
}


void
carmen_grid_mapping_get_map_origin(carmen_point_t *global_pose, double *x_origin, double *y_origin)
{
	*x_origin = floor((floor(global_pose->x / (local_gridmap_size / 3.0) ) - 1.0) * (local_gridmap_size / 3.0));
	*y_origin = floor((floor(global_pose->y / (local_gridmap_size / 3.0) ) - 1.0) * (local_gridmap_size / 3.0));
}

//################### Matrix Organization ######################
/*
 Interface
 | 6 | 7 | 8 |
 | 3 | 4 | 5 |
 | 0 | 1 | 2 |

 Memory
 | 0 | 3 | 6 |
 | 1 | 4 | 7 |
 | 2 | 5 | 8 |
 */
//##############################################################



static void
integrate_maps(carmen_map_t *new_map, carmen_map_t *map, int quadrant)
{


	switch (quadrant)
	{

	case 0://ok
		copy_chunk_map(new_map->complete_map + (local_gridmap_count_3 * local_gridmap_count), local_gridmap_count_3, map->complete_map, 0, local_gridmap_count_3 * 2, local_gridmap_count_3 * 2);
		break;
	case 1://ok
		copy_chunk_map(new_map->complete_map, local_gridmap_count_3, map->complete_map, 0, local_gridmap_count, local_gridmap_count_3 * 2);
		break;
	case 2://ok*
		copy_chunk_map(new_map->complete_map, local_gridmap_count_3, map->complete_map + (local_gridmap_count_3 * local_gridmap_count), 0, local_gridmap_count_3 * 2, local_gridmap_count_3 * 2);
		break;

	case 3://ok
		memcpy((void*)(new_map->complete_map + (local_gridmap_count * local_gridmap_count_3)), (void*)map->complete_map, sizeof(double) * local_gridmap_count * local_gridmap_count_3 * 2);
		break;

	case 5://ok
		memcpy((void*)new_map->complete_map, (void*)(map->complete_map + (local_gridmap_count * local_gridmap_count_3)), sizeof(double) * local_gridmap_count * local_gridmap_count_3 * 2);
		break;

	case 6:
		copy_chunk_map(new_map->complete_map + (local_gridmap_count_3 * local_gridmap_count), 0, map->complete_map, local_gridmap_count_3, local_gridmap_count_3 * 2, local_gridmap_count_3 * 2);
		break;

	case 7://ok
		copy_chunk_map(new_map->complete_map, 0, map->complete_map, local_gridmap_count_3, local_gridmap_count, local_gridmap_count_3 * 2);
		break;

	case 8:
		copy_chunk_map(new_map->complete_map, 0, map->complete_map + (local_gridmap_count_3 * local_gridmap_count), local_gridmap_count_3, local_gridmap_count_3 * 2, local_gridmap_count_3 * 2);
		break;

	default://ok
		break;
	}
}


static double*
get_cell_pointer(double *map, int cell_number)
{
	switch (cell_number)
	{
	case 0:
		return map;
		break;
	case 1:
		return &map[local_gridmap_count * local_gridmap_count_3];
		break;
	case 2:
		return &map[local_gridmap_count * 2 * local_gridmap_count_3];
		break;
	case 3:
		return &map[local_gridmap_count_3];
		break;
	case 4:
		return &map[local_gridmap_count * local_gridmap_count_3 + local_gridmap_count_3];
		break;
	case 5:
		return &map[local_gridmap_count * 2 * local_gridmap_count_3 + local_gridmap_count_3];
		break;
	case 6:
		return &map[2 * local_gridmap_count_3];
		break;
	case 7:
		return &map[2 * local_gridmap_count_3 + local_gridmap_count * local_gridmap_count_3];
		break;
	case 8:
		return &map[2 * local_gridmap_count_3 + local_gridmap_count * local_gridmap_count_3 * 2];
		break;
	}
	return NULL;
}

//################### Matrix Organization ######################
/*
 Interface
 | 6 | 7 | 8 |
 | 3 | 4 | 5 |
 | 0 | 1 | 2 |

 Memory
 | 0 | 3 | 6 |
 | 1 | 4 | 7 |
 | 2 | 5 | 8 |
 */
//##############################################################


static void
copy_cell_to_map(carmen_map_t *map, double **cell, int cell_number)
{
	int i;

	double *map_cell;
	map_cell = get_cell_pointer(map->complete_map, cell_number);

	for (i = 0; i < local_gridmap_count_3; i++)
	{
		memcpy(map_cell + i * local_gridmap_count, cell[i], local_gridmap_count_3 * sizeof(double));
	}
}


static void
copy_map_to_cell(double **cell, carmen_map_t *map, int cell_number)
{
	int i;

	double *map_cell;
	map_cell = get_cell_pointer(map->complete_map, cell_number);

	for (i = 0; i < local_gridmap_count_3; i++)
	{
		memcpy(cell[i], map_cell + i * local_gridmap_count, local_gridmap_count_3 * sizeof(double));
	}
}


//static void
//merge_map(carmen_map_t *map, double *map_file)
//{
//	int i;
//	for (i = 0; i < (map->config.x_size * map->config.y_size); i++)
//	{
//		if (map->complete_map[i] < 0)
//			map->complete_map[i] = map_file[i];
//	}
//}


int
carmen_grid_mapping_is_map_changed(carmen_position_t *map_origin, double x_origin, double y_origin)
{
	if (map_origin->x != x_origin || map_origin->y != y_origin)
		return 1;

	return 0;
}


void
carmen_grid_mapping_switch_maps(carmen_map_t *current_map, carmen_map_t *new_map)
{
	free(current_map->complete_map);
	free(current_map->map);

	current_map->complete_map = new_map->complete_map;
	current_map->map = new_map->map;
	current_map->config = new_map->config;

	new_map->complete_map = NULL;
	new_map->map = NULL;
}


void
carmen_grid_mapping_swap_maps_and_clear_old_map(carmen_map_t *current_map, carmen_map_t *new_map)
{
	double *aux, **aux2;

	aux = current_map->complete_map;
	current_map->complete_map = new_map->complete_map;
	new_map->complete_map = aux;

	aux2 = current_map->map;
	current_map->map = new_map->map;
	new_map->map = aux2;

	for (int i = 0; i < new_map->config.x_size * new_map->config.x_size; i++)
		new_map->complete_map[i] = -1;
}


//int
//carmen_grid_mapping_get_map_by_origin(char *map_path, carmen_point_t pose, carmen_map_t *map, carmen_map_t *new_map)
//{
//	char full_map_path[100];
//	char file_map_name[100];
//	double x_origin, y_origin;
//	int map_exists_on_file;
//	double *map_file;
//	carmen_point_t map_pose;
//
//	map_file = NULL;
//	carmen_grid_mapping_get_map_origin(&pose, &x_origin, &y_origin);
//
//
//	sprintf(file_map_name, "m%d_%d.map", (int) x_origin, (int) y_origin);
//	sprintf(full_map_path, "%s/m%d_%d.map", map_path, (int) x_origin, (int) y_origin);
//
//	map_exists_on_file = carmen_map_read_gridmap_chunk(full_map_path, new_map) != -1;
//
//	if (!map_exists_on_file)
//	{
//		//map not found in the folder
//		//allocate and create a new map
//		carmen_grid_mapping_initialize_map(new_map, local_gridmap_count, local_gridmap_resolution);
//	}
//	else
//	{
//		map_file = (double*) malloc(sizeof(double) * new_map->config.x_size * new_map->config.y_size);
//		memcpy(map_file, new_map->complete_map, sizeof(double) * new_map->config.x_size * new_map->config.y_size);
//	}
//
//
//	new_map->config.x_origin = x_origin;
//	new_map->config.y_origin = y_origin;
//
//	if (map != NULL)
//	{
//		map_pose.x = (pose.x - map->config.x_origin) / map->config.resolution;
//		map_pose.y = (pose.y - map->config.y_origin) / map->config.resolution;
//
//		map_quadrant = carmen_grid_mapping_get_quadrant(map_pose);
//		integrate_maps(new_map, map, map_quadrant);
//	}
//
//
//	if (map_exists_on_file)
//	{
//		merge_map(new_map, map_file);
//		free(map_file);
//	}
//
//	return map_exists_on_file;
//}


void
carmen_grid_mapping_change_blocks_map_by_origin(carmen_point_t pose, carmen_map_t *map, carmen_map_t *new_map)
{
	double x_origin, y_origin;
	carmen_point_t map_pose;

	carmen_grid_mapping_get_map_origin(&pose, &x_origin, &y_origin);

	new_map->config.x_origin = x_origin;
	new_map->config.y_origin = y_origin;

	if (map != NULL)
	{
		map_pose.x = (pose.x - map->config.x_origin) / map->config.resolution;
		map_pose.y = (pose.y - map->config.y_origin) / map->config.resolution;

		map_quadrant = carmen_grid_mapping_get_quadrant(map_pose);
		integrate_maps(new_map, map, map_quadrant);
	}
}


static int
find_buffered_map(double x_origin, double y_origin)
{
	int i;

	for (i = 0; i < MAP_BUFFER_SIZE; i++)
	{
		if (map_buffer[i])
		{
			if (map_buffer[i]->config.x_origin == x_origin && map_buffer[i]->config.y_origin == y_origin && map_buffer[i]->complete_map != NULL)
				return i;
		}
	}

	return -1;
}


int
carmen_grid_mapping_get_buffered_map(double x_origin, double y_origin, carmen_map_t *new_map, char map_type)
{
	double local_x_origin, local_y_origin;
	int map_exists_on_buffer = 1;
	int i, j, k;
	int map_index;

	if (new_map->complete_map == NULL)
		carmen_grid_mapping_initialize_map(new_map, local_gridmap_count, local_gridmap_resolution, map_type);

	for (i = 0, k = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++, k++)
		{
			map_exists_on_buffer = 1;
			local_x_origin = x_origin + j * (local_gridmap_size / 3.0);
			local_y_origin = y_origin + i * (local_gridmap_size / 3.0);

			map_index = find_buffered_map(local_x_origin, local_y_origin);

			if (map_index < 0)
			{
				map_exists_on_buffer = 0;
			}

			if (map_exists_on_buffer)
				copy_cell_to_map(new_map, map_buffer[map_index]->map, k);
		}
	}

	new_map->config.x_origin = x_origin;
	new_map->config.y_origin = y_origin;
	strcpy(new_map->config.origin, "from grid_mapping (buffered map)");

	return map_exists_on_buffer;
}


int
carmen_grid_mapping_get_block_map_by_origin_x_y(char *map_path, char map_type, double x_origin, double y_origin, carmen_map_t *new_map)
{
	carmen_map_t unk_map;

	char full_map_path[1024];
	double local_x_origin, local_y_origin;
	int block_map_exists_on_file = 1;
	int count_maps_on_file = 0;
	int i, j, k;

	if (new_map->complete_map == NULL)
		carmen_grid_mapping_initialize_map(new_map, local_gridmap_count, local_gridmap_resolution, map_type);
	else
		carmen_grid_mapping_set_unknown_value(new_map, map_type);

	for (i = 0, k = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++, k++)
		{
			local_x_origin = x_origin + j * (local_gridmap_size / 3.0);
			local_y_origin = y_origin + i * (local_gridmap_size / 3.0);

			sprintf(full_map_path, "%s/%c%d_%d.map", map_path, map_type, (int)local_x_origin, (int)local_y_origin);

			block_map_exists_on_file = carmen_map_read_gridmap_chunk_verbose(full_map_path, &unk_map, file_warnings) != -1;

			if (block_map_exists_on_file)
			{
				copy_cell_to_map(new_map, unk_map.map, k);
				free(unk_map.map);
				free(unk_map.complete_map);
				free(unk_map.config.map_name);
				unk_map.config.map_name = NULL;
				count_maps_on_file++;
			}
		}
	}

	strcpy(new_map->config.origin, "from grid_mapping (built from blocks in files)");
	new_map->config.x_origin = x_origin;
	new_map->config.y_origin = y_origin;

	return count_maps_on_file > 0;

}


int
carmen_grid_mapping_get_block_map_by_origin_x_y_verbose(char *map_path, char map_type, double x_origin, double y_origin, carmen_map_t *new_map, int verbose)
{
	int result;
	int previous_file_warnings_option = file_warnings;

	file_warnings = verbose;
	result = carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, map_type, x_origin, y_origin, new_map);
	file_warnings = previous_file_warnings_option;

	return result;
}


int
carmen_grid_mapping_get_block_map_by_origin(char *map_path, char map_type, carmen_point_t pose, carmen_map_t *new_map)
{
	double x_origin, y_origin;

	carmen_grid_mapping_get_map_origin(&pose, &x_origin, &y_origin);
	return carmen_grid_mapping_get_block_map_by_origin_x_y(map_path, map_type, x_origin, y_origin, new_map);
}


int
carmen_grid_mapping_update_map_buffer(carmen_map_t *map, char map_type)
{
	int i, j, k, x_origin, y_origin;
	int map_index;

	for (i = 0, k = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++, k++)
		{
			x_origin = map->config.x_origin + j * local_gridmap_size / 3.0;
			y_origin = map->config.y_origin + i * local_gridmap_size / 3.0;
			map_index = find_buffered_map(x_origin, y_origin);

			if(map_index < 0)
			{
				map_index = map_buffer_index;
				map_buffer_index = (map_buffer_index + 1) % MAP_BUFFER_SIZE;
			}

			if (map_buffer[map_index]->complete_map == NULL)
				carmen_grid_mapping_initialize_map(map_buffer[map_index], local_gridmap_count_3, local_gridmap_resolution, map_type);

			copy_map_to_cell(map_buffer[map_index]->map, map, k);
			map_buffer[map_index]->config.x_origin = x_origin;
			map_buffer[map_index]->config.y_origin = y_origin;
		}
	}

	return  1;
}


int
carmen_grid_mapping_save_block_map_by_origin(char *map_path, char map_type, carmen_map_t *map)
{
	char full_map_path[100];
	int i, j, k, x_origin, y_origin;
	carmen_FILE *fp;

	static carmen_map_t *unk_map = NULL;

	if (unk_map == NULL)
	{
		unk_map = (carmen_map_t *)calloc(1, sizeof(carmen_map_t));
		carmen_grid_mapping_initialize_map(unk_map, local_gridmap_count / 3.0, local_gridmap_resolution, map_type);
	}

	for (i = 0, k = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++, k++)
		{
			x_origin = map->config.x_origin + j * local_gridmap_size / 3.0;
			y_origin = map->config.y_origin + i * local_gridmap_size / 3.0;

			copy_map_to_cell(unk_map->map, map, k);

			sprintf(full_map_path, "%s/%c%d_%d.map", map_path, map_type,
					x_origin, y_origin);

			fp = carmen_fopen(full_map_path, "w+");

			if (fp == NULL)
			{
				fprintf(stderr, "Could not write map to file: %s\n", full_map_path);
				return 0;
			}

			if (carmen_map_write_all(fp, unk_map->map,
					map->config.x_size / 3,
					map->config.y_size / 3,
					map->config.resolution,
					"",	"", "", "Generated by big_map",
					NULL, 0, NULL, 0, NULL, 0) == -1)
			{
				return 0;
			}

			carmen_fclose(fp);
		}
	}

	return  1;
}


int
carmen_grid_mapping_save_map(char *map_file_name, carmen_map_t *map)
{
	carmen_FILE *fp;

	fp = carmen_fopen(map_file_name, "w+");

	if (fp == NULL)
	{
		fprintf(stderr, "Could not create map named %s in carmen_grid_mapping_save_map()!\n", map_file_name);
		return 0;
	}

	if (carmen_map_write_all(fp, map->map,
			map->config.x_size,
			map->config.y_size,
			map->config.resolution,
			"",	"", "", "Generated by LCAD-UFES",
			NULL, 0, NULL, 0, NULL, 0) == -1)
	{
		return 0;
	}

	carmen_fclose(fp);

	return  1;
}


int
carmen_grid_mapping_save_map_by_origin(char *map_path, carmen_map_t *map)
{
	char full_map_path[1024];

	sprintf(full_map_path, "%s/m%d_%d.map", map_path, (int) map->config.x_origin, (int) map->config.y_origin);

	return (carmen_grid_mapping_save_map(full_map_path, map));
}


int
carmen_grid_mapping_read_complete_map(char *map_path, carmen_map_t *map)
{
	FILE *file;
	char global_map_path[1000], aux[1000];
	int rtr;

	sprintf(global_map_path, "%s/complete_map.map", map_path);
	rtr = carmen_map_read_gridmap_chunk(global_map_path, map);

	sprintf(global_map_path, "%s/complete_map.info", map_path);

	file = fopen(global_map_path, "r");

	if (file == NULL)
	{
		fprintf(stderr, "Error: complete map not found!\n");
		return -1;
	}

	fscanf(file, "%s\n", aux);
	map->config.x_origin = atof(aux);
	fscanf(file, "%s\n", aux);
	map->config.y_origin = atof(aux);
	fclose(file);

	return rtr;
}


carmen_map_t *
carmen_grid_mapping_copy_map(carmen_map_t *map, carmen_map_t *map_from)
{
	int i;

	if (!map)
	{
		map = (carmen_map_t *) malloc(sizeof(carmen_map_t));
		map->complete_map = malloc(map_from->config.x_size * map_from->config.y_size * sizeof(double *));
		map->map = (double **) malloc(map_from->config.x_size * sizeof(double *));
		for (i = 0; i < map_from->config.x_size; i++)
			map->map[i] = map->complete_map + i * map_from->config.y_size;
	}

	map->config = map_from->config;
	memcpy(map->complete_map, map_from->complete_map, map_from->config.x_size *  map_from->config.y_size * sizeof(double));

	return (map);
}

