/*
 * publish.cpp
 *
 *  Created on: Set 19, 2017
 *      Author: Lucas Veronese
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <carmen/mapper_interface.h>
#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv/cv.h>

#include "shared_memory_test_interface.h"

// Buffer data structures
#define MAX_BUFFER 10

#define SHARED_MEM_COMPLETE_MAP "/posix-shared-mem-complete_map_shared"

carmen_map_config_t map_config;
carmen_map_t *map_buffer = NULL;

double *complete_map_shared = NULL;


void
fill_line(carmen_map_t *map, int line)
{
	for (int i = 0; i < map->config.y_size; i++)
		map->map[line][i] = 1.0;
}

void
clean_map(carmen_map_t *map)
{
	for (int i = 0; i < map->config.x_size; i++)
		for (int j = 0; j < map->config.y_size; j++)
			map->map[i][j] = -1.0;
}

void error (char *msg)
{
	perror (msg);
	exit (1);
}

void
init_shared_buffer_server()
{
	printf("%d\n", map_config.x_size * map_config.y_size);
	static int fd_shm_complete_map;
	if ((fd_shm_complete_map = shm_open (SHARED_MEM_COMPLETE_MAP, O_RDWR| O_CREAT, 0660)) == -1)
		error ((char*)"shm_open");

	if (ftruncate (fd_shm_complete_map, MAX_BUFFER * map_config.x_size * map_config.y_size * sizeof(double)) == -1)
		error ((char*)"ftruncate");

	if ((complete_map_shared = (double *)mmap (NULL, MAX_BUFFER * map_config.x_size * map_config.y_size * sizeof(double), PROT_READ | PROT_WRITE, MAP_SHARED, fd_shm_complete_map, 0)) == MAP_FAILED)
		error ((char*)"mmap");
}

void create_shared_buffer_server(carmen_map_t *vmap, int number_of_map, int gridmap_size_x, int gridmap_size_y, double gridmap_resolution)
{
	for (int i = 0; i < number_of_map; i++)
	{
		vmap[i].config.resolution = gridmap_resolution;
		vmap[i].config.x_size = gridmap_size_x;
		vmap[i].config.y_size = gridmap_size_y;
		vmap[i].config.map_name = NULL;
		vmap[i].map = (double**) malloc(sizeof(double*) * gridmap_size_x);
		carmen_test_alloc(vmap[i].map);
		vmap[i].complete_map = &(complete_map_shared[i * gridmap_size_x * gridmap_size_y]);

		for (int x = 0; x < vmap[i].config.x_size; x++)
		{
			vmap[i].map[x] = &(vmap[i].complete_map[x *vmap[i].config.y_size]);

			//initializing map with unknown
			for (int y = 0; y < vmap[i].config.y_size; y++)
			{
				vmap[i].map[x][y] = -1.0;
			}
		}
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_shared_memory_map_message(carmen_map_config_t config, int index, int number_of_map)
{
	carmen_shared_memory_test_map_message msg;
	msg.shared_memory_name = (char *)SHARED_MEM_COMPLETE_MAP;
	msg.config = config;
	msg.buffer_index = index;
	msg.buffer_size = number_of_map;

	msg.host = carmen_get_host();
	msg.timestamp = carmen_get_time();

	carmen_shared_memory_test_publish_map_message(&msg);
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("publish: disconnected.\n");

		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
read_parameters(int argc, char **argv)
{
	int num_items;
	double map_resolution, map_width, map_height;

	carmen_param_t param_list[] =
	{
			{(char *) "mapper",  (char *) "map_grid_res", CARMEN_PARAM_DOUBLE, &map_resolution, 0, NULL},
			{(char *) "mapper",  (char *) "map_width", CARMEN_PARAM_DOUBLE, &map_width, 0, NULL},
			{(char *) "mapper",  (char *) "map_height", CARMEN_PARAM_DOUBLE, &map_height, 0, NULL},
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	map_config.x_size = round(map_width / map_resolution);
	map_config.y_size = round(map_height / map_resolution);
	map_config.resolution = map_resolution;

	map_buffer = (carmen_map_t *)calloc(MAX_BUFFER, sizeof(carmen_map_t));
	init_shared_buffer_server();
	create_shared_buffer_server(map_buffer,MAX_BUFFER,map_config.x_size, map_config.y_size, map_config.resolution);

}


static void
define_messages(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_SHARED_MEMORY_TEST_MAP_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_SHARED_MEMORY_TEST_MAP_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_SHARED_MEMORY_TEST_MAP_MESSAGE_NAME);
}


///////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);

	define_messages();

	signal(SIGINT, shutdown_module);

	int map_index = 0;
	int line_to_fill = 0;

	while(1)
	{
		if (line_to_fill == map_config.y_size -1)
		{
			//clean_map(&map_buffer[map_index]);
			line_to_fill = 0;
		}

		//fill_line(&map_buffer[map_index], line_to_fill++);
		//memcpy(complete_map_shared, map_buffer[map_index].complete_map, map_buffer[map_index].config.x_size * map_buffer[map_index].config.y_size * sizeof(double));
		publish_shared_memory_map_message(map_buffer[map_index].config, map_index, MAX_BUFFER);

		int aux_index = (map_index + 1) % MAX_BUFFER;
		//memcpy(map_buffer[aux_index].complete_map, map_buffer[map_index].complete_map, map_buffer[map_index].config.x_size * map_buffer[map_index].config.y_size * sizeof(double));
		map_index = aux_index;

	}

	return(0);
}
