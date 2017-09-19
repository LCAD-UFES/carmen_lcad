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

#include <carmen/shared_memory_test_interface.h>


int fd_shm_complete_map;
double *complete_map_shared = NULL;


void
error (char *msg)
{
	perror (msg);
	exit (1);
}


cv::Mat
create_map_image(carmen_map_t *map)
{
	cv::Mat map_OUT = cv::Mat::zeros(map->config.x_size, map->config.y_size, CV_8UC3);

	for (int i = 0; i < map->config.x_size; i++)
	{
		for (int j = 0; j < map->config.y_size; j++)
		{
			if (map->map[i][j] < 0.0)
			{
				map_OUT.at<cv::Vec3b>(i,j)[0] = 255;
				map_OUT.at<cv::Vec3b>(i,j)[1] = 0;
				map_OUT.at<cv::Vec3b>(i,j)[2] = 0;
				continue;
			}

			uchar value = ((255.0  * (map->map[i][j]))) > 255 ? 255	: ((255.0  * (map->map[i][j])));
			map_OUT.at<cv::Vec3b>(i,j)[0] =	value;
			map_OUT.at<cv::Vec3b>(i,j)[1] =	value;
			map_OUT.at<cv::Vec3b>(i,j)[2] =	value;
		}
	}

	return map_OUT;
}


void
init_shared_buffer_client(char *shared_memory_name, size_t shared_memory_size)
{
	if ((fd_shm_complete_map = shm_open(shared_memory_name, O_RDONLY, 0)) == -1)
		error((char *) "Error in shm_open() within init_shared_buffer_client()");

	if ((complete_map_shared = (double *) mmap(NULL, shared_memory_size, PROT_READ, MAP_SHARED, fd_shm_complete_map, 0)) == MAP_FAILED)
		error((char *) "Error in mmap() within init_shared_buffer_client()");
}


void
create_shared_buffer(carmen_map_t *map_vector, int number_of_map, int gridmap_size_x, int gridmap_size_y, double gridmap_resolution)
{
	for (int i = 0; i < number_of_map; i++)
	{
		map_vector[i].config.resolution = gridmap_resolution;
		map_vector[i].config.x_size = gridmap_size_x;
		map_vector[i].config.y_size = gridmap_size_y;
		map_vector[i].config.map_name = NULL;
		map_vector[i].map = (double **) malloc(sizeof(double *) * gridmap_size_x);
		carmen_test_alloc(map_vector[i].map);
		map_vector[i].complete_map = &(complete_map_shared[i * gridmap_size_x * gridmap_size_y]);

		for (int x = 0; x < map_vector[i].config.x_size; x++)
			map_vector[i].map[x] = &(map_vector[i].complete_map[x * map_vector[i].config.y_size]);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
mapper_handler(carmen_shared_memory_test_map_message *message)
{
	static carmen_map_t *map_buffer = NULL;

	if (map_buffer == NULL)
	{
		map_buffer = (carmen_map_t *) calloc(message->number_of_maps, sizeof(carmen_map_t));
		init_shared_buffer_client(message->shared_memory_name, message->number_of_maps * message->config.x_size * message->config.y_size * sizeof(double));
		create_shared_buffer(map_buffer, message->number_of_maps, message->config.x_size, message->config.y_size, message->config.resolution);
	}

//	static int count = 0;
//	printf("%d\n", count++);

	cv::Mat map_img = create_map_image(&map_buffer[message->map_index]);
	cv::imshow("map", map_img);
	cv::waitKey(1);
}


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
subscribe_to_relevant_messages()
{
	carmen_shared_memory_test_subscribe_map_message(NULL, (carmen_handler_t) mapper_handler, CARMEN_SUBSCRIBE_LATEST);
}
///////////////////////////////////////////////////////////////////////////////////////////////


int
main (int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_module);

	subscribe_to_relevant_messages();

	carmen_ipc_dispatch();

	exit(0);
}
