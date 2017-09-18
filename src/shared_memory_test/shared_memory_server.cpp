/*
 * shared_memory_client.cpp
 *
 *  Created on: Sep 14, 2017
 *      Author: lucas
 */

/* * * logger.c: Write strings in POSIX shared memory to file * (Server process) */

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
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
// Buffer data structures
#define MAX_BUFFERS 10
#define LOGFILE "/tmp/example.log"

#define SHARED_MEM_MAP "/posix-shared-mem-map"
#define SHARED_MEM_COMPLETE_MAP "/posix-shared-mem-complete_map_shared"
#define SHARED_MEM_MAP_CONFIG "/posix-shared-mem-map_config"

void error (char *msg);

cv::Mat
create_map_image(carmen_map_t *map)
{
	cv::Mat map_OUT = cv::Mat::zeros(map->config.x_size, map->config.y_size, CV_8UC3);

	for (int i = 0; i < map->config.x_size; i++) {
		for (int j = 0; j < map->config.y_size; j++) {

			if (map->map[i][j] < 0.0) {
				map_OUT.at<cv::Vec3b>(i,j)[0] = 0;
				map_OUT.at<cv::Vec3b>(i,j)[1] = 0;
				map_OUT.at<cv::Vec3b>(i,j)[2] = 255;
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
mapper_handler(carmen_mapper_map_message *message)
{

}



int main (int argc, char **argv)
{

	carmen_map_t map;
	carmen_map_config_t *map_config;
	int fd_shm_config, fd_shm_map, fd_shm_complete_map;
	double **map_shared = NULL;
	double *complete_map_shared = NULL;

	if ((fd_shm_config = shm_open (SHARED_MEM_MAP_CONFIG, O_RDWR, 0)) == -1)
		error ((char*)"shm_open");

	if ((map_config = (carmen_map_config_t *)mmap (NULL, sizeof (carmen_map_config_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd_shm_config, 0)) == MAP_FAILED)
			error ((char*)"mmap");

	map.config = *map_config;

	cv::waitKey(33);

	if ((fd_shm_map = shm_open (SHARED_MEM_MAP, O_RDWR, 0)) == -1)
		error ((char*)"shm_open");

	if ((fd_shm_complete_map = shm_open (SHARED_MEM_COMPLETE_MAP, O_RDWR, 0)) == -1)
		error ((char*)"shm_open");

	if ((map_shared = (double **)mmap (NULL, map_config->x_size * sizeof(double*), PROT_READ | PROT_WRITE, MAP_SHARED, fd_shm_map, 0)) == MAP_FAILED)
		error ((char*)"mmap");

	if ((complete_map_shared = (double *)mmap (NULL, map_config->x_size * map_config->y_size * sizeof(double), PROT_READ | PROT_WRITE, MAP_SHARED, fd_shm_complete_map, 0)) == MAP_FAILED)
		error ((char*)"mmap");

	printf("%lf %lf\n", map.config.x_origin, map.config.y_origin);
	printf("%d %d\n", map_config->x_size, map_config->y_size);

	carmen_grid_mapping_create_new_map(&map, map_config->x_size, map_config->y_size, map_config->resolution, 'm');

   //memcpy(map.map, map_shared, map_config->x_size * sizeof(double*));




	while (1)
	{ // forever
		memcpy(map.complete_map, complete_map_shared, map_config->x_size * map_config->y_size * sizeof(double));
		//printf("%lf %lf\n", map_config->x_origin, map_config->y_origin);
		cv::Mat map_img = create_map_image(&map);
		cv::imshow("map", map_img);

		cv::waitKey(1);

	}
}

// Print system error and exit
void error (char *msg)
{
	perror (msg);
	exit (1);
}
