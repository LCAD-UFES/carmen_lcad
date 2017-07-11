#include <iostream>
#include <stdio.h>
#include <sys/io.h>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <sys/types.h>
#include <math.h>

#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION == 3
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#else
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
#include <carmen/road_mapper.h>

static carmen_map_t *current_road_map;

std::string window_name1 = "map probabilities";
#define MAX_PROB (pow(2.0, 16) - 1.0)

static void
read_parameters(int argc __attribute__ ((unused)), char **argv __attribute__ ((unused)))
{
}

static void
define_messages()
{
}

static void
register_handlers()
{
}

static void
road_mapper_display_map3_initialize_map(void)
{
	current_road_map = (carmen_map_p) calloc (1, sizeof(carmen_map_t));
	current_road_map->config.x_origin = current_road_map->config.y_origin = 0.0001;
	current_road_map->complete_map = NULL;
	current_road_map->map = NULL;
}

void
road_mapper_display_map3_deinitialize_map(void)
{
	if (current_road_map->complete_map) free(current_road_map->complete_map);
	free(current_road_map);
}

void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		std::cout << "road_mapper_display_map3: disconnected.\n";
		road_mapper_display_map3_deinitialize_map();
		exit(0);
	}
}

void
road_mapper_display_map3_display(void)
{
	int x = 0, y = 0;
	road_prob *cell_prob;
	cv::Vec3b color;
	uchar blue;
	uchar green;
	uchar red;

	cv::namedWindow(window_name1, 1);
	cv::moveWindow(window_name1, 78 + current_road_map->config.x_size, 10);

	cv::Mat image1(current_road_map->config.y_size, current_road_map->config.x_size,
					CV_8UC3, cv::Scalar::all(0));

	for (x = 0; x < current_road_map->config.x_size; x++)
	{
		for (y = 0; y < current_road_map->config.y_size; y++)
		{
			cell_prob = road_mapper_double_to_prob(&current_road_map->map[x][y]);
			road_mapper_cell_color(cell_prob, &blue, &green, &red);
			color[0] = blue;
			color[1] = green;
			color[2] = red;
			image1.at<cv::Vec3b>(current_road_map->config.y_size - 1 - y, x) = color;
		}
	}
	cv::imshow(window_name1, image1);
	std::cout << "\nPress \"Esc\" key to continue...\n";
	while(cv::waitKey() != 27);
	image1.release();
	cv::destroyWindow(window_name1);
}

int
main(int argc, char **argv)
{
	int no_valid_map_on_file;
	static char *map_file_name;

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);
	define_messages();

	signal(SIGINT, shutdown_module);

	road_mapper_display_map3_initialize_map();

	if (argc != 2)
	{
		std::cerr << argv[0] << " <road_map>.map" << std::endl;
		return -1;
	}
	else
	{
		map_file_name = argv[1];
		no_valid_map_on_file = carmen_map_read_gridmap_chunk(map_file_name, current_road_map) != 0;
		if (no_valid_map_on_file)
		{
			std::cout << "map_server: could not read offline map from file named: " << map_file_name << std::endl;
			return -1;
		}
		road_mapper_display_map3_display();
	}

	register_handlers();
	carmen_ipc_dispatch();

	return 0;
}
