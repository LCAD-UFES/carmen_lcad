#include <iostream>
#include <math.h>
#include "road_mapper_utils.h"

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

void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		std::cout << "road_mapper_display_map3: disconnected.\n";
		free_map_pointer(current_road_map);
		exit(0);
	}
}

void
road_mapper_display_map3_display(void)
{
	cv::namedWindow(window_name1, 1);
	cv::moveWindow(window_name1, 78 + current_road_map->config.x_size, 10);

	cv::Mat image1(current_road_map->config.y_size, current_road_map->config.x_size,
					CV_8UC3, cv::Scalar::all(0));

	road_map_to_image(current_road_map, &image1);

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

	current_road_map = alloc_map_pointer();

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
			std::cout << "road_mapper_display_map3: could not read offline map from file named: " << map_file_name << std::endl;
			return -1;
		}
		road_mapper_display_map3_display();
	}

	register_handlers();
	carmen_ipc_dispatch();

	return 0;
}
