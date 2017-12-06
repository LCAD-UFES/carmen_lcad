#include <iostream>
#include <math.h>
#include "road_mapper_utils.h"

static carmen_map_t *g_current_road_map;

std::string g_window_name1 = "map probabilities";
#define MAX_PROB (pow(2.0, 16) - 1.0)

int g_ipc_required = 0;
int g_img_channels = 3;
int g_class_bits = 0;
int g_road_map_index = 1;

static void
read_parameters(int argc, char **argv)
//read_parameters(int argc __attribute__ ((unused)), char **argv __attribute__ ((unused)))
{
	const char usage[] = "[-c <img_channels>] [-b <class_bits>] <road_map_1>.map [<road_map_2>.map ...]";
	for(int i = 1; i < argc; i++)
	{
		if(strncmp(argv[i], "-h", 2) == 0 || strncmp(argv[i], "--help", 6) == 0)
		{
			printf("Usage:\n%s %s\n", argv[0], usage);
			exit(1);
		}
		else if(strncmp(argv[i], "-c", 2) == 0 || strncmp(argv[i], "--img_channels", 14) == 0)
		{
			g_road_map_index++;
			if ((i + 1) < argc)
			{
				g_img_channels = atoi(argv[i + 1]);
				i++;
				g_road_map_index++;
			}
			else
			{
				printf("Image channels expected following -c option.\n");
				printf("Usage:\n%s %s\n", argv[0], usage);
			}
		}
		else if(strncmp(argv[i], "-b", 2) == 0 || strncmp(argv[i], "--class_bits", 12) == 0)
		{
			g_road_map_index++;
			if ((i + 1) < argc)
			{
				g_class_bits = atoi(argv[i + 1]);
				i++;
				g_road_map_index++;
			}
			else
			{
				printf("Class bits expected following -b option.\n");
				printf("Usage:\n%s %s\n", argv[0], usage);
			}
		}
		else
		{
			break;
		}
	}
	if (g_road_map_index == argc)
	{
		printf("At least one road map file expected\n");
		printf("Usage:\n%s %s\n", argv[0], usage);
		exit(1);
	}
	printf("Image channels set to %d.\n", g_img_channels);
	if (g_img_channels == 1)
	{
		printf("Class bits set to %d.\n", g_class_bits);
	}
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
		if (g_ipc_required)
		{
			carmen_ipc_disconnect();
		}
		std::cout << "road_mapper_display_map3: disconnected.\n";
		free_map_pointer(g_current_road_map);
		exit(0);
	}
}

void
road_mapper_display_map3_display(carmen_map_t *current_road_map, int img_channels, int img_class_bits)
{
	cv::namedWindow(g_window_name1, cv::WINDOW_AUTOSIZE);
	cv::moveWindow(g_window_name1, 78 + current_road_map->config.x_size, 10);

	cv::Mat image1;
	if (img_channels == 1)
	{
		image1 = cv::Mat(current_road_map->config.y_size,
						current_road_map->config.x_size,
						CV_8UC1);
		road_map_to_image_black_and_white(current_road_map, &image1, img_class_bits);
	}
	else
	{
		image1 = cv::Mat(current_road_map->config.y_size,
						current_road_map->config.x_size,
						CV_8UC3, cv::Scalar::all(0));
		road_map_to_image(current_road_map, &image1);
	}
	cv::imshow(g_window_name1, image1);
	std::cout << "Press \"Esc\" key to continue...\n\n";
	while((cv::waitKey() & 0xff) != 27);
}


int
main(int argc, char **argv)
{
	read_parameters(argc, argv);
	if (g_ipc_required)
	{
		carmen_ipc_initialize(argc, argv);
		carmen_param_check_version(argv[0]);
		define_messages();
	}
	signal(SIGINT, shutdown_module);

	g_current_road_map = alloc_map_pointer();
	for (int i = g_road_map_index; i < argc; i++)
	{
		char *map_file_name = argv[i];
		bool valid_map_on_file = (carmen_map_read_gridmap_chunk(map_file_name, g_current_road_map) == 0);
		if (valid_map_on_file)
		{
			std::cout << "File " << std::string(map_file_name) << " being displayed..." << std:: endl;
			road_mapper_display_map3_display(g_current_road_map, g_img_channels, g_class_bits);
		}
		else
		{
			std::cout << "road_mapper_display_map3: could not read offline map from file named: " << map_file_name << std::endl;
		}
	}
	if (g_ipc_required)
	{
		register_handlers();
		carmen_ipc_dispatch();
	}
	return 0;
}
