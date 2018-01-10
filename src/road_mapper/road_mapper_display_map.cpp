#include <iostream>
#include <math.h>
#include <string.h>
#include <libgen.h>
#include "road_mapper_utils.h"

using namespace std;

string g_window_name1 = "road map";
string g_window_name2 = "remission map";

#define MAX_PROB (pow(2.0, 16) - 1.0)

bool g_ipc_required = false;
int g_img_channels = 3;
int g_class_bits = 0;
char *g_remission_map_dir = NULL;
int g_road_map_index = 1;

static void
read_parameters(int argc, char **argv)
//read_parameters(int argc __attribute__ ((unused)), char **argv __attribute__ ((unused)))
{
	const char usage[] = "[-c <img_channels>] [-b <class_bits>] [-r <remission_map_dir>] <road_map_1>.map [...]";
	for(int i = 1; i < argc; i++)
	{
		if(strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
			exit(printf("Usage:\n%s %s\n", argv[0], usage));
		else if(strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--img_channels") == 0)
		{
			g_road_map_index++;
			if ((i + 1) < argc)
			{
				g_img_channels = atoi(argv[i + 1]);
				i++, g_road_map_index++;
			}
			else
				printf("Image channels expected following -c option.\nUsage:\n%s %s\n", argv[0], usage);
		}
		else if(strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--class_bits") == 0)
		{
			g_road_map_index++;
			if ((i + 1) < argc)
			{
				g_class_bits = atoi(argv[i + 1]);
				i++, g_road_map_index++;
			}
			else
				printf("Class bits expected following -b option.\nUsage:\n%s %s\n", argv[0], usage);
		}
		else if(strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--remission_maps") == 0)
		{
			g_road_map_index++;
			if ((i + 1) < argc)
			{
				g_remission_map_dir = argv[i + 1];
				int n = strlen(g_remission_map_dir) - 1;
				if (n >= 0 && g_remission_map_dir[n] == '/')
					g_remission_map_dir[n] = 0;
				i++, g_road_map_index++;
			}
			else
				printf("Remission map directory expected following -r option.\nUsage:\n%s %s\n", argv[0], usage);
		}
		else
			break;
	}
	if (g_road_map_index == argc)
		exit(printf("At least one road map file expected\nUsage:\n%s %s\n", argv[0], usage));
	printf("Image channels set to %d.\n", g_img_channels);
	if (g_img_channels == 1)
		printf("Class bits set to %d.\n", g_class_bits);
	if (g_remission_map_dir)
		printf("Remission map directory: %s.\n", g_remission_map_dir);
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
			carmen_ipc_disconnect();
		exit(printf("road_mapper_display_map: disconnected.\n"));
	}
}

void
road_mapper_display_road_map(carmen_map_p road_map, int img_channels, int img_class_bits)
{
	cv::namedWindow(g_window_name1, cv::WINDOW_AUTOSIZE);
	cv::moveWindow(g_window_name1, 78 + road_map->config.x_size, 10);

	cv::Mat image1;
	if (img_channels == 1)
	{
		image1 = cv::Mat(road_map->config.y_size, road_map->config.x_size, CV_8UC1);
		road_map_to_image_black_and_white(road_map, &image1, img_class_bits);
	}
	else
	{
		image1 = cv::Mat(road_map->config.y_size, road_map->config.x_size, CV_8UC3, cv::Scalar::all(0));
		road_map_to_image(road_map, &image1);
	}
	cv::imshow(g_window_name1, image1);
	if (g_remission_map_dir == NULL)
	{
		cout << "Press \"Esc\" key to continue...\n\n";
		while((cv::waitKey() & 0xff) != 27);
	}
}


void
road_mapper_display_remission_map(carmen_map_p sum_remission_map, carmen_map_p count_remission_map)
{
	static carmen_map_t mean_remission_map;
	static bool first_time = true;

	if (first_time)
	{
		carmen_grid_mapping_initialize_map(&mean_remission_map,
				sum_remission_map->config.x_size, sum_remission_map->config.resolution, 'm');
		first_time = false;
	}
	mean_remission_map.config = sum_remission_map->config;

	for (int i = 0; i < mean_remission_map.config.x_size * mean_remission_map.config.y_size; i++)
	{
		double mean = -1;
		if (count_remission_map->complete_map[i] > 0)
			mean = sum_remission_map->complete_map[i] / count_remission_map->complete_map[i];
		mean_remission_map.complete_map[i] = mean;
	}

	cv::namedWindow(g_window_name2, cv::WINDOW_AUTOSIZE);
	cv::moveWindow(g_window_name2, 79 + 2 * mean_remission_map.config.x_size, 10);

	cv::Mat image2;

	image2 = cv::Mat(mean_remission_map.config.y_size, mean_remission_map.config.x_size, CV_8UC1);
	remission_map_to_image(&mean_remission_map, &image2, 1);

	cv::imshow(g_window_name2, image2);
	cout << "Press \"Esc\" key to continue...\n\n";
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

	carmen_map_t road_map;
	for (int i = g_road_map_index; i < argc; i++)
	{
		char *road_map_filename = argv[i];
		bool valid_map_on_file = (carmen_map_read_gridmap_chunk(road_map_filename, &road_map) == 0);
		if (valid_map_on_file)
		{
			cout << "File " << string(road_map_filename) << " being displayed... ("
					<< (i - g_road_map_index + 1) << " of " << (argc - g_road_map_index) << ")" << endl;
			road_mapper_display_road_map(&road_map, g_img_channels, g_class_bits);
			if (g_remission_map_dir)
			{
				carmen_map_t sum_remission_map, count_remission_map;
				char remission_map_filename[2000];
				sprintf(remission_map_filename, "%s/s%s", g_remission_map_dir, &basename(road_map_filename)[1]);
				bool valid_map_1 = (carmen_map_read_gridmap_chunk(remission_map_filename, &sum_remission_map) == 0);
				if (!valid_map_1)
					cout << "road_mapper_display_map: could not read offline map from file named: " << remission_map_filename << endl;
				sprintf(remission_map_filename, "%s/c%s", g_remission_map_dir, &basename(road_map_filename)[1]);
				bool valid_map_2 = (carmen_map_read_gridmap_chunk(remission_map_filename, &count_remission_map) == 0);
				if (!valid_map_2)
					cout << "road_mapper_display_map: could not read offline map from file named: " << remission_map_filename << endl;
				if (valid_map_1 && valid_map_2)
					road_mapper_display_remission_map(&sum_remission_map, &count_remission_map);
			}
		}
		else
			cout << "road_mapper_display_map: could not read offline map from file named: " << road_map_filename << endl;
	}
	if (g_ipc_required)
	{
		register_handlers();
		carmen_ipc_dispatch();
	}
	return 0;
}
