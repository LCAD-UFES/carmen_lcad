#include <iostream>
#include <math.h>
#include <string.h>
#include <libgen.h>
#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
#include <iostream>
#include <string>
#include <string.h>
#include "rddf_graph.h"

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

using namespace std;

bool g_ipc_required = false;
int g_already_visited_index = 1;

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
already_visited_to_image(carmen_map_p map, cv::Mat *road_map_img)
{
	cv::Vec3b color;

	for (int x = 0; x < map->config.x_size; x++)
	{
		for (int y = 0; y < map->config.y_size; y++)
		{
			if (map->map[x][y] == 1)
			{
				color[0] = 0;
				color[1] = 0;
				color[2] = 0;
			}
			else
			{
				color[0] = 255;
				color[1] = 255;
				color[2] = 255;
			}
			//road_map_img->at<cv::Vec3b>(x, y) = color;
			road_map_img->at<cv::Vec3b>(map->config.y_size - 1 - y, x) = color;
		}
	}
//	cv::Point pt(road_map_img->cols/2.0, road_map_img->rows/2.0);
//	*road_map_img = rotate(*road_map_img, pt, 90);
}

void
rddf_graph_display_already_visited(carmen_map_p already_visited)
{
	cv::Mat image1;
	cv::namedWindow("already_visited", cv::WINDOW_AUTOSIZE);

	image1 = cv::Mat(already_visited->config.y_size, already_visited->config.x_size, CV_8UC3, cv::Scalar::all(0));
	already_visited_to_image(already_visited, &image1);
	cv::imshow("already_visited", image1);

	cout << "Press \"Esc\" key to continue...\n\n";
	while((cv::waitKey() & 0xff) != 27);
}

int
main(int argc, char **argv)
{
	if (g_ipc_required)
	{
		carmen_ipc_initialize(argc, argv);
		carmen_param_check_version(argv[0]);
		define_messages();
	}
	signal(SIGINT, shutdown_module);

	carmen_map_t already_visited;
	for (int i = g_already_visited_index; i < argc; i++)
	{
		char *already_visited_filename = argv[i];
		bool valid_map_on_file = (carmen_map_read_gridmap_chunk(already_visited_filename, &already_visited) == 0);
		if (valid_map_on_file)
		{
			cout << "File " << string(already_visited_filename) << " being displayed... ("
					<< (i - g_already_visited_index + 1) << " of " << (argc - g_already_visited_index) << ")" << endl;
			rddf_graph_display_already_visited(&already_visited);
		}

	}

	if (g_ipc_required)
	{
		register_handlers();
		carmen_ipc_dispatch();
	}
	return 0;
}
