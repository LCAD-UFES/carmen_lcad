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

static carmen_point_t global_pos, previous_global_pos;
static carmen_map_t *current_remission_map;
static carmen_map_t *current_road_map;

cv::Mat *road_map_image;
cv::Mat *remission_map_image;

// distance in meters between samples
#define distance_samples 2

cv::Mat
rotate(cv::Mat src, double angle)
{
    cv::Mat dst;
    cv::Point2f pt(src.cols/2., src.rows/2.);
    cv::Mat r = getRotationMatrix2D(pt, angle, 1.0);
    cv::warpAffine(src, dst, r, cv::Size(src.cols, src.rows));
    return dst;
}

void
remission_map_to_image(void)
{
	int i = 0, j = 0;
	for (i = 0; i < current_remission_map->config.x_size; i++)
	{
		for (j = 0; j < current_remission_map->config.y_size; j++)
		{
			uchar aux = (uchar) 3.5 * (255.0 * (1.0 - (current_remission_map->map[i][j] < 0 ? 1 : current_remission_map->map[i][j])) + 0.5);
			remission_map_image->at<uchar>(i, j) = 255 - aux;
		}
	}
	//cv::resize(*remission_map_image, *remission_map_image, cv::Size(350, 350));
	*remission_map_image = rotate(*remission_map_image, 90);
}

void
road_map_to_image(void)
{
	int x = 0, y = 0;
	road_prob *cell_prob;
	cv::Vec3b color;
	uchar blue;
	uchar green;
	uchar red;
	for (x = 0; x < current_road_map->config.x_size; x++)
	{
		for (y = 0; y < current_road_map->config.y_size; y++)
		{
			cell_prob = road_mapper_double_to_prob(&current_road_map->map[x][y]);
			road_mapper_cell_color(cell_prob, &blue, &green, &red);
			color[0] = blue;
			color[1] = green;
			color[2] = red;
			road_map_image->at<cv::Vec3b>(x, y) = color;
		}
	}
	//cv::resize(*road_map_image, *road_map_image, cv::Size(350, 350));
	*road_map_image = rotate(*road_map_image, 90);
}

static void
generate_new_samples(void)
{
	int x = round((global_pos.x - current_road_map->config.x_origin) / current_road_map->config.resolution);
	int y = round((global_pos.y - current_road_map->config.y_origin) / current_road_map->config.resolution);
	if (x >= current_road_map->config.x_size || x < 0 || y >= current_road_map->config.y_size || y < 0)
		return;
	remission_map_to_image();
	road_map_to_image();
	remission_map_image->at<uchar>(cv::Point(x, current_road_map->config.y_size -1 -y)) = 255;
	road_map_image->at<cv::Vec3b>(cv::Point(x, current_road_map->config.y_size -1 -y)) = cv::Vec3b(0, 0, 0);
	cv::imshow("remission_map", *remission_map_image);
	cv::imshow("road_map", *road_map_image);
	cv::waitKey(33);
}

static void
read_parameters(int argc __attribute__ ((unused)), char **argv __attribute__ ((unused)))
{
}

static void
define_messages()
{
	carmen_localize_ackerman_define_globalpos_messages();
	carmen_map_server_define_localize_map_message();
	carmen_map_server_define_road_map_message();
}

static void
global_pos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	static int first_time = 1;
	if (first_time == 1)
	{
		memcpy(&previous_global_pos, &msg->globalpos, sizeof(carmen_point_t));
		first_time = 0;
	}
	memcpy(&global_pos, &msg->globalpos, sizeof(carmen_point_t));
	double distance = carmen_distance(&previous_global_pos, &global_pos);
	if (distance >= distance_samples)
	{
		//printf("%.2lf\n", distance);
		memcpy(&previous_global_pos, &msg->globalpos, sizeof(carmen_point_t));
		generate_new_samples();
	}
}

static void
localize_map_handler(carmen_map_server_localize_map_message *msg)
{
	static int first_time = 1;
	if (first_time == 1)
	{
		carmen_grid_mapping_initialize_map(current_remission_map,
											msg->config.x_size,
											msg->config.resolution, 'm');
		remission_map_image = new cv::Mat(current_remission_map->config.y_size,
											current_remission_map->config.x_size,
											CV_8UC1);
		first_time = 0;
	}
	memcpy(current_remission_map->complete_map, msg->complete_mean_remission_map, sizeof(double) * msg->size);
	memcpy(&current_remission_map->config, &msg->config, sizeof(carmen_map_config_t));
}

static void
road_map_handler(carmen_map_server_road_map_message *msg)
{
	static int first_time = 1;
	if (first_time == 1)
	{
		carmen_grid_mapping_initialize_map(current_road_map,
											msg->config.x_size,
											msg->config.resolution, 'r');
		road_map_image = new cv::Mat(current_road_map->config.y_size,
											current_road_map->config.x_size,
											CV_8UC3,
											cv::Scalar::all(0));
		first_time = 0;
	}
	memcpy(current_road_map->complete_map, msg->complete_map, sizeof(double) * msg->size);
	memcpy(&current_road_map->config, &msg->config, sizeof(carmen_map_config_t));
}

static void
register_handlers()
{
	carmen_localize_ackerman_subscribe_globalpos_message(NULL,
															(carmen_handler_t) global_pos_handler,
															CARMEN_SUBSCRIBE_LATEST);

	carmen_map_server_subscribe_localize_map_message(NULL,
														(carmen_handler_t) localize_map_handler,
														CARMEN_SUBSCRIBE_LATEST);

	carmen_map_server_subscribe_road_map(NULL,
												(carmen_handler_t) road_map_handler,
												CARMEN_SUBSCRIBE_LATEST);
}

static void
initialize_maps(void)
{
	current_road_map = (carmen_map_p) calloc (1, sizeof(carmen_map_t));
	current_road_map->config.x_origin = current_road_map->config.y_origin = 0.0001;
	current_road_map->complete_map = NULL;
	current_road_map->map = NULL;

	current_remission_map = (carmen_map_p) calloc (1, sizeof(carmen_map_t));
	current_remission_map->config.x_origin = current_remission_map->config.y_origin = 0.0001;
	current_remission_map->complete_map = NULL;
	current_remission_map->map = NULL;
}

void
deinitialize_maps(void)
{
	if (current_road_map->complete_map) free(current_road_map->complete_map);
	if (current_road_map->map) free(current_road_map->map);
	free(current_road_map);

	if (current_remission_map->complete_map) free(current_remission_map->complete_map);
	if (current_remission_map->map) free(current_remission_map->map);
	free(current_remission_map);

	remission_map_image->release();
	road_map_image->release();
}

void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		std::cout << "road_mapper_generate_db: disconnected.\n";
		deinitialize_maps();
		exit(0);
	}
}

int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);
	define_messages();

	signal(SIGINT, shutdown_module);

	initialize_maps();

	register_handlers();
	carmen_ipc_dispatch();

	return 0;
}
