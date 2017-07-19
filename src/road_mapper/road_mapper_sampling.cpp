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

static carmen_point_t g_global_pos, g_previous_global_pos;
static carmen_map_t *g_remission_map;
static carmen_map_t *g_road_map;
static int g_sample_width = 0;
static int g_sample_height = 0;
static double g_distance_samples = 0.0;
static int g_n_offsets = 0;
static int g_n_rotations = 0;
static double g_distance_offset = 0.0;

cv::Mat *g_road_map_img;
cv::Mat *g_remission_map_img;

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
	for (i = 0; i < g_remission_map->config.x_size; i++)
	{
		for (j = 0; j < g_remission_map->config.y_size; j++)
		{
			uchar aux = (uchar) 3.5 * (255.0 * (1.0 - (g_remission_map->map[i][j] < 0 ? 1 : g_remission_map->map[i][j])) + 0.5);
			g_remission_map_img->at<uchar>(i, j) = 255 - aux;
		}
	}
	//cv::resize(*g_remission_map_img, *g_remission_map_img, cv::Size(350, 350));
	*g_remission_map_img = rotate(*g_remission_map_img, 90);
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
	for (x = 0; x < g_road_map->config.x_size; x++)
	{
		for (y = 0; y < g_road_map->config.y_size; y++)
		{
			cell_prob = road_mapper_double_to_prob(&g_road_map->map[x][y]);
			road_mapper_cell_color(cell_prob, &blue, &green, &red);
			color[0] = blue;
			color[1] = green;
			color[2] = red;
			g_road_map_img->at<cv::Vec3b>(x, y) = color;
		}
	}
	//cv::resize(*g_road_map_img, *g_road_map_img, cv::Size(350, 350));
	*g_road_map_img = rotate(*g_road_map_img, 90);
}

static void
generate_new_samples(void)
{
	if (g_road_map->config.x_origin != g_remission_map->config.x_origin ||
			g_road_map->config.y_origin != g_remission_map->config.y_origin)
		return;

	int x = round((g_global_pos.x - g_road_map->config.x_origin) / g_road_map->config.resolution);
	int y = round((g_global_pos.y - g_road_map->config.y_origin) / g_road_map->config.resolution);
	if (x >= g_road_map->config.x_size || x < 0 || y >= g_road_map->config.y_size || y < 0)
		return;

	remission_map_to_image();
	road_map_to_image();
	g_remission_map_img->at<uchar>(cv::Point(x, g_road_map->config.y_size -1 -y)) = 255;
	g_road_map_img->at<cv::Vec3b>(cv::Point(x, g_road_map->config.y_size -1 -y)) = cv::Vec3b(0, 0, 0);
	cv::imshow("remission_map", *g_remission_map_img);
	cv::imshow("road_map", *g_road_map_img);
	cv::waitKey(33);
}

static void
read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] =
	{
			{(char*)"road_mapper",  (char*)"sample_width", 		CARMEN_PARAM_INT, 		&(g_sample_width), 		0, NULL},
			{(char*)"road_mapper",  (char*)"sample_height",		CARMEN_PARAM_INT, 		&(g_sample_height), 	0, NULL},
			{(char*)"road_mapper",  (char*)"distance_sample",	CARMEN_PARAM_DOUBLE, 	&(g_distance_samples), 	0, NULL},
			{(char*)"road_mapper",  (char*)"n_offset",			CARMEN_PARAM_INT, 		&(g_n_offsets), 		0, NULL},
			{(char*)"road_mapper",  (char*)"n_rotation",		CARMEN_PARAM_INT, 		&(g_n_rotations), 		0, NULL},
			{(char*)"road_mapper",  (char*)"distance_offset",	CARMEN_PARAM_DOUBLE, 	&(g_distance_offset),	0, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
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
		memcpy(&g_previous_global_pos, &msg->globalpos, sizeof(carmen_point_t));
		first_time = 0;
	}
	memcpy(&g_global_pos, &msg->globalpos, sizeof(carmen_point_t));
	double distance = carmen_distance(&g_previous_global_pos, &g_global_pos);
	if (distance >= g_distance_samples)
	{
		//printf("%.2lf\n", distance);
		memcpy(&g_previous_global_pos, &msg->globalpos, sizeof(carmen_point_t));
		generate_new_samples();
	}
}

static void
localize_map_handler(carmen_map_server_localize_map_message *msg)
{
	static int first_time = 1;
	if (first_time == 1)
	{
		carmen_grid_mapping_initialize_map(g_remission_map,
											msg->config.x_size,
											msg->config.resolution, 'm');
		g_remission_map_img = new cv::Mat(g_remission_map->config.y_size,
											g_remission_map->config.x_size,
											CV_8UC1);
		first_time = 0;
	}
	memcpy(g_remission_map->complete_map, msg->complete_mean_remission_map, sizeof(double) * msg->size);
	memcpy(&g_remission_map->config, &msg->config, sizeof(carmen_map_config_t));
}

static void
road_map_handler(carmen_map_server_road_map_message *msg)
{
	static int first_time = 1;
	if (first_time == 1)
	{
		carmen_grid_mapping_initialize_map(g_road_map,
											msg->config.x_size,
											msg->config.resolution, 'r');
		g_road_map_img = new cv::Mat(g_road_map->config.y_size,
											g_road_map->config.x_size,
											CV_8UC3,
											cv::Scalar::all(0));
		first_time = 0;
	}
	memcpy(g_road_map->complete_map, msg->complete_map, sizeof(double) * msg->size);
	memcpy(&g_road_map->config, &msg->config, sizeof(carmen_map_config_t));
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
	g_road_map = (carmen_map_p) calloc (1, sizeof(carmen_map_t));
	g_road_map->config.x_origin = g_road_map->config.y_origin = 0.0001;
	g_road_map->complete_map = NULL;
	g_road_map->map = NULL;

	g_remission_map = (carmen_map_p) calloc (1, sizeof(carmen_map_t));
	g_remission_map->config.x_origin = g_remission_map->config.y_origin = 0.0001;
	g_remission_map->complete_map = NULL;
	g_remission_map->map = NULL;
}

void
deinitialize_maps(void)
{
	if (g_road_map->complete_map) free(g_road_map->complete_map);
	if (g_road_map->map) free(g_road_map->map);
	free(g_road_map);

	if (g_remission_map->complete_map) free(g_remission_map->complete_map);
	if (g_remission_map->map) free(g_remission_map->map);
	free(g_remission_map);

	g_remission_map_img->release();
	g_road_map_img->release();
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
