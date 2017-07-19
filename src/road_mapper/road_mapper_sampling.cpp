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
static int g_sample_width = 0;
static int g_sample_height = 0;
static double g_distance_samples = 0.0;
static int g_n_offsets = 0;
static int g_n_rotations = 0;
static double g_distance_offset = 0.0;

cv::Mat *g_road_map_img;
cv::Mat *g_remission_map_img;

#define VEC_SIZE 20
static carmen_map_p g_vec_remission_map[VEC_SIZE];
static carmen_map_p g_vec_road_map[VEC_SIZE];

cv::Mat
rotate(cv::Mat src, cv::Point pt, double angle)
{
    cv::Mat dst;
    cv::Mat r = getRotationMatrix2D(pt, angle, 1.0);
    cv::warpAffine(src, dst, r, cv::Size(src.cols, src.rows));
    return dst;
}

void
remission_map_to_image(carmen_map_p map)
{
	int i = 0, j = 0;
	for (i = 0; i < map->config.x_size; i++)
	{
		for (j = 0; j < map->config.y_size; j++)
		{
			uchar aux = (uchar) 3.5 * (255.0 * (1.0 - (map->map[i][j] < 0 ? 1 : map->map[i][j])) + 0.5);
			g_remission_map_img->at<uchar>(i, j) = 255 - aux;
		}
	}
	//cv::resize(*g_remission_map_img, *g_remission_map_img, cv::Size(350, 350));
	cv::Point pt(g_remission_map_img->cols/2.0, g_remission_map_img->rows/2.0);
	*g_remission_map_img = rotate(*g_remission_map_img, pt, 90);
}

void
road_map_to_image(carmen_map_p map)
{
	int x = 0, y = 0;
	road_prob *cell_prob;
	cv::Vec3b color;
	uchar blue;
	uchar green;
	uchar red;
	for (x = 0; x < map->config.x_size; x++)
	{
		for (y = 0; y < map->config.y_size; y++)
		{
			cell_prob = road_mapper_double_to_prob(&map->map[x][y]);
			road_mapper_cell_color(cell_prob, &blue, &green, &red);
			color[0] = blue;
			color[1] = green;
			color[2] = red;
			g_road_map_img->at<cv::Vec3b>(x, y) = color;
		}
	}
	//cv::resize(*g_road_map_img, *g_road_map_img, cv::Size(350, 350));
	cv::Point pt(g_road_map_img->cols/2.0, g_road_map_img->rows/2.0);
	*g_road_map_img = rotate(*g_road_map_img, pt, 90);
}

int
global_pos_on_map_q4(carmen_map_p *maps)
{
	carmen_map_p map = maps[0];
	double q4_width = (map->config.resolution * map->config.x_size / 3.0);
	double q4_height = (map->config.resolution * map->config.y_size / 3.0);
	double x_origin_q4 = 0.0;
	double y_origin_q4 = 0.0;
	int i;
	for (i = 0; i < VEC_SIZE; i++)
	{
		map = maps[i];
		x_origin_q4 = (map->config.x_origin + q4_width);
		y_origin_q4 = (map->config.y_origin + q4_height);

		if (g_global_pos.x >= x_origin_q4 && g_global_pos.x <  x_origin_q4 + q4_width &&
				g_global_pos.y >= y_origin_q4 && g_global_pos.y < y_origin_q4 + q4_height)
			return i;
	}
	return -1;
}

int
maps_has_same_origin(carmen_map_p map1, carmen_map_p map2)
{
	return (map1->config.x_origin == map2->config.x_origin &&
			map1->config.y_origin == map2->config.y_origin);
}

static void
generate_offset_samples(int x, int y, carmen_map_config_t *map_conf)
{
	int i;
	double offset_i = 0;
	int offset_i_x = 0;
	int offset_i_y = 0;
	double distance_offset = g_distance_offset / map_conf->resolution;
	int x_sample_origin = 0;
	int y_sample_origin = 0;
	cv::Rect roi;
	cv::Mat remission_sample;
	cv::Mat road_sample;
	char name[256];
	for (i = -g_n_offsets; i <= g_n_offsets; i++)
	{
		offset_i = i * distance_offset;
		offset_i_x = offset_i * sin(g_global_pos.theta);
		offset_i_y = -offset_i * cos(g_global_pos.theta);

		x_sample_origin = (int) round((x + offset_i_x) - g_sample_width / 2);
		y_sample_origin = (int) round((y + offset_i_y) - g_sample_height / 2);

		// roi point is on the top-left corner
		roi = cv::Rect(cv::Point(x_sample_origin, y_sample_origin), cv::Size(g_sample_width, g_sample_height));
		remission_sample = (*g_remission_map_img)(roi);
		road_sample = (*g_road_map_img)(roi);
		//sprintf(name, "remission_sample_%d", offset_i);
		cv::imshow("remission", remission_sample);
		//sprintf(name, "road_sample_%d", offset_i);
		cv::imshow("road", road_sample);
		cv::waitKey(0);
	}
	remission_sample.release();
	road_sample.release();
}

static void
generate_rotate_samples(int x, int y, carmen_map_config_t *map_conf)
{
	int i;
	double rotation_angle = 0;
	double delta_rotation = 360.0 / g_n_rotations;
	int x_sample_origin = x - g_sample_width / 2;
	int y_sample_origin = y - g_sample_height / 2;
	char name[256];
	cv::Rect roi;
	cv::Mat remission_sample;
	cv::Mat road_sample;
	cv::Mat remission_map_img_aux;
	cv::Mat road_map_img_aux;
	cv::Point pt = cv::Point(x, y);

	for (i = 1; i <= g_n_rotations; i++)
	{
		rotation_angle = i * delta_rotation;
		remission_map_img_aux = rotate(*g_remission_map_img, pt, rotation_angle);
		road_map_img_aux = rotate(*g_road_map_img, pt, rotation_angle);

		// roi point is on the top-left corner
		roi = cv::Rect(cv::Point(x_sample_origin, y_sample_origin), cv::Size(g_sample_width, g_sample_height));
		remission_sample = (remission_map_img_aux)(roi);
		road_sample = (road_map_img_aux)(roi);
		//sprintf(name, "remission_sample_%d", offset_i);
		cv::imshow("remission", remission_sample);
		//sprintf(name, "road_sample_%d", offset_i);
		cv::imshow("road", road_sample);
		cv::waitKey(0);
	}
	remission_map_img_aux.release();
	road_map_img_aux.release();
	remission_sample.release();
	road_sample.release();
}

static void
generate_samples(void)
{
	int i_remission_map = global_pos_on_map_q4(g_vec_remission_map);
	int i_road_map = global_pos_on_map_q4(g_vec_road_map);
	if (i_remission_map == -1 || i_road_map == -1)
	{
		printf("Could not find remission_map[%d] or road_map[%d]\n", i_remission_map, i_road_map);
		return;
	}

	carmen_map_p remission_map = g_vec_remission_map[i_remission_map];
	carmen_map_p road_map = g_vec_road_map[i_road_map];
	if (!maps_has_same_origin(remission_map, road_map))
	{
		printf("Sync remission_map[%d] and road_map[%d] doens't have same origin!\n", i_remission_map, i_road_map);
		return;
	}

	int x_map = round((g_global_pos.x - remission_map->config.x_origin) / remission_map->config.resolution);
	int y_map = round((g_global_pos.y - remission_map->config.y_origin) / remission_map->config.resolution);
	int x_img = x_map;
	int y_img = remission_map->config.y_size -1 -y_map;

	remission_map_to_image(remission_map);
	road_map_to_image(road_map);
	g_remission_map_img->at<uchar>(cv::Point(x_img, y_img)) = 255;
	g_road_map_img->at<cv::Vec3b>(cv::Point(x_img, y_img)) = cv::Vec3b(0, 0, 0);

	generate_offset_samples(x_img, y_img, &remission_map->config);
	//generate_rotate_samples(x_img, y_img, &remission_map->config);
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
		memcpy(&g_previous_global_pos, &g_global_pos, sizeof(carmen_point_t));
		generate_samples();
	}
}

static void
localize_map_handler(carmen_map_server_localize_map_message *msg)
{
	static int first_time = 1;
	static int i = 0;
	if (first_time == 1)
	{
		int i;
		for (i = 0; i < VEC_SIZE; i++)
		{
			carmen_grid_mapping_initialize_map(g_vec_remission_map[i],
												msg->config.x_size,
												msg->config.resolution, 'm');
		}
		g_remission_map_img = new cv::Mat(g_vec_remission_map[0]->config.y_size,
											g_vec_remission_map[0]->config.x_size,
											CV_8UC1);
		first_time = 0;
	}
	memcpy(g_vec_remission_map[i%VEC_SIZE]->complete_map,
			msg->complete_mean_remission_map,
			sizeof(double) * msg->size);
	memcpy(&g_vec_remission_map[i%VEC_SIZE]->config,
			&msg->config,
			sizeof(carmen_map_config_t));
	i++;
}

static void
road_map_handler(carmen_map_server_road_map_message *msg)
{
	static int first_time = 1;
	static int i = 0;
	if (first_time == 1)
	{
		int i;
		for (i = 0; i < VEC_SIZE; i++)
		{
			carmen_grid_mapping_initialize_map(g_vec_road_map[i],
												msg->config.x_size,
												msg->config.resolution, 'r');
		}
		g_road_map_img = new cv::Mat(g_vec_road_map[0]->config.y_size,
										g_vec_road_map[0]->config.x_size,
										CV_8UC3,
										cv::Scalar::all(0));
		first_time = 0;
	}
	memcpy(g_vec_road_map[i%VEC_SIZE]->complete_map,
			msg->complete_map,
			sizeof(double) * msg->size);
	memcpy(&g_vec_road_map[i%VEC_SIZE]->config,
			&msg->config,
			sizeof(carmen_map_config_t));
	i++;
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

carmen_map_p
alloc_map_pointer(void)
{
	carmen_map_p map;
	map = (carmen_map_p) calloc (1, sizeof(carmen_map_t));
	map->config.x_origin = map->config.y_origin = 0.0001;
	map->complete_map = NULL;
	map->map = NULL;
	return map;
}

void
free_map_pointer(carmen_map_p map)
{
	if (map->complete_map) free(map->complete_map);
	if (map->map) free(map->map);
	free(map);
}

static void
initialize_maps(void)
{
	int i;
	for (i = 0; i < VEC_SIZE; i++)
	{
		g_vec_remission_map[i] = alloc_map_pointer();
		g_vec_road_map[i] = alloc_map_pointer();
	}
}

void
deinitialize_maps(void)
{
	int i;
	for (i = 0; i < VEC_SIZE; i++)
	{
		free_map_pointer(g_vec_remission_map[i]);
		free_map_pointer(g_vec_road_map[i]);
	}

	g_remission_map_img->release();
	g_road_map_img->release();
}

void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		std::cout << "road_mapper_sampling: disconnected.\n";
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
