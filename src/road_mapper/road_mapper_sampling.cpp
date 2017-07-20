#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
#include <carmen/road_mapper.h>

#include <wordexp.h>
#include "road_mapper_utils.h"

static carmen_point_t g_global_pos;
static int g_sample_width = 0;
static int g_sample_height = 0;
static double g_distance_samples = 0.0;
static int g_n_offsets = 0;
static int g_n_rotations = 0;
static double g_distance_offset = 0.0;
wordexp_t g_out_path_p;
char* g_out_path;

cv::Mat *g_road_map_img;
cv::Mat *g_remission_map_img;

#define VEC_SIZE 20
static carmen_map_p g_vec_remission_map[VEC_SIZE];
static carmen_map_p g_vec_road_map[VEC_SIZE];

void
generate_rotate_samples(int x, int y,
						double offset_i_meters)
{
	int i;
	double rotation_angle = 0;
	double delta_rotation = 360.0 / g_n_rotations;
	int x_sample_origin = x - g_sample_width / 2;
	int y_sample_origin = y - g_sample_height / 2;
	char name[256];
	char path[512];
	cv::Rect roi;
	cv::Point pt = cv::Point(x, y);
	cv::Mat remission_sample;
	cv::Mat remission_map_img_aux;
	cv::Mat road_sample;
	cv::Mat road_map_img_aux;
	for (i = 0; i < g_n_rotations; i++)
	{
		rotation_angle = i * delta_rotation;
		remission_map_img_aux = rotate(*g_remission_map_img, pt, rotation_angle);
		road_map_img_aux = rotate(*g_road_map_img, pt, rotation_angle);

		// ROI point is on the top-left corner
		roi = cv::Rect(cv::Point(x_sample_origin, y_sample_origin),
						cv::Size(g_sample_width, g_sample_height));
		remission_sample = remission_map_img_aux(roi);
		road_sample = road_map_img_aux(roi);

		sprintf(name, "i%.0lf_%.0lf_%.2lf_%.2lf.png",
										g_global_pos.x, g_global_pos.y,
										offset_i_meters, rotation_angle);
		sprintf(path, "%s/%s", g_out_path, name);
		cv::imwrite(path, remission_sample);
		name[0] = 'r';
		sprintf(path, "%s/%s", g_out_path, name);
		cv::imwrite(path, road_sample);
	}
	remission_map_img_aux.release();
	road_map_img_aux.release();
	remission_sample.release();
	road_sample.release();
}

void
generate_offset_samples(int x, int y, double resolution)
{
	int i;
	int offset_i_x = 0;
	int offset_i_y = 0;
	double offset_i = 0;
	double offset_i_meters = 0;
	double distance_offset = g_distance_offset / resolution;
	int x_sample_center = 0;
	int y_sample_center = 0;
	for (i = -g_n_offsets; i <= g_n_offsets; i++)
	{
		offset_i_meters = i * g_distance_offset;
		offset_i = i * distance_offset;
		// OpenCV Y axis is downward, but theta Y-axis is upward. So we change theta sign.
		offset_i_x = offset_i * sin(-g_global_pos.theta);
		// OpenCV Y axis is downward, but theta Y-axis is upward. So we change theta sign.
		offset_i_y = -offset_i * cos(-g_global_pos.theta);

		x_sample_center = x + offset_i_x;
		y_sample_center = y + offset_i_y;

		generate_rotate_samples(x_sample_center,
								y_sample_center,
								offset_i_meters);
	}
}

int
generate_samples(void)
{
	int i_remission_map = global_pos_on_map_q4(g_global_pos, g_vec_remission_map, VEC_SIZE);
	int i_road_map = global_pos_on_map_q4(g_global_pos, g_vec_road_map, VEC_SIZE);
	if (i_remission_map == -1 || i_road_map == -1)
	{
		printf("Could not find remission_map[%d] or road_map[%d]\n", i_remission_map, i_road_map);
		return -1;
	}

	carmen_map_p remission_map = g_vec_remission_map[i_remission_map];
	carmen_map_p road_map = g_vec_road_map[i_road_map];
	if (!maps_has_same_origin(remission_map, road_map))
	{
		printf("Sync remission_map[%d] and road_map[%d] doens't have same origin!\n", i_remission_map, i_road_map);
		return -1;
	}

	int x_map = round((g_global_pos.x - remission_map->config.x_origin) / remission_map->config.resolution);
	int y_map = round((g_global_pos.y - remission_map->config.y_origin) / remission_map->config.resolution);
	int x_img = x_map;
	int y_img = remission_map->config.y_size -1 -y_map;

	remission_map_to_image(remission_map, g_remission_map_img);
	road_map_to_image(road_map, g_road_map_img);
	//g_remission_map_img->at<uchar>(cv::Point(x_img, y_img)) = 255;
	//g_road_map_img->at<cv::Vec3b>(cv::Point(x_img, y_img)) = cv::Vec3b(0, 0, 0);
	generate_offset_samples(x_img, y_img, remission_map->config.resolution);
	return 0;
}

static void
read_parameters(int argc, char **argv)
{
	char *out_path = (char *)".";
	char **w;
	carmen_param_t param_list[] =
	{
			{(char*)"road_mapper",  (char*)"sample_width", 		CARMEN_PARAM_INT, 		&(g_sample_width), 		0, NULL},
			{(char*)"road_mapper",  (char*)"sample_height",		CARMEN_PARAM_INT, 		&(g_sample_height), 	0, NULL},
			{(char*)"road_mapper",  (char*)"distance_sample",	CARMEN_PARAM_DOUBLE, 	&(g_distance_samples), 	0, NULL},
			{(char*)"road_mapper",  (char*)"n_offset",			CARMEN_PARAM_INT, 		&(g_n_offsets), 		0, NULL},
			{(char*)"road_mapper",  (char*)"n_rotation",		CARMEN_PARAM_INT, 		&(g_n_rotations), 		0, NULL},
			{(char*)"road_mapper",  (char*)"distance_offset",	CARMEN_PARAM_DOUBLE, 	&(g_distance_offset),	0, NULL},
			{(char*)"road_mapper",  (char*)"out_path",			CARMEN_PARAM_STRING, 	&(out_path),			0, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	// expand environment variables on path to full path
	wordexp(out_path, &g_out_path_p, 0 );
	w = g_out_path_p.we_wordv;
	g_out_path = *w;
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
	static carmen_point_t previous_global_pos;
	int err = 0;
	if (first_time == 1)
	{
		memcpy(&previous_global_pos, &msg->globalpos, sizeof(carmen_point_t));
		first_time = 0;
	}
	memcpy(&g_global_pos, &msg->globalpos, sizeof(carmen_point_t));
	double distance = carmen_distance(&previous_global_pos, &g_global_pos);
	//printf("#####%.2lf %.2lf %.2lf (%.2lf %.2lf)\n", g_global_pos.x, g_global_pos.y, distance,
	//													previous_global_pos.x, previous_global_pos.y);
	if (distance >= g_distance_samples)
	{
		//printf("%.2lf %.2lf %.2lf (%.2lf %.2lf)\n", g_global_pos.x, g_global_pos.y, distance,
		//													previous_global_pos.x, previous_global_pos.y);
		err = generate_samples();
		if (err == 0)
		{
			memcpy(&previous_global_pos, &g_global_pos, sizeof(carmen_point_t));
		}
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
															CARMEN_SUBSCRIBE_ALL);

	carmen_map_server_subscribe_localize_map_message(NULL,
														(carmen_handler_t) localize_map_handler,
														CARMEN_SUBSCRIBE_ALL);

	carmen_map_server_subscribe_road_map(NULL,
												(carmen_handler_t) road_map_handler,
												CARMEN_SUBSCRIBE_ALL);
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

	wordfree(&g_out_path_p);
}

void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("road_mapper_sampling: disconnected.\n");
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
