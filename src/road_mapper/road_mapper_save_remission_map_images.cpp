#include "road_mapper_utils.h"
#include <wordexp.h>

wordexp_t g_out_path_p;
char* g_out_path;
cv::Mat *g_remission_map_img = NULL;
cv::Mat *g_remission_map_img3 = NULL;
static carmen_map_p g_remission_map = NULL;
static int g_remission_image_channels = 0;

void
save_remission_map_image(void)
{
	static double x = 0;
	static double y = 0;
	if (x != g_remission_map->config.x_origin || y != g_remission_map->config.y_origin)
	{
		char name[256];
		char path[512];
		x = g_remission_map->config.x_origin;
		y = g_remission_map->config.y_origin;
		sprintf(name, "i%.0lf_%.0lf.png", x, y);
		if (g_remission_image_channels == 1 || g_remission_image_channels == '*')
		{
			remission_map_to_image(g_remission_map, g_remission_map_img, 1);
			sprintf(path, "%s/%c%s", g_out_path, 'h', &name[1]);
			printf("saving remission map image %s\n", path);
			cv::imwrite(path, *g_remission_map_img);
		}
		if (g_remission_image_channels == 3 || g_remission_image_channels == '*')
		{
			remission_map_to_image(g_remission_map, g_remission_map_img3, 3);
			sprintf(path, "%s/%c%s", g_out_path, 'i', &name[1]);
			printf("saving remission map image %s\n", path);
			cv::imwrite(path, *g_remission_map_img3);
		}
	}
}

static void
read_parameters(int argc, char **argv)
{
	char *out_path = (char *)".";
	char **w;
	char *remission_image_channels = (char *)"*";
	carmen_param_t param_list[] =
	{
			{(char*)"road_mapper",  (char*)"out_path_remission",			CARMEN_PARAM_STRING, 	&(out_path),					0, NULL},
			{(char*)"road_mapper",  (char*)"remission_image_channels",		CARMEN_PARAM_STRING, 	&(remission_image_channels),	0, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	// expand environment variables on path to full path
	wordexp(out_path, &g_out_path_p, 0 );
	w = g_out_path_p.we_wordv;
	g_out_path = *w;

	// image channels
	g_remission_image_channels = '*';
	if(strcmp(remission_image_channels, "1") == 0 || strcmp(remission_image_channels, "3") == 0)
	{
		g_remission_image_channels = atoi(remission_image_channels);
	}
}

static void
define_messages()
{
	carmen_map_server_define_localize_map_message();
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

		if (g_remission_image_channels == 1 || g_remission_image_channels == '*')
		{
			g_remission_map_img = new cv::Mat(g_remission_map->config.y_size,
												g_remission_map->config.x_size,
												CV_8UC1);
		}
		if (g_remission_image_channels == 3 || g_remission_image_channels == '*')
		{
			g_remission_map_img3 = new cv::Mat(g_remission_map->config.y_size,
												g_remission_map->config.x_size,
												CV_8UC3,
												cv::Scalar::all(0));
		}
		first_time = 0;
	}
	memcpy(g_remission_map->complete_map,
			msg->complete_mean_remission_map,
			sizeof(double) * msg->size);
	memcpy(&g_remission_map->config,
			&msg->config,
			sizeof(carmen_map_config_t));

	save_remission_map_image();
}

static void
register_handlers()
{
	carmen_map_server_subscribe_localize_map_message(NULL,
														(carmen_handler_t) localize_map_handler,
														CARMEN_SUBSCRIBE_LATEST);
}

static void
initialize_maps(void)
{
	g_remission_map = alloc_map_pointer();
}

void
deinitialize_maps(void)
{
	if (g_remission_map)
		free_map_pointer(g_remission_map);
	if (g_remission_map_img)
		g_remission_map_img->release();
	if (g_remission_map_img3)
		g_remission_map_img3->release();
	wordfree(&g_out_path_p);
}

void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("road_mapper_save_remission_map_images: disconnected.\n");
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
