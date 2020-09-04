#include "road_mapper_utils.h"
#include <wordexp.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

static int g_remission = 0;
static int g_offline = 1;
char *g_out_dir = (char *) ".";
static int g_image_channels = 3;
cv::Mat *g_remission_map_img = NULL;
cv::Mat *g_remission_map_img3 = NULL;
static carmen_map_p g_remission_map = NULL;
cv::Mat *g_offline_map_img = NULL;
cv::Mat *g_offline_map_img3 = NULL;
static carmen_map_p g_offline_map = NULL;


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
		if (g_image_channels == 1 || g_image_channels == '*')
		{
			remission_map_to_image(g_remission_map, g_remission_map_img, 1);
			sprintf(path, "%s/%c%s", g_out_dir, 'h', &name[1]);
			printf("saving remission map image %s\n", path);
			cv::imwrite(path, *g_remission_map_img);
		}
		if (g_image_channels == 3 || g_image_channels == '*')
		{
			remission_map_to_image(g_remission_map, g_remission_map_img3, 3);
			sprintf(path, "%s/%c%s", g_out_dir, 'i', &name[1]);
			printf("saving remission map image %s\n", path);
			cv::imwrite(path, *g_remission_map_img3);
		}
	}
}


void
save_offline_map_image(void)
{
	static double x = 0;
	static double y = 0;
	if (x != g_offline_map->config.x_origin || y != g_offline_map->config.y_origin)
	{
		char name[256];
		char path[512];
		x = g_offline_map->config.x_origin;
		y = g_offline_map->config.y_origin;
		sprintf(name, "m%.0lf_%.0lf.png", x, y);
		if (g_image_channels == 1 || g_image_channels == '*')
		{
			offline_map_to_image(g_offline_map, g_offline_map_img, 1);
			sprintf(path, "%s/%c%s", g_out_dir, 'n', &name[1]);
			printf("saving offline map image %s\n", path);
			cv::imwrite(path, *g_offline_map_img);
		}
		if (g_image_channels == 3 || g_image_channels == '*')
		{
			offline_map_to_image(g_offline_map, g_offline_map_img3, 3);
			sprintf(path, "%s/%c%s", g_out_dir, 'm', &name[1]);
			printf("saving offline map image %s\n", path);
			cv::imwrite(path, *g_offline_map_img3);
		}
	}
}


void
prog_usage(char *prog_name, const char *error_msg = NULL)
{
	if (error_msg)
		fprintf(stderr, "\n%s\n", error_msg);

	fprintf(stderr, "\nUsage:   %s   -remission {on|off}  -offline {on|off}  -out_dir <dir>  -image_channels {1|3|'*'}\n", prog_name);
	fprintf(stderr,   "default: %s   -remission  off      -offline  on       -out_dir .      -image_channels 3\n\n", prog_name);
	exit(-1);
}


static void
read_parameters(int argc, char **argv)
{
	char *out_dir = NULL, *image_channels = NULL;

	carmen_param_t param_list[] =
	{
			{(char *) "commandline",  (char*) "remission",			CARMEN_PARAM_ONOFF, 	&(g_remission),		0, NULL},
			{(char *) "commandline",  (char*) "offline",			CARMEN_PARAM_ONOFF, 	&(g_offline),		0, NULL},
			{(char *) "commandline",  (char*) "out_dir",			CARMEN_PARAM_STRING, 	&(out_dir),			0, NULL},
			{(char *) "commandline",  (char*) "image_channels",		CARMEN_PARAM_STRING, 	&(image_channels),	0, NULL},
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

	if (!g_remission && !g_offline)
	{
		prog_usage(argv[0], "neither remission nor offline map was chosen");
	}

	if (out_dir)
	{
		// expand environment variables on path to full path
		wordexp_t we_out_dir;
		wordexp(out_dir, &we_out_dir, 0);
		g_out_dir = realpath(*we_out_dir.we_wordv, NULL);
		wordfree(&we_out_dir);

		struct stat st_out_dir;
		int st = stat(g_out_dir, &st_out_dir);
		if (st != 0 || !S_ISDIR(st_out_dir.st_mode))
			prog_usage(argv[0], "invalid out_dir");
	}

	if (image_channels)
	{
		if(strcmp(image_channels, "1") == 0 || strcmp(image_channels, "3") == 0)
			g_image_channels = atoi(image_channels);
		else if(strcmp(image_channels, "*") == 0)
			g_image_channels = '*';
		else
			prog_usage(argv[0], "invalid image_channels");
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

		if (g_image_channels == 1 || g_image_channels == '*')
		{
			g_remission_map_img = new cv::Mat(g_remission_map->config.y_size,
												g_remission_map->config.x_size,
												CV_8UC1);
		}
		if (g_image_channels == 3 || g_image_channels == '*')
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
offline_map_handler(carmen_map_server_offline_map_message *msg)
{
	static int first_time = 1;
	if (first_time == 1)
	{
		carmen_grid_mapping_initialize_map(g_offline_map,
											msg->config.x_size,
											msg->config.resolution, 'm');

		if (g_image_channels == 1 || g_image_channels == '*')
		{
			g_offline_map_img = new cv::Mat(g_offline_map->config.y_size,
											g_offline_map->config.x_size,
											CV_8UC1);
		}
		if (g_image_channels == 3 || g_image_channels == '*')
		{
			g_offline_map_img3 = new cv::Mat(g_offline_map->config.y_size,
											g_offline_map->config.x_size,
											CV_8UC3,
											cv::Scalar::all(0));
		}
		first_time = 0;
	}
	memcpy(g_offline_map->complete_map,
			msg->complete_map,
			sizeof(double) * msg->size);
	memcpy(&g_offline_map->config,
			&msg->config,
			sizeof(carmen_map_config_t));

	save_offline_map_image();
}


static void
register_handlers()
{
	if (g_remission)
		carmen_map_server_subscribe_localize_map_message(NULL, (carmen_handler_t) localize_map_handler, CARMEN_SUBSCRIBE_LATEST);
	if (g_offline)
		carmen_map_server_subscribe_offline_map(NULL, (carmen_handler_t) offline_map_handler, CARMEN_SUBSCRIBE_LATEST);
}


static void
initialize_maps(void)
{
	if (g_remission)
		g_remission_map = alloc_map_pointer();
	if (g_offline)
		g_offline_map = alloc_map_pointer();
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
	if (g_offline_map)
		free_map_pointer(g_offline_map);
	if (g_offline_map_img)
		g_offline_map_img->release();
	if (g_offline_map_img3)
		g_offline_map_img3->release();
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		deinitialize_maps();
		exit(0);
	}
}


int
main(int argc, char **argv)
{
	if (argc > 1 && strcmp(argv[1], "-h") == 0)
		prog_usage(argv[0]);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);

	signal(SIGINT, shutdown_module);

	initialize_maps();

	register_handlers();
	carmen_ipc_dispatch();

	return 0;
}
