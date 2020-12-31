#include "road_mapper_utils.h"
#include <wordexp.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

static int g_remission = 0;
static int g_offline = 1;
static int g_up_north = 0;
static int g_split = 0;
char *g_out_dir = (char *) ".";
static int g_image_channels = 3;
cv::Mat *g_remission_map_img = NULL;
cv::Mat *g_remission_map_img3 = NULL;
static carmen_map_p g_remission_map = NULL;
cv::Mat *g_offline_map_img = NULL;
cv::Mat *g_offline_map_img3 = NULL;
static carmen_map_p g_offline_map = NULL;


void
rotate_90_counterclockwise(cv::Mat &img)
{
	uchar cell_1a, cell_1b, cell_1c, cell_1d;
	cv::Vec3b cell_3a, cell_3b, cell_3c, cell_3d;

	if (img.cols != img.rows)
		return;

	for (int row = 0; row < img.rows / 2; row++)
	{
		for (int col = 0; col < img.cols / 2; col++)
		{
			if (img.channels() == 1)
			{
				cell_1a = img.at<uchar>(row, col);
				cell_1b = img.at<uchar>(col, img.rows - 1 - row);
				cell_1c = img.at<uchar>(img.rows - 1 - row, img.cols - 1 - col);
				cell_1d = img.at<uchar>(img.cols - 1 - col, row);
				img.at<uchar>(row, col) = cell_1b;
				img.at<uchar>(col, img.rows - 1 - row) = cell_1c;
				img.at<uchar>(img.rows - 1 - row, img.cols - 1 - col) = cell_1d;
				img.at<uchar>(img.cols - 1 - col, row) = cell_1a;
			}
			else if (img.channels() == 3)
			{
				cell_3a = img.at<cv::Vec3b>(row, col);
				cell_3b = img.at<cv::Vec3b>(col, img.rows - 1 - row);
				cell_3c = img.at<cv::Vec3b>(img.rows - 1 - row, img.cols - 1 - col);
				cell_3d = img.at<cv::Vec3b>(img.cols - 1 - col, row);
				img.at<cv::Vec3b>(row, col) = cell_3b;
				img.at<cv::Vec3b>(col, img.rows - 1 - row) = cell_3c;
				img.at<cv::Vec3b>(img.rows - 1 - row, img.cols - 1 - col) = cell_3d;
				img.at<cv::Vec3b>(img.cols - 1 - col, row) = cell_3a;
			}
		}
	}
}


bool
map_img_is_empty(cv::Mat &map_img, char file_type)
{
	uchar empty_value = 0;
	cv::Vec3b empty_value3;

	if (file_type == 'h')
		empty_value = 253;
	else if (file_type == 'i')
		empty_value3 = cv::Vec3b(253, 253, 253);
	else if (file_type == 'n')
		empty_value = (255 + 144 + 30) / 3;
	else if (file_type == 'm')
		empty_value3 = cv::Vec3b(255, 144, 30);

	for (int row = 0; row < map_img.rows; row++)
	{
		for (int col = 0; col < map_img.cols; col++)
		{
			if (map_img.channels() == 1 && map_img.at<uchar>(row, col) != empty_value)
				return false;

			if (map_img.channels() == 3 && map_img.at<cv::Vec3b>(row, col) != empty_value3)
				return false;
		}
	}
	return true;
}


void
write_split(cv::Mat &map_img, char *path, char file_type, double x_origin, double y_origin, double x_meters, double y_meters)
{
	int split_rows = 3, split_cols = 3;
	char full_path[2000];
	double x, y;
	cv::Mat split_map;
	cv::Point split_origin;
	cv::Size split_size(map_img.cols / split_cols, map_img.rows / split_rows);
	double split_x_meters = x_meters / split_cols;
	double split_y_meters = y_meters / split_rows;

	for (int split_row = 0; split_row < split_rows; split_row++)
	{
		for (int split_col = 0; split_col < split_cols; split_col++)
		{
			split_origin.x = split_size.width  * split_col;
			split_origin.y = map_img.rows - split_size.height * (split_row + 1);
			split_map = map_img(cv::Rect(split_origin, split_size));
			if (map_img_is_empty(split_map, file_type))
				continue;
			x = x_origin + split_x_meters * split_col;
			y = y_origin + split_y_meters * split_row;
			sprintf(full_path, "%s/%c%.0lf_%.0lf.png", path, file_type, x, y);
			cv::imwrite(full_path, split_map);
		}
	}
}


void
save_remission_map_image(void)
{
	static double x = 0.0;
	static double y = 0.0;
	if (x != g_remission_map->config.x_origin || y != g_remission_map->config.y_origin)
	{
		char name[256];
		char path[512];
		x = g_remission_map->config.x_origin;
		y = g_remission_map->config.y_origin;
		double x_meters = g_remission_map->config.resolution * ((g_up_north) ? g_remission_map->config.y_size : g_remission_map->config.x_size);
		double y_meters = g_remission_map->config.resolution * ((g_up_north) ? g_remission_map->config.x_size : g_remission_map->config.y_size);
		double x_origin = (g_up_north) ? - (y + x_meters) : x;
		double y_origin = (g_up_north) ?  x : y;
		sprintf(name, "i%.0lf_%.0lf.png", x_origin, y_origin);
		if (g_image_channels == 1 || g_image_channels == '*')
		{
			remission_map_to_image(g_remission_map, g_remission_map_img, 1);
			sprintf(path, "%s/%c%s", g_out_dir, 'h', &name[1]);
			printf("saving remission map image %s\n", path);
			if (g_up_north)
				rotate_90_counterclockwise(*g_remission_map_img);
			if (g_split)
				write_split(*g_remission_map_img, g_out_dir, 'h', x_origin, y_origin, x_meters, y_meters);
			else
				cv::imwrite(path, *g_remission_map_img);
		}
		if (g_image_channels == 3 || g_image_channels == '*')
		{
			remission_map_to_image(g_remission_map, g_remission_map_img3, 3);
			sprintf(path, "%s/%c%s", g_out_dir, 'i', &name[1]);
			printf("saving remission map image %s\n", path);
			if (g_up_north)
				rotate_90_counterclockwise(*g_remission_map_img3);
			if (g_split)
				write_split(*g_remission_map_img3, g_out_dir, 'i', x_origin, y_origin, x_meters, y_meters);
			else
				cv::imwrite(path, *g_remission_map_img3);
		}
	}
}


void
save_offline_map_image(void)
{
	static double x = 0.0;
	static double y = 0.0;
	if (x != g_offline_map->config.x_origin || y != g_offline_map->config.y_origin)
	{
		char name[256];
		char path[512];
		x = g_offline_map->config.x_origin;
		y = g_offline_map->config.y_origin;
		double x_meters = g_offline_map->config.resolution * ((g_up_north) ? g_offline_map->config.y_size : g_offline_map->config.x_size);
		double y_meters = g_offline_map->config.resolution * ((g_up_north) ? g_offline_map->config.x_size : g_offline_map->config.y_size);
		double x_origin = (g_up_north) ? - (y + x_meters) : x;
		double y_origin = (g_up_north) ?  x : y;
		sprintf(name, "m%.0lf_%.0lf.png", x_origin, y_origin);
		if (g_image_channels == 1 || g_image_channels == '*')
		{
			offline_map_to_image(g_offline_map, g_offline_map_img, 1);
			sprintf(path, "%s/%c%s", g_out_dir, 'n', &name[1]);
			printf("saving offline map image %s\n", path);
			if (g_up_north)
				rotate_90_counterclockwise(*g_offline_map_img);
			if (g_split)
				write_split(*g_offline_map_img, g_out_dir, 'n', x_origin, y_origin, x_meters, y_meters);
			else
				cv::imwrite(path, *g_offline_map_img);
		}
		if (g_image_channels == 3 || g_image_channels == '*')
		{
			offline_map_to_image(g_offline_map, g_offline_map_img3, 3);
			sprintf(path, "%s/%c%s", g_out_dir, 'm', &name[1]);
			printf("saving offline map image %s\n", path);
			if (g_up_north)
				rotate_90_counterclockwise(*g_offline_map_img3);
			if (g_split)
				write_split(*g_offline_map_img3, g_out_dir, 'm', x_origin, y_origin, x_meters, y_meters);
			else
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
	fprintf(stderr,   "         %*c   -up_north  {on|off}  -split   {on|off}\n", (int) strlen(prog_name), ' ');
	fprintf(stderr,   "default: %s   -remission  off      -offline  on       -out_dir .      -image_channels 3\n", prog_name);
	fprintf(stderr,   "         %*c   -up_north   off      -split    off\n\n", (int) strlen(prog_name), ' ');
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
			{(char *) "commandline",  (char*) "up_north",			CARMEN_PARAM_ONOFF, 	&(g_up_north),		0, NULL},
			{(char *) "commandline",  (char*) "split",				CARMEN_PARAM_ONOFF, 	&(g_split),			0, NULL},
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
	memcpy(g_remission_map->complete_map, msg->complete_mean_remission_map, sizeof(double) * msg->size);
	memcpy(&g_remission_map->config, &msg->config, sizeof(carmen_map_config_t));

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
	memcpy(g_offline_map->complete_map, msg->complete_map, sizeof(double) * msg->size);
	memcpy(&g_offline_map->config, &msg->config, sizeof(carmen_map_config_t));

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
