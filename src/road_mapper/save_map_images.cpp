#include "road_mapper_utils.h"
#include <wordexp.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <carmen/grid_mapping.h>
#include <prob_map.h>
#include <prob_interface.h>
#include <prob_measurement_model.h>
#include <prob_transforms.h>


static int g_remission = 0;
static int g_offline = 1;
static int g_up_north = 0;
static int g_split = 0;
static int g_load_map_from_folder_mode = 0;
char *g_out_dir = (char *) ".";
char *g_input_dir = (char *) ".";
static int g_image_channels = 3;
cv::Mat *g_remission_map_img = NULL;
cv::Mat *g_remission_map_img3 = NULL;
cv::Mat *g_remission_map_img4 = NULL;
static carmen_map_p g_remission_map = NULL;
cv::Mat *g_offline_map_img = NULL;
cv::Mat *g_offline_map_img3 = NULL;
cv::Mat *g_offline_map_img4 = NULL;
static carmen_map_p g_offline_map = NULL;
carmen_position_t g_min_pose = { DBL_MAX,  DBL_MAX};
carmen_position_t g_max_pose = {-DBL_MAX, -DBL_MAX};
double g_resolution = 0.2;


void
rotate_90_counterclockwise(cv::Mat &img)
{
	uchar cell_1a, cell_1b, cell_1c, cell_1d;
	cv::Vec3b cell_3a, cell_3b, cell_3c, cell_3d;
	cv::Vec4b cell_4a, cell_4b, cell_4c, cell_4d;

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
			else if (img.channels() == 4)
			{
				cell_4a = img.at<cv::Vec4b>(row, col);
				cell_4b = img.at<cv::Vec4b>(col, img.rows - 1 - row);
				cell_4c = img.at<cv::Vec4b>(img.rows - 1 - row, img.cols - 1 - col);
				cell_4d = img.at<cv::Vec4b>(img.cols - 1 - col, row);
				img.at<cv::Vec4b>(row, col) = cell_4b;
				img.at<cv::Vec4b>(col, img.rows - 1 - row) = cell_4c;
				img.at<cv::Vec4b>(img.rows - 1 - row, img.cols - 1 - col) = cell_4d;
				img.at<cv::Vec4b>(img.cols - 1 - col, row) = cell_4a;
			}
		}
	}
}


void
get_empty_values(uchar *empty_value, cv::Vec3b *empty_value3, cv::Vec4b *empty_value4, char map_img_type)
{
	if (map_img_type == 'h' || map_img_type == 'i')
	{
		*empty_value = 253;
		*empty_value3 = cv::Vec3b(253, 253, 253);
		*empty_value4 = cv::Vec4b(253, 253, 253, 255);
	}
	if (map_img_type == 'n' || map_img_type == 'm')
	{
		*empty_value = (255 + 144 + 30) / 3;
		*empty_value3 = cv::Vec3b(255, 144, 30);
		*empty_value4 = cv::Vec4b(255, 144, 30, 255);
	}
}


bool
map_img_is_empty(cv::Mat &map_img, char map_img_type)
{
	uchar empty_value;
	cv::Vec3b empty_value3;
	cv::Vec4b empty_value4;
	get_empty_values(&empty_value, &empty_value3, &empty_value4, map_img_type);

	for (int row = 0; row < map_img.rows; row++)
	{
		for (int col = 0; col < map_img.cols; col++)
		{
			if (map_img.channels() == 1 && map_img.at<uchar>(row, col) != empty_value)
				return false;

			if (map_img.channels() == 3 && map_img.at<cv::Vec3b>(row, col) != empty_value3)
				return false;

			if (map_img.channels() == 4 && map_img.at<cv::Vec4b>(row, col) != empty_value4)
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
		if (g_image_channels == 4)
		{
			remission_map_to_image(g_remission_map, g_remission_map_img4, 4);
			sprintf(path, "%s/%c%s", g_out_dir, 'i', &name[1]);
			printf("saving remission map image %s\n", path);
			if (g_up_north)
				rotate_90_counterclockwise(*g_remission_map_img4);
			if (g_split)
				write_split(*g_remission_map_img4, g_out_dir, 'i', x_origin, y_origin, x_meters, y_meters);
			else
				cv::imwrite(path, *g_remission_map_img4);
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
		if (g_image_channels == 4)
		{
			offline_map_to_image(g_offline_map, g_offline_map_img4, 4);
			sprintf(path, "%s/%c%s", g_out_dir, 'm', &name[1]);
			printf("saving offline map image %s\n", path);
			if (g_up_north)
				rotate_90_counterclockwise(*g_offline_map_img4);
			if (g_split)
				write_split(*g_offline_map_img4, g_out_dir, 'm', x_origin, y_origin, x_meters, y_meters);
			else
				cv::imwrite(path, *g_offline_map_img4);
		}
	}
}


void
prog_usage(char *prog_name, const char *error_msg = NULL, const char *error_msg2 = NULL)
{
	if (error_msg)
		fprintf(stderr, "\n%s", error_msg);
	if (error_msg2)
		fprintf(stderr, "%s", error_msg2);

	fprintf(stderr, "\n\nUsage:   %s   -remission {on|off}  -offline {on|off}  -out_dir <dir>  -image_channels {1|3|4|'*'}\n", prog_name);
	fprintf(stderr,   "         %*c   -up_north  {on|off}  -split   {on|off}\n", (int) strlen(prog_name), ' ');
	fprintf(stderr,   "default: %s   -remission  off      -offline  on       -out_dir .      -image_channels 3\n", prog_name);
	fprintf(stderr,   "         %*c   -up_north   off      -split    off\n\n", (int) strlen(prog_name), ' ');
	fprintf(stderr, "To load from map folder:\nUsage:   %s   -input_dir <dir>     [the options above...] \n\n", prog_name);

	exit(-1);
}


static void
read_parameters(int argc, char **argv)
{
	char *result = NULL;
	char *image_channels = NULL;

	for (int i = 1; i < argc; i += 2)
	{
		if (strcmp(argv[i], "-remission") == 0)
			result = get_param(&g_remission, argc, argv, i, CARMEN_PARAM_ONOFF);
		else if (strcmp(argv[i], "-offline") == 0)
			result = get_param(&g_offline, argc, argv, i, CARMEN_PARAM_ONOFF);
		else if (strcmp(argv[i], "-out_dir") == 0)
			result = get_param(&g_out_dir, argc, argv, i, CARMEN_PARAM_DIR);
		else if (strcmp(argv[i], "-image_channels") == 0)
			result = get_param(&image_channels, argc, argv, i, CARMEN_PARAM_STRING);
		else if (strcmp(argv[i], "-up_north") == 0)
			result = get_param(&g_up_north, argc, argv, i, CARMEN_PARAM_ONOFF);
		else if (strcmp(argv[i], "-split") == 0)
			result = get_param(&g_split, argc, argv, i, CARMEN_PARAM_ONOFF);
		else if (strcmp(argv[i], "-input_dir") == 0)
		{
			result = get_param(&g_input_dir, argc, argv, i, CARMEN_PARAM_DIR);
			g_load_map_from_folder_mode = 1;
		}
		else
			prog_usage(argv[0], "Invalid option: ", argv[i]);
		if (result != NULL)
			prog_usage(argv[0], result);
	};

	if (!g_remission && !g_offline)
		prog_usage(argv[0], "Neither -remission nor -offline option was set on");

	if (image_channels)
	{
		if(strcmp(image_channels, "1") == 0 || strcmp(image_channels, "3") == 0 || strcmp(image_channels, "4") == 0)
			g_image_channels = atoi(image_channels);
		else if(strcmp(image_channels, "*") == 0)
			g_image_channels = '*';
		else
			prog_usage(argv[0], "Invalid -image_channels: ", image_channels);
	}
}


void
set_complete_map_limits(carmen_map_config_t config)
{
	double x_size_meters = (double) config.x_size * config.resolution;
	double y_size_meters = (double) config.y_size * config.resolution;

	if (config.x_origin < g_min_pose.x)
		g_min_pose.x = config.x_origin;

	if ((config.x_origin + x_size_meters) > g_max_pose.x)
		g_max_pose.x = config.x_origin + x_size_meters;

	if (config.y_origin < g_min_pose.y)
		g_min_pose.y = config.y_origin;

	if ((config.y_origin + y_size_meters) > g_max_pose.y)
		g_max_pose.y = config.y_origin + y_size_meters;

	if (config.resolution != g_resolution)
	{
		g_resolution = config.resolution;
		fprintf(stderr, "Warning: map is in a distinct resolution: %lf  origin: (%lf, %lf)  size: (%d, %d)\n",
				config.resolution, config.x_origin, config.y_origin, config.x_size, config.y_size);
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
		if (g_image_channels == 4 || g_image_channels == '*')
		{
			g_remission_map_img4 = new cv::Mat(g_remission_map->config.y_size,
												g_remission_map->config.x_size,
												CV_8UC4,
												cv::Scalar::all(0));
		}
		first_time = 0;
	}
	memcpy(g_remission_map->complete_map, msg->complete_mean_remission_map, sizeof(double) * msg->size);
	g_remission_map->config = msg->config;

	save_remission_map_image();
	set_complete_map_limits(msg->config);
}


static void
remission_block_map_handler(carmen_map_t *remission_block_map)
{
	carmen_map_server_localize_map_message msg;

	msg.complete_mean_remission_map = remission_block_map->complete_map;
	msg.size = (remission_block_map->config.x_size * remission_block_map->config.y_size);
	msg.config = remission_block_map->config;

	localize_map_handler(&msg);
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
		if (g_image_channels == 4 || g_image_channels == '*')
		{
			g_offline_map_img4 = new cv::Mat(g_offline_map->config.y_size,
											g_offline_map->config.x_size,
											CV_8UC4,
											cv::Scalar::all(0));
		}
		first_time = 0;
	}
	memcpy(g_offline_map->complete_map, msg->complete_map, sizeof(double) * msg->size);
	g_offline_map->config = msg->config;

	save_offline_map_image();
	set_complete_map_limits(msg->config);
}


static void
offline_block_map_handler(carmen_map_t *offline_block_map)
{
	carmen_map_server_offline_map_message msg;

	msg.complete_map = offline_block_map->complete_map;
	msg.size = (offline_block_map->config.x_size * offline_block_map->config.y_size);
	msg.config = offline_block_map->config;

	offline_map_handler(&msg);
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
	if (g_remission_map_img4)
		g_remission_map_img4->release();
	if (g_offline_map)
		free_map_pointer(g_offline_map);
	if (g_offline_map_img)
		g_offline_map_img->release();
	if (g_offline_map_img3)
		g_offline_map_img3->release();
	if (g_offline_map_img4)
		g_offline_map_img4->release();
}


static int
get_map_origin_by_filename(char *full_path, double *x_origin, double *y_origin)
{
	char *filename, *file_extension;
	double x, y;
	int pos;

	*x_origin = *y_origin = 0.0;

	filename = strrchr(full_path, '/');
	if (filename == NULL)
		filename = full_path;
	else
		filename++;

	file_extension = strrchr(filename, '.');
	if (file_extension == NULL)
		file_extension = filename + strlen(filename);
	else
		file_extension++;

	if (sscanf(filename, "%*c%lf_%lf%n", &x, &y, &pos) == 2 && (filename + pos) == file_extension)
	{
		*x_origin = x;
		*y_origin = y;
		return 0;
	}
	return -1;
}


void
build_complete_map_image(char map_img_type)
{
	uchar empty_value;
	cv::Vec3b empty_value3;
	cv::Vec4b empty_value4;
	get_empty_values(&empty_value, &empty_value3, &empty_value4, map_img_type);
	int x_size, y_size;
	double x_origin, y_origin, x, y;

	if (g_up_north)
	{
		x_size = round((g_max_pose.y - g_min_pose.y) / g_resolution);
		y_size = round((g_max_pose.x - g_min_pose.x) / g_resolution);
		x_origin = - g_max_pose.y;
		y_origin = g_min_pose.x;
	}
	else
	{
		x_size = round((g_max_pose.x - g_min_pose.x) / g_resolution);
		y_size = round((g_max_pose.y - g_min_pose.y) / g_resolution);
		x_origin = g_min_pose.x;
		y_origin = g_min_pose.y;
	}

	cv::Mat complete_map;
	if (g_image_channels == 4)
		complete_map = cv::Mat(y_size, x_size, CV_8UC4, empty_value4);
	else if (map_img_type == 'h' || map_img_type == 'n')
		complete_map = cv::Mat(y_size, x_size, CV_8UC1, empty_value);
	else
		complete_map = cv::Mat(y_size, x_size, CV_8UC3, empty_value3);

	char full_path[1000];
	struct dirent *dirp;
	DIR *dp  = opendir(g_out_dir);

	while ((dirp = readdir(dp)) != NULL)
	{
		sprintf(full_path, "%s/%s", g_out_dir, dirp->d_name);
		struct stat buf;
		if (stat(full_path, &buf) != 0 || S_ISDIR(buf.st_mode))
			continue;

		if (dirp->d_name[0] == map_img_type && strcmp(&(dirp->d_name[strlen(dirp->d_name) - 4]), ".png") == 0)
		{
			if (get_map_origin_by_filename(full_path, &x, &y) == 0)
			{
				cv::Mat map = cv::imread(full_path, (map_img_type == 'h' || map_img_type == 'n') ? cv::IMREAD_GRAYSCALE : cv::IMREAD_UNCHANGED);
				if (!map.empty())
				{
					int left = round((x - x_origin) / g_resolution);
					int top  = round(y_size - (y - y_origin) / g_resolution) - map.rows;
					map.copyTo(complete_map(cv::Rect(left, top, map.cols, map.rows)));
					map.release();
				}
			}
		}
	}

	closedir(dp);
	sprintf(full_path, "%s/complete_%c%d_%d.png", g_out_dir, map_img_type, int(x_origin), int(y_origin));
	cv::imwrite(full_path, complete_map);
	fprintf(stderr, "%s generated\n", full_path);
}


void
build_complete_map_images()
{
	if (g_offline && (g_image_channels == 1 || g_image_channels == '*'))
		build_complete_map_image('n');
	if (g_offline && (g_image_channels == 3 || g_image_channels == 4 || g_image_channels == '*'))
		build_complete_map_image('m');
	if (g_remission && (g_image_channels == 1 || g_image_channels == '*'))
		build_complete_map_image('h');
	if (g_remission && (g_image_channels == 3 || g_image_channels == 4 || g_image_channels == '*'))
		build_complete_map_image('i');
}


void
load_and_save_map_from_folder()
{
	carmen_map_t offline_block_map, remission_block_map, remission_count_block_map;
	memset(&offline_block_map, 0, sizeof(carmen_map_t));
	memset(&remission_block_map, 0, sizeof(carmen_map_t));
	memset(&remission_count_block_map, 0, sizeof(carmen_map_t));

	char full_path[1000];
	struct dirent *dirp;
	DIR *dp  = opendir(g_input_dir);

	while ((dirp = readdir(dp)) != NULL)
	{
		sprintf(full_path, "%s/%s", g_input_dir, dirp->d_name);
		struct stat buf;
		if (stat(full_path, &buf) != 0 || S_ISDIR(buf.st_mode))
			continue;

		if (g_offline && dirp->d_name[0] == 'm' && strcmp(&(dirp->d_name[strlen(dirp->d_name) - 4]), ".map") == 0)
		{
			if (get_map_origin_by_filename(full_path, &offline_block_map.config.x_origin, &offline_block_map.config.y_origin) == 0 &&
				carmen_map_read_gridmap_chunk(full_path, &offline_block_map) == 0)
			{
				offline_block_map_handler(&offline_block_map);
			}
			carmen_map_free_gridmap(&offline_block_map);
		}

		if (g_remission && dirp->d_name[0] == 's' && strcmp(&(dirp->d_name[strlen(dirp->d_name) - 4]), ".map") == 0)
		{
			if (get_map_origin_by_filename(full_path, &remission_block_map.config.x_origin, &remission_block_map.config.y_origin) == 0 &&
				carmen_map_read_gridmap_chunk(full_path, &remission_block_map) == 0)
			{
				sprintf(full_path, "%s/c%d_%d.map", g_input_dir, int(remission_block_map.config.x_origin), int(remission_block_map.config.y_origin));
				remission_count_block_map.config = remission_block_map.config;
				if (carmen_map_read_gridmap_chunk(full_path, &remission_count_block_map) == 0)
				{
					carmen_prob_models_calc_mean_remission_map(&remission_block_map, &remission_block_map, &remission_count_block_map);
					remission_block_map_handler(&remission_block_map);
				}
				carmen_map_free_gridmap(&remission_count_block_map);
			}
			carmen_map_free_gridmap(&remission_block_map);
		}
	}

	closedir(dp);
	build_complete_map_images();
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		fprintf(stderr,"\nInterrupt signal received\n\n");
		if(!g_load_map_from_folder_mode)
		{
			carmen_ipc_disconnect();
			build_complete_map_images();
		}
		deinitialize_maps();
		exit(0);
	}
}


int
main(int argc, char **argv)
{
	if (argc > 1 && strcmp(argv[1], "-h") == 0)
		prog_usage(argv[0]);

	read_parameters(argc, argv);

	signal(SIGINT, shutdown_module);

	initialize_maps();

	if(g_load_map_from_folder_mode)
	{
		printf("Running input data mode\n");
		load_and_save_map_from_folder();
	}
	else
	{
		printf("Running Carmen mode\n");
		carmen_ipc_initialize(argc, argv);
		carmen_param_check_version(argv[0]);
		register_handlers();
		carmen_ipc_dispatch();
	}

	return 0;
}
