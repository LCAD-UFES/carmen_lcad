#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION == 3
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#else
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

#include <carmen/carmen.h>
#include <carmen/carmen_gps_wrapper.h>
#include <carmen/download_map_interface.h>
#include <carmen/grid_mapping.h>

#include "download_map.h"

carmen_vector_3D_t pose_last_map_downloaded;

static int last_x_origin = INT32_MAX;
static int last_y_origin = INT32_MAX;
static carmen_download_map_message download_map_message;
static int carmen_download_map_from_internet = 0;
//
// O mapa do google no zoom 19 possui 0.3 metros por pixel.
// Sendo assim, o mapa com dimensoes de 512x512 pixels, tera 153.6x153.6 metros.
//
// static double distance_to_download_new_map = 10.0;
// static double distance_to_download_new_map = 76.8; // 76.8 = (153.6 / 2.0)

static void
carmen_publish_download_map_message (carmen_download_map_message *download_map_message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData (CARMEN_DOWNLOAD_MAP_MESSAGE_NAME, download_map_message);
	carmen_test_ipc_exit (err, "Could not publish", CARMEN_DOWNLOAD_MAP_MESSAGE_FMT);
}


static IplImage*
download_map_from_google_maps (double latitude, double longitude)
{
	IplImage *img;

	int map_width = 512;

	char *maptype = (char *) "satellite";
	char *filename = (char *) "gps_image.png";

	get_image_from_gps (latitude, longitude, maptype, map_width, filename);

	img = cvLoadImage (filename, CV_LOAD_IMAGE_COLOR);

	return img;
}


static void
carmen_build_download_map_message (IplImage *img, carmen_vector_3D_t position, carmen_download_map_message *download_map_message)
{
	int i, j, p_img, p_msg;

	download_map_message->height = img->height;
	download_map_message->width = img->width;
	download_map_message->size = 3 * img->height * img->width;
	download_map_message->position = position;
	download_map_message->timestamp = carmen_get_time();
	download_map_message->host = carmen_get_host();

	download_map_message->image_data = (char *) calloc (3 * img->height * img->width, sizeof(char));

	for(i = 0; i < img->height; i++)
	{
		for(j = 0; j < img->width; j++)
		{
			p_msg = 3 * (i * img->width + j);
			p_img = (i * img->widthStep) + (3 * j);

			// opencv image is BGR and I want my image in rgb format ...
			download_map_message->image_data [p_msg + 2] = img->imageData[p_img + 0];
			download_map_message->image_data [p_msg + 1] = img->imageData[p_img + 1];
			download_map_message->image_data [p_msg + 0] = img->imageData[p_img + 2];
		}
	}
}


void
carmen_grid_mapping_get_map_origin2(carmen_point_t *global_pose, int *x_origin, int* y_origin)
{
	*x_origin = (floor(global_pose->x / 50) - 1) * 50;
	*y_origin = (floor(global_pose->y / 50) - 1) * 50;
}


void
get_map_origin (double x, double y, int *x_origin, int *y_origin)
{
	carmen_point_t pose;

	pose.x = x;
	pose.y = y;

	carmen_grid_mapping_get_map_origin2 (&pose, x_origin, y_origin);
}


void
format_map_path (int x_origin, int y_origin, char* map_filename)
{
	char *carmen_home = getenv("CARMEN_HOME");

	if (carmen_home == NULL)
		exit(printf("Could not get environment variable $CARMEN_HOME in create_texture()\n"));

	sprintf(map_filename,
			"%s/data/google_maps/m%d_%d.bmp",
			carmen_home,
			x_origin,
			y_origin);
}


void
save_map_image (IplImage *img, int origin_x, int origin_y)
{
	char map_filename [1024];

	format_map_path(origin_x, origin_y, map_filename);
	cvSaveImage(map_filename, img, NULL);
}


IplImage *
find_map_from_data (int origin_x, int origin_y)
{
	IplImage *map;
	char map_filename [1024];

	format_map_path(origin_x, origin_y, map_filename);
	map = cvLoadImage (map_filename, CV_LOAD_IMAGE_ANYCOLOR);

	return map;
}


int
new_map_is_necessary(int x_origin, int y_origin)
{
	if (x_origin != last_x_origin || y_origin != last_y_origin)
		return 1;
	else
		return 0;
}


static void
localize_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
	IplImage *map_img = NULL;
	carmen_vector_3D_t position;

	double latitude, longitude, elevation, map_center_x, map_center_y;
	int x_origin, y_origin;


	double robot_real_pos_x = msg->globalpos.x;
	double robot_real_pos_y = msg->globalpos.y;

	position.x = msg->globalpos.x;
	position.y = msg->globalpos.y;
	position.z = 0;

	get_map_origin (robot_real_pos_x, robot_real_pos_y, &x_origin, &y_origin);

	// o mapa do google tem 153.6 (512 pixels * 0.3 metros por pixel) metros de lado
	// como a origem calculada eh no canto inferior esquerdo do mapa, preciso somar meio lado
	// para calcular as coordenadas do meio do mapa

	map_center_x = (double)(x_origin + 75);
	map_center_y = (double)(y_origin + 75);

	// os valores sao colocados invertidos por causa do sistema de coordenadas do gps
	double gps_position_x = -map_center_y;
	double gps_position_y = map_center_x;

	carmen_Utm_Gdc3(gps_position_x, gps_position_y, 0, 24, 0, &latitude, &longitude, &elevation);

	if (new_map_is_necessary(x_origin, y_origin))
	{
		map_img = find_map_from_data (x_origin, y_origin);

		if (map_img == NULL)
		{
			if (carmen_download_map_from_internet)
			{
				printf("map downloaded from latitude: %lf e longitude: %lf\n", latitude, longitude);

				map_img = download_map_from_google_maps (latitude, longitude);
				save_map_image (map_img, x_origin, y_origin);
			}
		}

		if (map_img != NULL)
		{
			carmen_build_download_map_message (map_img, position, &download_map_message);

			download_map_message.map_center.x = map_center_x; // x_origin;
			download_map_message.map_center.y = map_center_y; // y_origin;
			download_map_message.map_center.z = 0.0;

			carmen_publish_download_map_message (&download_map_message);

			free (download_map_message.image_data);
			cvReleaseImage (&map_img);

			last_x_origin = x_origin;
			last_y_origin = y_origin;
		}
	}

}



void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("download_map: disconnected.\n");

		exit(0);
	}
}


void
carmen_download_map_define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_DOWNLOAD_MAP_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_DOWNLOAD_MAP_MESSAGE_FMT);

	carmen_test_ipc_exit(err, "Could not define", CARMEN_DOWNLOAD_MAP_MESSAGE_NAME);
}


static void
carmen_get_download_map_parameters (int argc, char** argv)
{
	carmen_param_t param_list[] = {
			{(char*) "download_map", (char*) "from_internet", CARMEN_PARAM_ONOFF, &carmen_download_map_from_internet, 0, NULL},
	};

	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}


int main(int argc, char **argv)
{
	srand(time(NULL));

	carmen_ipc_initialize(argc, argv);
	carmen_get_download_map_parameters (argc, argv);

	signal(SIGINT, shutdown_module);

	pose_last_map_downloaded.x = 9999999;
	pose_last_map_downloaded.y = 9999999;
	pose_last_map_downloaded.z = 9999999;

	carmen_download_map_define_messages();
	carmen_localize_ackerman_subscribe_globalpos_message(
			NULL, (carmen_handler_t) localize_globalpos_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();
	return (0);
}
