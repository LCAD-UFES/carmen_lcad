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
#include <carmen/grid_mapping.h>

#include "download_map.h"

carmen_vector_3D_t pose_last_map_downloaded;

//
// O mapa do google no zoom 19 possui 0.3 metros por pixel.
// Sendo assim, o mapa com dimensoes de 512x512 pixels, tera 153.6x153.6 metros.
//
// static double distance_to_download_new_map = 10.0;
// static double distance_to_download_new_map = 76.8; // 76.8 = (153.6 / 2.0)

static IplImage*
download_map_from_google_maps (double latitude, double longitude)
{
	IplImage *img;

	int map_width = 600;

	char *maptype = "satellite";
	char *filename = "gps_image.png";

	get_image_from_gps (latitude, longitude, maptype, map_width, filename);

	img = cvLoadImage (filename, CV_LOAD_IMAGE_COLOR);

	return img;
}


static void
carmen_grid_mapping_get_map_origin2(carmen_point_t *global_pose, int *x_origin, int* y_origin)
{
	*x_origin = (floor(global_pose->x / 50.0) - 1) * 50;
	*y_origin = (floor(global_pose->y / 50.0) - 1) * 50;
}


static void
get_map_origin (double x, double y, int *x_origin, int *y_origin)
{
	carmen_point_t pose;

	pose.x = x;
	pose.y = y;

	carmen_grid_mapping_get_map_origin2 (&pose, x_origin, y_origin);
}


static void
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


static void
save_map_image (IplImage *img, int origin_x, int origin_y)
{
	char map_filename [1024];

	format_map_path(origin_x, origin_y, map_filename);
	cvSaveImage(map_filename, img, NULL);
}

static IplImage *
find_map_from_data (int origin_x, int origin_y)
{
	IplImage *map;
	char map_filename [1024];

	format_map_path(origin_x, origin_y, map_filename);
	map = cvLoadImage (map_filename, CV_LOAD_IMAGE_ANYCOLOR);

	return map;
}



static void
download(double first_x, double first_y, double end_x, double end_y)
{
	IplImage *map_img = NULL;

	//IplImage *big_map_img = cvCreateImage(cvSize(10000, 10000), IPL_DEPTH_8U, 3);

	double latitude, longitude, elevation, map_center_x, map_center_y;
	int x_origin, y_origin;//, hemi;
	//double img_center_x, img_center_y, img_center_z, zone;
	double x, y;
	int i, j;

	for (j=0, y = first_y; y <= end_y; y += 100.0, j++)
	{
		for (i=0, x = first_x; x <= end_x; x += 100.0, i++)
		{
			double robot_real_pos_x = x;
			double robot_real_pos_y = y;


			get_map_origin (robot_real_pos_x, robot_real_pos_y, &x_origin, &y_origin);

			// o mapa do google tem 153.6 (512 pixels * 0.3 metros por pixel) metros de lado
			// como a origem calculada eh no canto inferior esquerdo do mapa, preciso somar meio lado
			// para calcular as coordenadas do meio do mapa
			map_center_x = (double)(x_origin + 90);
			map_center_y = (double)(y_origin + 90);


			// os valores sao colocados invertidos por causa do sistema de coordenadas do gps
			double gps_position_x = -map_center_y;
			double gps_position_y = map_center_x;


			carmen_Utm_Gdc3(gps_position_x, gps_position_y, 0, 24, 0, &latitude, &longitude, &elevation);

			map_img = find_map_from_data (x_origin, y_origin);

			if (map_img == NULL)
			{
				map_img = download_map_from_google_maps (latitude, longitude);
				save_map_image (map_img, x_origin, y_origin);
			}
			else
			{
				static int count = 0;
				char map_filename [1024];
				char *carmen_home = getenv("CARMEN_HOME");
				sprintf(map_filename,
							"%s/data/google_maps/a%d.bmp",
							carmen_home,
							count);
				cvSaveImage(map_filename, map_img, NULL);
				count++;
			}
//			break;
		}
//		break;
	}
//	cvSaveImage("map.png", big_map_img, NULL);
//	printf("linhas %d colunas %d\n", i, j);
}



int
main(int argc, char **argv)
{
	argc = argc;
	double x = atof(argv[1]);
	double y = atof(argv[2]);

	double number_of_maps_x = atoi(argv[3]);
	double number_of_maps_y = atoi(argv[4]);

	double map_res = atof(argv[5]);

	double first_x = x - (number_of_maps_x / 2) * map_res;
	double first_y = y;

	printf("%lf %lf\n", first_x, first_y);

	double end_x = x + (number_of_maps_x / 2) * map_res;
	double end_y = y + number_of_maps_y * map_res;

	printf("%lf %lf\n", end_x, end_y);

	download(first_x, first_y, end_x, end_y);

	return (0);
}


//sprintf(aux, "%0.4lf", latitude);
//latitude = atof(aux);
//
//sprintf(aux, "%0.4lf", longitude);
//longitude = atof(aux);
//carmen_Gdc3_Utm(&img_center_x, &img_center_y, &img_center_z, &zone, &hemi, latitude, longitude, 0.0);



//if (first_time == 1)
//{
//	first_img_origin_x = -img_center_y + 90;
//	first_img_origin_y = img_center_x + 90;
//
//	first_time = 0;
//}
//
//cvTranspose(map_img, map_img);
//cvFlip(map_img, map_img, 1);
//
//bw = (int) ((img_center_x + 90 - first_img_origin_y)/0.3);
//bz = (int) (-(-img_center_y + 90 - first_img_origin_x)/0.3);
//
//printf("%d %d\n", bw, bz);
//
//
//for (w = 0; w < map_img->height; w++, bw++)
//{
//	bz = (int)(-(-img_center_y + 90 - first_img_origin_x)/0.3);
//	for (z = 0; z < map_img->width; z++, bz++)
//	{
//		if (bw > 0 && bz > 0)
//		{
//			big_map_img->imageData[bw * big_map_img->widthStep + 3 * bz] = map_img->imageData[(w) * map_img->widthStep + 3*(z)];
//			big_map_img->imageData[bw * big_map_img->widthStep + 3 * bz + 1] = map_img->imageData[(w) * map_img->widthStep + 3*(z)+1];
//			big_map_img->imageData[bw * big_map_img->widthStep + 3 * bz + 2] = map_img->imageData[(w) * map_img->widthStep + 3*(z)+2];
//		}
//	}
//}
