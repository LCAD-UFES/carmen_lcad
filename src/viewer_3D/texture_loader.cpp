#include "texture_loader.h"
#include "download_map.h"
#include <math.h>

#include <carmen/carmen_gps_wrapper.h>

#if CV_MAJOR_VERSION == 3
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#else
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

static IplImage* ipl_image = NULL;

static char *map_image = NULL; //[3 * TEXTURE_SIZE * TEXTURE_SIZE];
static char *remission_map_image = NULL;

static carmen_vector_2D_t tex_coord_min;
static carmen_vector_2D_t tex_coord_max;

static carmen_vector_2D_t map_origin;
static carmen_vector_2D_t map_limit;

static double pixels_per_meter;

carmen_vector_2D_t get_map_image_tex_coord_min(void)
{
	return tex_coord_min;
}

carmen_vector_2D_t get_map_image_tex_coord_max(void)
{
	return tex_coord_max;
}

static void set_texture_coordinates(int x_origin_int, int y_origin_int, int square_size_int, double x_origin_double, double y_origin_double, double square_size_double)
{
	tex_coord_min.x = (x_origin_double - (double)(x_origin_int)) / (double)(square_size_int);
	tex_coord_min.y = (y_origin_double - (double)(y_origin_int)) / (double)(square_size_int);

	tex_coord_max.x = (x_origin_double + square_size_double - (double)(x_origin_int)) / (double)(square_size_int);
	tex_coord_max.y = (y_origin_double + square_size_double - (double)(y_origin_int)) / (double)(square_size_int);
}

static void
update_map_image_using_small_map(double map_center_x, double map_center_y, double square_size)
{
	// according to the following URL, the pixel size at equator in zoom level 19 are 30cm
	// => http://www.google.com.br/url?sa=t&rct=j&q=google%20number%20images%20tiles%20of%20zoom%20level&source=web&cd=3&ved=0CC0QFjAC&url=http%3A%2F%2Fwww.microimages.com%2Fdocumentation%2FTechGuides%2F76googleMapsStruc.pdf&ei=gpJTUOihOIWQ8wTM9YGIDg&usg=AFQjCNHcqDaejFREnp_sDF9oNQK1BcHMjA&cad=rja
	double meter_per_pixel = 0.3;
	pixels_per_meter = 1 / meter_per_pixel;

	double x_origin_in_meters_double = map_center_x - square_size / 2.0;
	double y_origin_in_meters_double = map_center_y - square_size / 2.0;

	double 	x_origin_in_pixels_double = x_origin_in_meters_double * pixels_per_meter;
	double 	y_origin_in_pixels_double = y_origin_in_meters_double * pixels_per_meter;
	int 	x_origin_in_pixels_int = (int)(x_origin_in_pixels_double + 0.5);
	int 	y_origin_in_pixels_int = (int)(y_origin_in_pixels_double + 0.5);

	double square_size_in_pixels_double = square_size * pixels_per_meter;
	set_texture_coordinates(x_origin_in_pixels_int, y_origin_in_pixels_int, TEXTURE_SIZE, x_origin_in_pixels_double, y_origin_in_pixels_double, square_size_in_pixels_double);

	memcpy (map_image, ipl_image->imageData, ipl_image->imageSize * sizeof(char));
}

static void
update_remission_map_image_using_small_map(double map_center_x, double map_center_y, double square_size)
{
	double meter_per_pixel = 0.2;
	pixels_per_meter = 1 / meter_per_pixel;

	double x_origin_in_meters_double = map_center_x - square_size / 2.0;
	double y_origin_in_meters_double = map_center_y - square_size / 2.0;

	double 	x_origin_in_pixels_double = x_origin_in_meters_double * pixels_per_meter;
	double 	y_origin_in_pixels_double = y_origin_in_meters_double * pixels_per_meter;
	int 	x_origin_in_pixels_int = (int)(x_origin_in_pixels_double + 0.5);
	int 	y_origin_in_pixels_int = (int)(y_origin_in_pixels_double + 0.5);

	double square_size_in_pixels_double = square_size * pixels_per_meter;
	set_texture_coordinates(x_origin_in_pixels_int, y_origin_in_pixels_int, REMISSION_MAP_SIZE, x_origin_in_pixels_double, y_origin_in_pixels_double, square_size_in_pixels_double);

	memcpy (remission_map_image, ipl_image->imageData, ipl_image->imageSize * sizeof(char));
}

void
rotate_map()
{
	IplImage *cpy = cvCreateImage (cvSize(ipl_image->width, ipl_image->height), ipl_image->depth, ipl_image->nChannels);
	cvCopy(ipl_image, cpy, NULL);

	cvConvertImage(cpy, ipl_image, CV_CVTIMG_FLIP);
	cvReleaseImage (&cpy);
}

char *
update_map_image_texture2(carmen_vector_3D_t map_center, double square_size, IplImage *img)
{
	if (map_image == NULL)
		map_image = (char *) calloc (TEXTURE_SIZE * TEXTURE_SIZE * 3, sizeof(char));

	ipl_image = img;
	rotate_map();
	update_map_image_using_small_map(map_center.x, map_center.y, square_size);

	return (map_image);
}

char *
update_remission_map_image_texture(carmen_vector_3D_t map_center, double square_size, IplImage *img)
{
	if (remission_map_image == NULL)
		remission_map_image = (char *) calloc (REMISSION_MAP_SIZE * REMISSION_MAP_SIZE * 3.1, sizeof(char));

	ipl_image = img;
	rotate_map();
	update_remission_map_image_using_small_map(map_center.x, map_center.y, square_size);

	return (remission_map_image);
}

int create_texture(void)
{
	char *CARMEN_HOME_ENVIRONMENT_VARIABLE;
	char texture_file_name[1024];

	CARMEN_HOME_ENVIRONMENT_VARIABLE = getenv("CARMEN_HOME");
	if (CARMEN_HOME_ENVIRONMENT_VARIABLE == NULL)
	{
		printf("Could not get environment variable $CARMEN_HOME in create_texture()\n");
		exit(1);
	}
	strcpy(texture_file_name, CARMEN_HOME_ENVIRONMENT_VARIABLE);
	strcat(texture_file_name, "/src/viewer_3D/Logo.png");
	ipl_image = NULL;
	ipl_image = cvLoadImage(texture_file_name, CV_LOAD_IMAGE_UNCHANGED);
	if (!ipl_image)
	{
		printf("Could not load image file %s in create_texture().\n", texture_file_name);
		exit(1);
	}

	unsigned int texture;

	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);


	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	//glTexImage2D(GL_TEXTURE_2D, 0, 3, ipl_image->width, ipl_image->height, 0, GL_BGR, GL_UNSIGNED_BYTE, ipl_image->imageData);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);


	//map_origin.x = 363403.043386 + 5.0;
	//map_origin.y = 7756427.475544 - 46.0;

	//map_limit.x = 364218.273548 + 5.0;
	//map_limit.y = 7758097.367853 - 46.0;

	map_origin.x = 363425.764824 - 13.5;
	map_origin.y = 7756427.087422 + 16.5;

	map_limit.x = 364204.889738 - 13.5;
	map_limit.y = 7758102.253080 + 16.5;


	// Ajuste que foi preciso fazer para encaixar a imagem com as coordenadas tiradas do google maps,
	// precisa conferir o valor destar coodernadas.
	//map_origin.x -= 5.0;
	//map_origin.y -= 40.0;
	//map_limit.x -= 5.0;
	//map_limit.y -= 40.0;

	double map_width_meters = map_limit.x - map_origin.x;
	pixels_per_meter = (double)(ipl_image->width) / map_width_meters;

	// This should yield about the same result
	//double map_height_meters = map_limit.y - map_origin.y;
	//pixels_per_meter = (double)(ipl_image->height) / map_height_meters;

	cvReleaseImage(&ipl_image);
	return texture;
}

unsigned int create_texture2(void)
{
	unsigned int texture_id;
    glGenTextures( 1, &texture_id );
	glBindTexture(GL_TEXTURE_2D, texture_id);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	return texture_id;
}
