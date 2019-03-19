#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string.h>
//./slice_map 150.0 512 /media/My\ Passport/parte_final.png 0 0 0 0 ../../data/google_block_map/ 7756300 -365250

int main(int argc __attribute__ ((unused)), char **argv)
{
//	double big_slice_size_in_meters;
//	int big_slice_size_in_pixels;
	int init_extra_pixels_in_coluns;
	int init_extra_pixels_in_lines;
//	int last_extra_pixels_in_coluns;
//	int last_extra_pixels_in_lines;
	//int slive_size_in_pixles;
	//double slive_size_in_meters;
	double origin_x, origin_y;
	carmen_map_t map;
	carmen_map_t map_count;

	char map_image_name[2000];
	char global_map_path[2000];
	char map_path[2000];


	//big_slice_size_in_meters = atof(argv[1]);
	//big_slice_size_in_pixels = atoi(argv[2]);
	strcpy(map_image_name, argv[3]);

	init_extra_pixels_in_coluns = atoi(argv[4]);
//	last_extra_pixels_in_coluns = atoi(argv[5]);
	init_extra_pixels_in_lines = atoi(argv[6]);
//	last_extra_pixels_in_lines = atoi(argv[7]);
	strcpy(map_path, argv[8]);
	origin_x = atof(argv[9]);
	origin_y = atof(argv[10]);


	//slive_size_in_meters = atof(argv[8]);

	//slive_size_in_pixles = slive_size_in_meters / (big_slice_size_in_pixels / big_slice_size_in_meters);

	IplImage *big_map_image_colour = cvLoadImage(map_image_name, CV_LOAD_IMAGE_COLOR);
	IplImage *big_map_image_gray = cvCreateImage(cvSize(big_map_image_colour->width, big_map_image_colour->height), IPL_DEPTH_8U, 1);
//	IplImage *big_map_image_gray2 = cvCreateImage(cvSize((int)(9250), (int)(10500)), IPL_DEPTH_8U, 1);

	cvCvtColor(big_map_image_colour, big_map_image_gray, CV_BGR2GRAY);
//	cvResize(big_map_image_gray, big_map_image_gray);

//	IplImage *new_big_map_image_gray = cvCreateImage(cvSize(init_extra_pixels_in_coluns + last_extra_pixels_in_coluns + big_map_image_gray->width,
//															init_extra_pixels_in_lines + last_extra_pixels_in_lines + big_map_image_gray->height), IPL_DEPTH_8U, 1);
	map.config.x_origin = origin_x;
	map.config.y_origin = origin_y;

	map_count.config.x_origin = origin_x;
	map_count.config.y_origin = origin_y;


	int i, j, k;
//	for (i = 0, j = 0; i < big_map_image_gray->height; i++, j++)
//	{
//		memcpy(&new_big_map_image_gray->imageData[j * new_big_map_image_gray->widthStep + init_extra_pixels_in_coluns], &big_map_image_gray->imageData[i * big_map_image_gray->widthStep], big_map_image_gray->width);
//	}

	carmen_grid_mapping_create_new_map(&map, big_map_image_gray->width, big_map_image_gray->height, 0.2, 'm');
	carmen_grid_mapping_create_new_map(&map_count, big_map_image_gray->width, big_map_image_gray->height, 0.2, 'm');


//	cvEqualizeHist(big_map_image_gray, big_map_image_gray);

	for (i = big_map_image_gray->height - 1, k = 0; i >= 0; i--, k++)
	{
		for (j = 0; j < big_map_image_gray->width; j++)
		{
			if ((unsigned char)big_map_image_gray->imageData[i * big_map_image_gray->widthStep + j] != 0)
				map.map[j][k] = ((unsigned char)big_map_image_gray->imageData[i * big_map_image_gray->widthStep + j] / 255.0);
			else
				map.map[j][k] = -1.0;

			map_count.map[j][k] = 1.0;
		}
	}

	sprintf(global_map_path, "%s/count_google_map.map", map_path);
	carmen_grid_mapping_save_map(global_map_path, &map_count);

	for(i = 0; i < map.config.x_size * map.config.y_size; i++)
	{
		map_count.complete_map[i] = -1.0;
	}

	for (k = 0; k < map.config.x_size - init_extra_pixels_in_lines; k++)
	{
		for (j = 0; j < map.config.y_size; j++)
		{
			if ((j - init_extra_pixels_in_coluns) > 0)
				map_count.map[k][j] = map.map[k + init_extra_pixels_in_lines][j - init_extra_pixels_in_coluns];
		}
	}

	sprintf(global_map_path, "%s/complete_map.map", map_path);
	carmen_grid_mapping_save_map(global_map_path, &map_count);

	sprintf(global_map_path, "%s/complete_map.info", map_path);
	FILE *file = fopen(global_map_path, "w+");

	fprintf(file, "%f\n%f\n", map.config.x_origin, map.config.y_origin);

	fclose(file);

	return 0;
}
