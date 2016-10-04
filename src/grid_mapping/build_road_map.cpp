/*
 * complete_map_to_bitmap.c
 *
 *  Created on: 30/08/2016
 *      Author: Thomas
 */

#include <dirent.h>
#include <stdio.h>
#include <carmen/carmen.h>
#include "grid_mapping.h"
#include <prob_map.h>

#include <prob_measurement_model.h>
#include <prob_transforms.h>
#include <carmen/mapper_interface.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <omp.h>
#include <vector>

static char *map_path = ".";
char new_complete_map_name[1024];


static char *map_name = ".";
static char *map_name_info = ".";

char *map_type = "mmmmmmm";

static void
read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] = {
			{(char *)"command_line",	 (char *)"map_path",	CARMEN_PARAM_STRING, &map_path,	 1, NULL},
			{(char *)"command_line",	 (char *)"map_name",	CARMEN_PARAM_STRING, &map_name,	 1, NULL},
	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}

static void
get_map_origin_by_filename(char *filename, double *x_origin, double *y_origin)
{
	char map_name[1000], *div_char, *aux;

	*x_origin = *y_origin = 0.0;

	if (filename == NULL)
		return;

	div_char = strrchr(filename, '/');

	if (div_char == NULL)
	{
		return;
	}

	div_char++;

	strcpy(map_name, div_char);

	div_char = strrchr(map_name, '_');

	if (div_char != NULL && (map_name[0] == 'm' || map_name[0] == 'g' || map_name[0] == 's' || map_name[0] == '2' || map_name[0] == 'c'
			|| map_name[0] == 'u' || map_name[0] == 'o' || map_name[0] == 'e'))
	{
		aux = strrchr(map_name, '.');

		*aux = '\0';
		*div_char = '\0';
		div_char++;

		if((isdigit(*(map_name + 1)) || *(map_name + 1) == '-') && (isdigit(*div_char) || *div_char == '-'))
		{
			*x_origin = atof(map_name + 1);
			*y_origin = atof(div_char);
		}
	}
}


int
carmen_grid_mapping_read_complete_map_type(char *map_path, carmen_map_t *map, char *map_type)
{
	FILE *file;
	char global_map_path[1000], aux[1000];
	int rtr;

	if (map_type[0] == 'u'){
		map_name = "%s/complete_map_sum.map";
		map_name_info = "%s/complete_map_sum.info";
	}else if(map_type[0] == 'o'){
		map_name = "%s/complete_map_count.map";
		map_name_info = "%s/complete_map_count.info";
	}else if(map_type[0] == 'e'){
		map_name = "%s/complete_map_mean.map";
		map_name_info = "%s/complete_map_mean.info";
	}else if(map_type[0] == 's'){
		map_name = "%s/complete_map_remission.map";
		map_name_info = "%s/complete_map_remission.info";
	}else{
		map_name = "%s/complete_map.map";
		map_name_info = "%s/complete_map.info";
	}

	sprintf(global_map_path, map_name, map_path);
	rtr = carmen_map_read_gridmap_chunk(global_map_path, map);

	sprintf(global_map_path, map_name_info, map_path);

	file = fopen(global_map_path, "r");

	if (file == NULL)
		fprintf(stderr, "Error: complete map not found!\n");

	fscanf(file, "%s\n", aux);
	map->config.x_origin = atof(aux);
	fscanf(file, "%s\n", aux);
	map->config.y_origin = atof(aux);
	fclose(file);

	return rtr;
}


void
segment_remission_map(carmen_map_t *remission_map, carmen_map_t *map, carmen_map_t *road_map)
{
	cv::Mat map_img = cv::Mat::zeros(remission_map->config.x_size, remission_map->config.y_size, CV_8UC1);
	cv::Mat occupancy_map_img = cv::Mat::zeros(remission_map->config.x_size, remission_map->config.y_size, CV_8UC1);
	cv::Mat offline_map_img = cv::Mat::zeros(remission_map->config.x_size, remission_map->config.y_size, CV_8UC3);

	int erosion_size = 1;
	cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE,
			cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
			cv::Point( erosion_size, erosion_size ));


	for (int i = 0; i < remission_map->config.x_size; i++)
	{
		for (int j = 0; j < remission_map->config.y_size; j++)
		{
			//if (remission_map->map[i][j] < 0.0)
			//continue;

			uchar aux = (uchar)((255.0 * (1.0 - (remission_map->map[i][j] < 0 ? 1 : remission_map->map[i][j]))) + 0.5);
			map_img.at<uchar>(i, j) = aux;

			aux = 255 * (map->map[i][j] > 0.5 ? 1.0 : 0.0);
			occupancy_map_img.at<uchar>(i, j) = aux;
		}
	}

	//	cv::Mat map_img1 = cv::Mat::zeros(600, 800, CV_8UC1);
	//	cv::resize(map_img, map_img1,map_img1.size());
	//	cv::imshow("map_img1", map_img1);

	std::vector<std::vector<cv::Point> > contours;
	std::vector<std::vector<cv::Point> > contours2;

	//cv::medianBlur(map_img, map_img, 3);

	cv::imshow("map_occupancy", occupancy_map_img);
	cv::imshow("map_img1", map_img);

	cv::threshold(map_img, map_img, 235/*isto pode ser calibrado*/, 255, cv::THRESH_BINARY);

	cv::imshow("map_img2", map_img);
	//	cv::Mat map_img2 = cv::Mat::zeros(600, 800, CV_8UC1);
	//	cv::resize(map_img, map_img2,map_img2.size());
	//	cv::imshow("map_img2", map_img2);

	cv::erode(map_img, map_img, element);

	cv::imshow("map_img3", map_img);
	//	cv::Mat map_img3 = cv::Mat::zeros(600, 800, CV_8UC1);
	//	cv::resize(map_img, map_img3,map_img3.size());
	//	cv::imshow("map_img3", map_img3);
	//	cv::waitKey(0);


	findContours(map_img.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));

	//	double max_area = -DBL_MAX;
	//	int index = 0;


	for (uint i = 0; i < contours.size(); i++)
	{
		double area = contourArea(contours[i])* map->config.resolution;

		if (area < 1000)
			drawContours(map_img, contours, i, CV_RGB(0, 0, 0), -1);
	}


	cv::imshow("map_img4", map_img);

	cv::dilate(map_img, map_img, element);
	cv::dilate(map_img, map_img, element);
	cv::dilate(map_img, map_img, element);
	cv::dilate(map_img, map_img, element);

	cv::erode(map_img, map_img, element);
	cv::erode(map_img, map_img, element);
	cv::erode(map_img, map_img, element);
	cv::erode(map_img, map_img, element);
	cv::erode(map_img, map_img, element);

	cv::erode(map_img, map_img, element);
	cv::erode(map_img, map_img, element);
	cv::erode(map_img, map_img, element);
	cv::erode(map_img, map_img, element);
	cv::erode(map_img, map_img, element);

	cv::imshow("map_img5", map_img);

	//	for (int i = 0; i < remission_map->config.x_size; i++)
	//	{
	//		for (int j = 0; j < remission_map->config.x_size; j++)
	//		{
	//			if (map->map[i][j] < 0.0)
	//			{
	//				offline_map_img.at<cv::Vec3b>(i, j) = cv::Vec3b(180, 0, 0);
	//				continue;
	//			}
	//
	//			if (map_img.at<uchar>(i, j) > 128)
	//			{
	//				offline_map_img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
	//				continue;
	//			}
	//
	//			uchar aux = 255 * (1.0 - map->map[i][j]);
	//			offline_map_img.at<cv::Vec3b>(i, j) = cv::Vec3b(aux, aux, aux);
	//		}
	//	}


	for (int i = 0; i < remission_map->config.x_size; i++)
	{
		for (int j = 0; j < remission_map->config.y_size; j++)
		{
			if (map_img.at<uchar>(i, j) > 128)
				road_map->map[i][j] = 0.0;
			else
				road_map->map[i][j] = 1.0;

		}

	}

	cv::imshow("map_imgf", map_img);
	cv::waitKey(0);
}


int
main(int argc, char **argv)
{

	FILE *file;
	char full_path[1000] = "";
	char map_name_offline[1000] = "";

	char full_path_offline_map[1000] = "";

	//	DIR *dp;
	//	struct dirent *dirp;
	//	int i;
	//	double x_origin, y_origin, size = 0.0;
	char complete_map_name[1024];

	carmen_map_t offline_map;
	carmen_map_t remission_map;
	carmen_map_t road_map;
	carmen_map_t block_map;

	//	carmen_map_t map1;
	//	carmen_map_t map_limpo;
	//	carmen_map_t map3;

	carmen_ipc_initialize(argc, argv);
	read_parameters(argc, argv);

	//carmen_grid_mapping_init_parameters(map_resolution, block_size_in_meters);

	//	carmen_grid_mapping_read_complete_map_type(map_path, &remission_map, "s");
	//	carmen_grid_mapping_read_complete_map_type(map_path, &offline_map, "m");
	//	carmen_grid_mapping_read_complete_map_type("../data/mapper_20160510_teste_f/", &complete_map_average, "m");
	//
	//
	//	carmen_grid_mapping_read_complete_map_type("../data/mapper_20160510_teste_sujo/", &map1, "m");
	//	carmen_grid_mapping_read_complete_map_type("../data/mapper_20160510_teste_limpo1/", &map_limpo, "m");
	//	carmen_grid_mapping_read_complete_map_type("../data/mapper_20160602-16h/", &map3, "m");

	strcpy(map_name_offline, map_name);
	map_name_offline[0] = 'm';


	strcat(full_path_offline_map, map_path);
	strcat(full_path_offline_map, "/");
	strcat(full_path_offline_map, map_name_offline); //

	strcat(full_path, map_path);
	strcat(full_path, "/");
	strcat(full_path, map_name); //

	//	strcpy(full_path_offline_map, full_path);
	//	full_path_offline_map[0] = 'm';

	carmen_map_read_gridmap_chunk(full_path, &remission_map);
	printf("block map read = %s \n",full_path);

	carmen_map_read_gridmap_chunk(full_path_offline_map, &offline_map);

	road_map.config.resolution = remission_map.config.resolution;
	road_map.config.x_origin = remission_map.config.x_origin;
	road_map.config.y_origin = remission_map.config.y_origin;
	road_map.config.x_size = remission_map.config.x_size;
	road_map.config.y_size = remission_map.config.y_size;
	road_map.complete_map = (double*) malloc(sizeof(double) * remission_map.config.x_size * remission_map.config.y_size);
	road_map.map = (double**)malloc(sizeof(double*) * remission_map.config.x_size);


	for (int x = 0; x < road_map.config.x_size; x++)
	{
		road_map.map[x] = &road_map.complete_map[x * road_map.config.y_size];

		//initializing map with unknown
		for (int y = 0; y < road_map.config.y_size; y++)
		{
			road_map.map[x][y] = 1.0;
		}
	}


	//	dp  = opendir(map_path);
	//
	//		for(dirp = readdir(dp), i = 0; dirp != NULL; dirp = readdir(dp))
	//		{
	//			if (dirp->d_name[0] != map_type[0])
	//				continue;
	//			get_map_origin_by_filename(full_path, &block_map.config.x_origin, &block_map.config.y_origin);
	//			build_complete_map(&block_map, &complete_map);

	segment_remission_map(&remission_map, &offline_map, &road_map);
	map_name[0] = 'r';
	//	free(block_map.complete_map);
	//	free(block_map.map);

	//	full_path[0] = '\0';
	//		}

	//		closedir(dp);



	strcat(new_complete_map_name, map_path);
	strcat(new_complete_map_name, map_name);
	carmen_grid_mapping_save_map(new_complete_map_name, &road_map);
	printf(" map save = %s \n",new_complete_map_name);

	//	sprintf(global_map_path, "%s/complete_map_road.info", map_path);
	//	file = fopen(global_map_path, "w");
	//
	//	fprintf(file, "%f\n%f\n", road_map.config.x_origin, road_map.config.y_origin);

	//	fclose(file);

	return 1;
}
