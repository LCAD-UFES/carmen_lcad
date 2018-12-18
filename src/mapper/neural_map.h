/*
 * neural_map.h
 *
 *  Created on: Nov 29, 2018
 *      Author: vinicius
 */

#ifndef MAPPER2_NEURAL_MAP_H_
#define MAPPER2_NEURAL_MAP_H_

#include <tf.h>
#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>

#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION == 3
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#else
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

#include <vector>

class Neural_map
{
public:
	double car_x;
	double car_y;
	int size_x;
	int size_y;
	double resolution;
	double rotation;
	int neural_mapper_max_dist;

	carmen_map_t raw_min_hight_map;
	carmen_map_t raw_max_hight_map;
	carmen_map_t raw_mean_hight_map;
	carmen_map_t raw_number_of_lasers_map;
	carmen_map_t raw_square_sum_map;
	carmen_map_t neural_mapper_occupancy_map;

	Neural_map();
	Neural_map(int size_x, int size_y, double resolution, double car_x, double car_y, double car_rotation, int neural_mapper_max_dist);
	void set_car_x(double car_x);
	void set_car_y(double car_y);
	void set_car_rotation(double rotation);
	void clear_maps();
	void update_maps(Neural_map new_map);
};


class Neural_map_queue
{
public:
	int n_maps;
	Neural_map *neural_maps;
	Neural_map output_map = Neural_map();

	int size_x;
	int size_y;
	double resolution;
	Neural_map_queue();
	Neural_map_queue(int n_maps, int size_x, int size_y, double resolution, int dist);
	carmen_map_t fixed_normalize_map(carmen_map_t value_map, double new_max, double last_max, double min);
	void map_to_png(carmen_map_t complete_map, char* csv_name, bool is_label, double map_max, double map_min, bool rgb_color=false);
	void save_map(carmen_map_t map, char* map_name, char* path, bool is_label, double rotation, double map_max, int max_index);
	void update(Neural_map new_map, int pos);
	void push(Neural_map new_map);
	void acumulate_maps();
	void export_png(char* path, int max_index);
	void convertMapToChar();
	std::vector<cv::Mat> get_maps();
	cv::Mat map_to_png2(carmen_map_t complete_map, bool is_label, double map_max, double map_min, bool rgb_map);
	void foward_map(carmen_map_t map, char* map_name, char* path, bool is_label, double rotation, double map_max, int max_index);
	void map_to_png3(carmen_map_t complete_map, char* csv_name, double map_max, double map_min);

};

#endif /* MAPPER2_NEURAL_MAP_H_ */

