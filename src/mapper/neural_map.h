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
#include <prob_measurement_model.h>
#include <prob_map.h>
#include <prob_interface.h>
#include <prob_measurement_model.h>
#include <prob_transforms.h>

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

#include "lib_neural_mapper_py.h"
#include <math.h>

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

	carmen_map_t normalized_max;
	carmen_map_t normalized_mean;
	carmen_map_t normalized_min;
	carmen_map_t normalized_numb;
	carmen_map_t normalized_std;

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
	carmen_map_t fixed_normalize_map_2(carmen_map_t value_map, double new_max, double last_max, double min);
	cv::Mat convert_to_rgb(carmen_map_t* complete_map, int x_size, int y_size);
	cv::Mat convert_prob_to_rgb(cv::Mat *image_prob, int x_size, int y_size);
	void convert_predicted_to_log_ods_snapshot_map(carmen_map_t* log_ods_snapshot, cv::Mat *image_prob, carmen_pose_3D_t *car_position, double x_origin, double y_origin);
	void fixed_normalize_map_all_maps(carmen_map_t *value_map, carmen_map_t *value_map2, carmen_map_t *value_map3, carmen_map_t *value_map4, carmen_map_t *value_map5);
	double fixed_normalize_cell(double value_map, double new_max, double last_max, double min);
	void map_to_png(carmen_map_t complete_map, char* csv_name, bool is_label, double map_max, double map_min, bool rgb_color=false);
	void save_map_as_png(carmen_map_t map, char* map_name, char* path, bool is_label, double rotation, double map_max, int map_index);
	void save_map_as_binary_file(carmen_map_t map, char* map_name, char* path, bool is_label, double rotation, double map_max, int map_index, double current_timestamp, carmen_pose_3D_t neural_mapper_robot_pose);
	void save_map_as_compact_map_binary_file(carmen_map_t map, char* map_name, char* path, bool is_label, double rotation, int map_index);
	void export_as_binary_file(char* path, int map_index, double current_timestamp, carmen_pose_3D_t neural_mapper_robot_pose);
	void update(Neural_map new_map, int pos);
	void push(Neural_map new_map);
	void acumulate_maps();
	void export_png(char* path, int max_index);
	void convertMapToChar();
	std::vector<cv::Mat> get_maps();
	cv::Mat map_to_png2(carmen_map_t complete_map, bool is_label, double map_max, double map_min, bool rgb_map);
	void foward_map(carmen_map_t *log_ods_snapshot, int size, carmen_pose_3D_t *car_position, double x_origin, double y_origin);//, char* map_name, char* path, bool is_label, double rotation, double map_max, int max_index);
	void map_to_png3(carmen_map_t complete_map, char* csv_name, double map_max, double map_min);
	void map_to_binary_file(carmen_map_t complete_map, char* csv_name, bool is_label, double map_max, double map_min, bool rgb_color=false);

};

#endif /* MAPPER2_NEURAL_MAP_H_ */

