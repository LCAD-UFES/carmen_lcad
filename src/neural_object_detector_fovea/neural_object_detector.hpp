#ifndef SRC_NEURAL_OBJECT_DETECTOR_NEURAL_OBJECT_DETECTOR_HPP_
#define SRC_NEURAL_OBJECT_DETECTOR_NEURAL_OBJECT_DETECTOR_HPP_

#include <carmen/carmen.h>
#include <carmen/carmen_darknet_interface.hpp>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_camera_calibration.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/traffic_light_interface.h>
#include <carmen/traffic_light_messages.h>
#include <carmen/rddf_messages.h>
#include <carmen/laser_ldmrs_utils.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <carmen/tf.h>
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>
#include <fstream>

using namespace std;
using namespace cv;



typedef struct
{
	double translate_factor_x;
	double translate_factor_y;
	double scale_factor_x;
	double scale_factor_y;
}t_transform_factor;


typedef struct
{
	int i;
	carmen_position_t camera_pos;
	carmen_position_t rddf_pos;
	vector <carmen_position_t> rddf_list;
}debug_infos;


carmen_vector_3D_t
rotate_point(carmen_vector_3D_t point, double theta);


carmen_vector_3D_t
translate_point(carmen_vector_3D_t point, carmen_vector_3D_t offset);


bool
check_rect_inside_image (cv::Rect rec, cv::Mat img);


void
get_image_crops(vector<cv::Mat> &scene_crops, vector<t_transform_factor> &transform_factor_of_slice_to_original_frame,
		cv::Mat image, vector<carmen_position_t> rddf_points_in_image_filtered,
		vector<double> distances_of_rddf_from_car);


vector<carmen_position_t>
get_rddf_points_in_image_full (tf::StampedTransform world_to_camera_pose,
		carmen_behavior_selector_road_profile_message last_rddf_poses, carmen_camera_parameters camera_parameters,
		int img_width, int img_height);


double
euclidean_distance (double x1, double x2, double y1, double y2);


void
carmen_translate_2d(double *x, double *y, double offset_x, double offset_y);


int
get_closest_rddf_index(double *camera_pose_x, double *camera_pose_y, carmen_pose_3D_t camera_pose, carmen_pose_3D_t board_pose,
		carmen_point_t globalpos, carmen_behavior_selector_road_profile_message last_rddf_poses);


vector<carmen_position_t>
get_rddf_points_in_image_filtered_by_meters_spacement(double meters_spacement, vector<double> &distances_of_rddf_from_car, tf::StampedTransform world_to_camera_pose,
		carmen_pose_3D_t camera_pose, carmen_pose_3D_t board_pose, carmen_point_t globalpos,
		carmen_behavior_selector_road_profile_message last_rddf_poses, vector<debug_infos> closest_rddf, carmen_camera_parameters camera_parameters,
		int img_width, int img_height);


vector<bbox_t>
get_predictions_of_crops (int i, cv::Mat image, void *network_struct, char **classes_names);


float
intersectionOverUnion(bbox_t box1, bbox_t box2);


float
calc_percentage_of_rectangles_intersection(cv::Point l1, cv::Point r1, cv::Point l2, cv::Point r2);


vector<bbox_t>
transform_bounding_boxes_of_crops (vector< vector<bbox_t> > bounding_boxes_of_crops, vector<t_transform_factor> transform_factor_of_slice_to_original_frame,
		cv::Mat or_image, char **classes_names);


void
show_detections_alberto(vector<t_transform_factor> transform_factor_of_slice_to_original_frame, vector<cv::Mat> scene_crops,
		vector<vector<bbox_t>> bounding_boxes_of_crops, vector<bbox_t> predictions,
		vector<carmen_position_t> rddf_points_in_image_filtered, int qtd_crops, char **classes_names, char *groundtruth_path, double image_timestamp);


vector<cv::Scalar>
get_slice_colors (unsigned int crops_size);


//void
//save_detections(double timestamp, vector<bbox_t> bounding_boxes_of_crops_in_original_image, cv::Mat rgb_image,
//				vector<cv::Mat> scene_crops, vector<cv::Scalar> colors, vector<t_transform_factor> transform_factor_of_slice_to_original_frame,
//				vector<carmen_position_t> rddf_points_in_image_filtered, vector<carmen_position_t> rddf_points_in_image_full);



#endif /* SRC_NEURAL_OBJECT_DETECTOR_NEURAL_OBJECT_DETECTOR_HPP_ */
