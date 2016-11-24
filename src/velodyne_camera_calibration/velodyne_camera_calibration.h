/*
 * velodyne_camera_calibration.h
 *
 *  Created on: Oct 20, 2016
 *      Author: vinicius
 */

#ifndef SRC_VELODYNE_CAMERA_CALIBRATION_VELODYNE_CAMERA_CALIBRATION_H_
#define SRC_VELODYNE_CAMERA_CALIBRATION_VELODYNE_CAMERA_CALIBRATION_H_
#include <vector>

typedef struct {
	carmen_velodyne_points_in_cam_t velodyne_points_in_cam;
	bool hit_in_obstacle;
} carmen_velodyne_points_in_cam_with_obstacle_t, *carmen_velodyne_points_in_cam_with_obstacle_p;

void
carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(carmen_velodyne_partial_scan_message *velodyne_message);

std::vector<carmen_velodyne_points_in_cam_t>
carmen_velodyne_camera_calibration_lasers_points_in_camera(carmen_velodyne_partial_scan_message *velodyne_message,
		int image_width, int image_height);

std::vector<carmen_velodyne_points_in_cam_t>
carmen_velodyne_camera_calibration_lasers_points_in_camera_with_obstacle(carmen_velodyne_partial_scan_message *velodyne_message,
		int image_width, int image_height);

std::vector<carmen_velodyne_points_in_cam_with_obstacle_t>
carmen_velodyne_camera_calibration_lasers_points_in_camera_with_obstacle_and_display(carmen_velodyne_partial_scan_message *velodyne_message,
		int image_width, int image_height);


//std::vector<carmen_velodyne_points_in_cam_t>
//carmen_velodyne_camera_calibration_lasers_points_bounding_box(carmen_velodyne_partial_scan_message *velodyne_message,
//		carmen_bumblebee_basic_stereoimage_message *bumblebee_message, double *confidence, bounding_box *box);

#endif /* SRC_VELODYNE_CAMERA_CALIBRATION_VELODYNE_CAMERA_CALIBRATION_H_ */
