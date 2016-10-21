/*
 * velodyne_camera_calibration.h
 *
 *  Created on: Oct 20, 2016
 *      Author: vinicius
 */

#ifndef SRC_VELODYNE_CAMERA_CALIBRATION_VELODYNE_CAMERA_CALIBRATION_H_
#define SRC_VELODYNE_CAMERA_CALIBRATION_VELODYNE_CAMERA_CALIBRATION_H_
#include <vector>

void
carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(carmen_velodyne_partial_scan_message *velodyne_message);
std::vector<carmen_velodyne_points_in_cam_t>
carmen_velodyne_camera_calibration_lasers_points_in_camera(carmen_velodyne_partial_scan_message *velodyne_message,
		carmen_bumblebee_basic_stereoimage_message *bumblebee_message);


#endif /* SRC_VELODYNE_CAMERA_CALIBRATION_VELODYNE_CAMERA_CALIBRATION_H_ */
