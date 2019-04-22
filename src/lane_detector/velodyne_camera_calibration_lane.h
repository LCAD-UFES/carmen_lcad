/*
 * velodyne_camera_calibration_lane.h
 *
 *  Created on: 13 de abr de 2019
 *      Author: marcelo
 */

#ifndef SRC_LANE_DETECTOR_VELODYNE_CAMERA_CALIBRATION_LANE_H_
#define SRC_LANE_DETECTOR_VELODYNE_CAMERA_CALIBRATION_LANE_H_
#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_camera_calibration.h>
#include <carmen/camera_boxes_to_world.h>
#include <carmen/carmen_darknet_interface.hpp>

typedef struct
{
	unsigned int a;
	unsigned int b;
	unsigned int c;
}lines;

tf::Point
spherical_to_cartesian(double hangle, double vangle, double range);

carmen_velodyne_partial_scan_message
find_velodyne_most_sync_with_cam(double bumblebee_timestamp);

void
carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(carmen_velodyne_partial_scan_message *velodyne_message);

lines
get_line(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);

lines
construct_the_line(bbox_t &predictions, bool left_or_right);

double
calculate_the_distance_point_to_the_line (lines line_aux, carmen_velodyne_points_in_cam_with_obstacle_t point_aux);

std::vector<carmen_velodyne_points_in_cam_with_obstacle_t>
carmen_velodyne_camera_calibration_lasers_points_in_camera_with_obstacle_and_display(carmen_velodyne_partial_scan_message *velodyne_message,
                                                                                     carmen_camera_parameters camera_parameters,
                                                                                     carmen_pose_3D_t velodyne_pose, carmen_pose_3D_t camera_pose,
                                                                                     int image_width, int image_height);

std::vector<carmen_velodyne_points_in_cam_with_obstacle_t>
locate_the_candidate_points_in_the_bounding_box(bbox_t &predictions,
		std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> &laser_points_in_camera_box_list,
		bool left_or_right);

#endif /* SRC_LANE_DETECTOR_VELODYNE_CAMERA_CALIBRATION_LANE_H_ */
