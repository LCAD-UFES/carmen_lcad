/*
 * camera_boxes_to_world.cpp
 *
 *  Created on: 13 de jun de 2017
 *      Author: luan
 */


#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_camera_calibration.h>

#include "camera_boxes_to_world.h"

std::vector<carmen_vector_3D_t>
camera_boxes_to_world(std::vector<bounding_box> bouding_boxes_list,
		carmen_velodyne_partial_scan_message *velodyne_sync_with_cam)
{

	std::vector<carmen_vector_3D_t> bounding_box_poses;

	carmen_velodyne_camera_calibration_lasers_points_in_camera_with_obstacle_and_display(
					velodyne_sync_with_cam, 0, 0);

	return bounding_box_poses;
}


