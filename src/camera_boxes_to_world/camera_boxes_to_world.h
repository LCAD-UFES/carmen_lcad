/*
 * camera_boxes_to_world.h
 *
 *  Created on: 13 de jun de 2017
 *      Author: luan
 */

#ifndef SRC_CAMERA_BOXES_TO_WORLD_CAMERA_BOXES_TO_WORLD_H_
#define SRC_CAMERA_BOXES_TO_WORLD_CAMERA_BOXES_TO_WORLD_H_

#include <carmen/carmen.h>
#include <carmen/velodyne_camera_calibration.h>
#include <vector>


typedef struct {
	carmen_image_coord_t pt1;
	carmen_image_coord_t pt2;
} bounding_box;


std::vector< std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> >
velodyne_points_in_boxes(std::vector<bounding_box> bouding_boxes_list,
                         carmen_velodyne_partial_scan_message *velodyne_sync_with_cam,
                         carmen_camera_parameters camera_parameters,
                         carmen_pose_3D_t velodyne_pose, carmen_pose_3D_t camera_pose,
                         unsigned int width, unsigned int height);

carmen_vector_3D_t
box_position(std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> laser_points_in_camera_box);

carmen_vector_3D_t
compute_centroid(std::vector<carmen_vector_3D_t> points_inside_box);

std::vector<std::vector<carmen_vector_3D_t> >
get_cluster_list(std::vector< std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> > laser_points_in_camera_box_list);

#endif /* SRC_CAMERA_BOXES_TO_WORLD_CAMERA_BOXES_TO_WORLD_H_ */
