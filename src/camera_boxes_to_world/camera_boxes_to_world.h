/*
 * camera_boxes_to_world.h
 *
 *  Created on: 13 de jun de 2017
 *      Author: luan
 */

#ifndef SRC_CAMERA_BOXES_TO_WORLD_CAMERA_BOXES_TO_WORLD_H_
#define SRC_CAMERA_BOXES_TO_WORLD_CAMERA_BOXES_TO_WORLD_H_

#include <carmen/carmen.h>
#include <vector>


typedef struct {
	carmen_image_coord_t pt1;
	carmen_image_coord_t pt2;
} bounding_box;


std::vector< std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> >
velodyne_points_in_boxes(std::vector<bounding_box> bouding_boxes_list,
		carmen_velodyne_partial_scan_message *velodyne_sync_with_cam,
		unsigned int width, unsigned int height);

carmen_vector_3D_t
box_position(std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> laser_points_in_camera_box);


#endif /* SRC_CAMERA_BOXES_TO_WORLD_CAMERA_BOXES_TO_WORLD_H_ */
