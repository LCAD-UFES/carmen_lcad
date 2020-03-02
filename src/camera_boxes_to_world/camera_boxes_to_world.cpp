/*
 * camera_boxes_to_world.cpp
 *
 *  Created on: 13 de jun de 2017
 *      Author: luan
 */

#include "camera_boxes_to_world.h"
#include <carmen/velodyne_interface.h>


std::vector< std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> >
velodyne_points_in_boxes(std::vector<bounding_box> bouding_boxes_list,
                         carmen_velodyne_partial_scan_message *velodyne_sync_with_cam,
                         carmen_camera_parameters camera_parameters,
                         carmen_pose_3D_t velodyne_pose, carmen_pose_3D_t camera_pose,
                         unsigned int width, unsigned int height)
{

	std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> points_lasers_in_cam_with_obstacle;

	// Removes the ground, Removes points outside cameras field of view and Returns the points that are obstacles
	points_lasers_in_cam_with_obstacle = carmen_velodyne_camera_calibration_lasers_points_in_camera_with_obstacle_and_display(
				velodyne_sync_with_cam,camera_parameters, velodyne_pose,camera_pose, width, height);

	std::vector< std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> > laser_points_in_camera_box_list;

	for (unsigned int i = 0; i < bouding_boxes_list.size(); i++)
	{
		std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> laser_points_in_camera_box;
		for (unsigned int j = 0; j < points_lasers_in_cam_with_obstacle.size(); j++)
		{
			if (points_lasers_in_cam_with_obstacle[j].velodyne_points_in_cam.ipx > bouding_boxes_list[i].pt1.x
					&& points_lasers_in_cam_with_obstacle[j].velodyne_points_in_cam.ipx < bouding_boxes_list[i].pt2.x
					&& points_lasers_in_cam_with_obstacle[j].velodyne_points_in_cam.ipy > bouding_boxes_list[i].pt1.y
					&& points_lasers_in_cam_with_obstacle[j].velodyne_points_in_cam.ipy < bouding_boxes_list[i].pt2.y
					&& points_lasers_in_cam_with_obstacle[j].hit_in_obstacle == true)
			{
				laser_points_in_camera_box.push_back(points_lasers_in_cam_with_obstacle[j]);
			}
		}

		laser_points_in_camera_box_list.push_back(laser_points_in_camera_box);
	}

	return laser_points_in_camera_box_list;
}


carmen_vector_3D_t
compute_centroid(std::vector<carmen_vector_3D_t> points_inside_box)
{
	carmen_vector_3D_t centroid;
	centroid.x = 0.0;
	centroid.y = 0.0;
	centroid.z = 0.0;

	for (unsigned int i = 0; i < points_inside_box.size(); i++)
	{
		centroid.x += points_inside_box[i].x;
		centroid.y += points_inside_box[i].y;
		centroid.z += points_inside_box[i].z;
	}

	centroid.x = centroid.x  / (double) points_inside_box.size();
	centroid.y = centroid.y  / (double) points_inside_box.size();
	centroid.z = centroid.z  / (double) points_inside_box.size();

	return (centroid);
}


carmen_vector_3D_t
box_position(std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> laser_points_in_camera_box)
{
	std::vector<carmen_vector_3D_t> points_inside_box;

	for (unsigned int i = 0; i < laser_points_in_camera_box.size(); i++)
	{
		/*
		 * The angles grow to right instead to the left.
		 */
		laser_points_in_camera_box[i].velodyne_points_in_cam.laser_polar.horizontal_angle = -laser_points_in_camera_box[i].velodyne_points_in_cam.laser_polar.horizontal_angle;

		points_inside_box.push_back(carmen_covert_sphere_to_cartesian_coord(laser_points_in_camera_box[i].velodyne_points_in_cam.laser_polar));

	}

	return (compute_centroid(points_inside_box));

}


std::vector<std::vector<carmen_vector_3D_t> >
get_cluster_list(std::vector< std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> > laser_points_in_camera_box_list)
{
	std::vector<std::vector<carmen_vector_3D_t> > cluster_list;
	for (unsigned int i = 0; i < laser_points_in_camera_box_list.size(); i++)
	{
		std::vector<carmen_vector_3D_t> cluster;
		for (unsigned int j = 0; j < laser_points_in_camera_box_list[i].size(); j++)
		{
			carmen_vector_3D_t p;
			laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.laser_polar.horizontal_angle = -laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.laser_polar.horizontal_angle;
			p = carmen_covert_sphere_to_cartesian_coord(laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.laser_polar);
			cluster.push_back(p);
		}
		cluster_list.push_back(cluster);
	}
	return (cluster_list);
}
