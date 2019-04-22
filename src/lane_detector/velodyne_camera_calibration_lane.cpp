/*
 * velodyne_camera_calibration_lane.cpp
 *
 *  Created on: 13 de abr de 2019
 *      Author: marcelo
 */

#include "velodyne_camera_calibration_lane.h"
#include <math.h>
const int MIN_ANGLE_OBSTACLE = 2;
const int MAX_ANGLE_OBSTACLE = 188;

#define MAX_RANGE 50.0
#define MIN_RANGE 0.5

tf::Point
spherical_to_cartesian(double hangle, double vangle, double range)
{
	double x, y, z;

	x = range * cos(vangle) * cos(hangle);
	y = range * cos(vangle) * sin(hangle);
	z = range * sin(vangle);

	return tf::Point(x, y, z);
}


const static double sorted_vertical_angles[32] =
{
		-30.67, -29.33, -28.0, -26.67, -25.33, -24.0, -22.67, -21.33, -20.0,
		-18.67, -17.33, -16.0, -14.67, -13.33, -12.0, -10.67, -9.3299999, -8.0,
		-6.6700001, -5.3299999, -4.0, -2.6700001, -1.33, 0.0, 1.33, 2.6700001, 4.0,
		5.3299999, 6.6700001, 8.0, 9.3299999, 10.67
};

const int column_correspondence[32] =
{
		0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23, 8,
		24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31
};
std::vector<carmen_velodyne_partial_scan_message> velodyne_vector;

carmen_velodyne_partial_scan_message
find_velodyne_most_sync_with_cam(double bumblebee_timestamp)
{
    carmen_velodyne_partial_scan_message velodyne;
    double minTimestampDiff = DBL_MAX;
    int minTimestampIndex = -1;
    for (unsigned int i = 0; i < velodyne_vector.size(); i++)
    {
        if (fabs(velodyne_vector[i].timestamp - bumblebee_timestamp) < minTimestampDiff)
        {
            minTimestampIndex = i;
            minTimestampDiff = fabs(velodyne_vector[i].timestamp - bumblebee_timestamp);
        }
    }
    velodyne = velodyne_vector[minTimestampIndex];
    return (velodyne);
}


void
carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(carmen_velodyne_partial_scan_message *velodyne_message)
{
	int i, j;
	unsigned short original_distances[32];
	unsigned char original_intensities[32];

	for (i = 0; i < velodyne_message->number_of_32_laser_shots; i++)
	{
		memcpy(original_distances, velodyne_message->partial_scan[i].distance, 32 * sizeof(unsigned short));
		memcpy(original_intensities, velodyne_message->partial_scan[i].intensity, 32 * sizeof(unsigned char));

		for (j = 0; j < 32; j++)
		{
			velodyne_message->partial_scan[i].distance[column_correspondence[j]] = original_distances[j];
			velodyne_message->partial_scan[i].intensity[column_correspondence[j]] = original_intensities[j];
		}
	}
}


lines
get_line(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2)
{
	// Line Equation a = yA - yB, b = xB - xA, c = xAyB - xByA
	unsigned int a = y1 - y2;
	unsigned int b = x2 - x1;
	unsigned int c = (x1 * y2) - (x2 * y1);

	lines line_aux;
	line_aux.a = a;
	line_aux.b = b;
	line_aux.c = c;
	return line_aux;
}

lines
construct_the_line(bbox_t &predictions, bool left_or_right)
{
	lines line;
	if (left_or_right)
	{
		line = get_line(predictions.x, predictions.y + predictions.h, predictions.x + predictions.w, predictions.y) ;
	}else
	{
		line = get_line(predictions.x, predictions.y, predictions.x + predictions.w, predictions.y + predictions.h);
	}

	return line;
}

double
calculate_the_distance_point_to_the_line (lines line_aux, carmen_velodyne_points_in_cam_with_obstacle_t point_aux)
{
	double distance = (abs(line_aux.a * point_aux.velodyne_points_in_cam.ipx + line_aux.b * point_aux.velodyne_points_in_cam.ipy + line_aux.c)
			/ sqrt(line_aux.a * line_aux.a + line_aux.b * line_aux.b));
	return distance;
}
std::vector<carmen_velodyne_points_in_cam_with_obstacle_t>
carmen_velodyne_camera_calibration_lasers_points_in_camera_with_obstacle_and_display(carmen_velodyne_partial_scan_message *velodyne_message,
                                                                                     carmen_camera_parameters camera_parameters,
                                                                                     carmen_pose_3D_t velodyne_pose, carmen_pose_3D_t camera_pose,
                                                                                     int image_width, int image_height)
{
	std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> laser_points_in_camera;

	double fx_meters = camera_parameters.fx_factor * image_width * camera_parameters.pixel_size;
	double fy_meters = camera_parameters.fy_factor * image_height * camera_parameters.pixel_size;

	double cu = camera_parameters.cu_factor * (double) image_width;
	double cv = camera_parameters.cv_factor * (double) image_height;

	for (int j = 0; j < velodyne_message->number_of_32_laser_shots; j++)
	{
		double h_angle = carmen_normalize_theta(carmen_degrees_to_radians(velodyne_message->partial_scan[j].angle));

		for (int i = 1; i < 32; i++)
		{
			double v_angle = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[i]));
			double v_angle_1 = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[i-1]));

			double range = (((double) velodyne_message->partial_scan[j].distance[i]) / 500.0);
			double range_1 = (((double) velodyne_message->partial_scan[j].distance[i-1]) / 500.0);

			if (range <= MIN_RANGE)
				range = MAX_RANGE;

			if (range > MAX_RANGE)
				range = MAX_RANGE;

			if (range >= MAX_RANGE)
				continue;

			tf::Point p3d_velodyne_reference = spherical_to_cartesian(h_angle, v_angle, range);
			tf::Point p3d_velodyne_reference_1 = spherical_to_cartesian(h_angle, v_angle_1, range_1);

			if (p3d_velodyne_reference.x() > 0)
			{
				//metodo de jose
				double delta_x = p3d_velodyne_reference.x() - p3d_velodyne_reference_1.x();

				//obstacle_z = global_point_position_in_the_world.z - (robot_position.z - robot_wheel_radius);
				double delta_z = (p3d_velodyne_reference.z()) - (p3d_velodyne_reference_1.z());//verificar o z no mapper
				double line_angle = carmen_radians_to_degrees(fabs(atan2(delta_z, delta_x)));

				bool obstacle = (line_angle > MIN_ANGLE_OBSTACLE) && (line_angle < MAX_ANGLE_OBSTACLE);

				tf::Point p3d_camera_reference = move_to_camera_reference(p3d_velodyne_reference,velodyne_pose,camera_pose);

                double px = (fx_meters * (p3d_camera_reference.y() / p3d_camera_reference.x()) / camera_parameters.pixel_size + cu);
                double py = (fy_meters * (-p3d_camera_reference.z() / p3d_camera_reference.x()) / camera_parameters.pixel_size + cv);

				int ipx = (int) px;
				int ipy = (int) py;

				if (ipx >= 0 && ipx <= image_width && ipy >= 0 && ipy <= image_height)
				{
					carmen_velodyne_points_in_cam_with_obstacle_t laser_px_points = {ipx, ipy, {h_angle,v_angle,range}, obstacle};

					laser_points_in_camera.push_back(laser_px_points);
				}

			}
		}
	}
	return laser_points_in_camera;
}

std::vector<carmen_velodyne_points_in_cam_with_obstacle_t>
locate_the_candidate_points_in_the_bounding_box(bbox_t &predictions,
		std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> &laser_points_in_camera_box_list,
		bool left_or_right)
{
	lines line_aux = construct_the_line(predictions, left_or_right);
	std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> candidate_points;
	for (int j = 0; j < laser_points_in_camera_box_list.size(); j++)
	{
		double distance_aux = calculate_the_distance_point_to_the_line(line_aux, laser_points_in_camera_box_list[j]);
		if (distance_aux <= 5)
		{
			candidate_points.push_back(laser_points_in_camera_box_list[j]);
		}
	}
	return candidate_points;
}


