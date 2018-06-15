
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <carmen/carmen.h>
#include <carmen/visual_tracker_interface.h>
#include "velodyne_camera_calibration.h"

#include <tf.h>

const int MIN_ANGLE_OBSTACLE = 2;
const int MAX_ANGLE_OBSTACLE = 188;

#define MAX_RANGE 50.0
#define MIN_RANGE 0.5

const int column_correspondence[32] =
{
		0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23, 8,
		24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31
};

const static double sorted_vertical_angles[32] =
{
		-30.67, -29.33, -28.0, -26.67, -25.33, -24.0, -22.67, -21.33, -20.0,
		-18.67, -17.33, -16.0, -14.67, -13.33, -12.0, -10.67, -9.3299999, -8.0,
		-6.6700001, -5.3299999, -4.0, -2.6700001, -1.33, 0.0, 1.33, 2.6700001, 4.0,
		5.3299999, 6.6700001, 8.0, 9.3299999, 10.67
};

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


tf::Point
spherical_to_cartesian(double hangle, double vangle, double range)
{
	double x, y, z;

	x = range * cos(vangle) * cos(hangle);
	y = range * cos(vangle) * sin(hangle);
	z = range * sin(vangle);

	return tf::Point(x, y, z);
}


tf::Point
move_to_camera_reference(tf::Point p3d_velodyne_reference, carmen_pose_3D_t velodyne_pose, carmen_pose_3D_t camera_pose)
{
    tf::Transform pose_velodyne_in_board(
            tf::Quaternion(velodyne_pose.orientation.yaw, velodyne_pose.orientation.pitch, velodyne_pose.orientation.roll),
            tf::Vector3(velodyne_pose.position.x, velodyne_pose.position.y, velodyne_pose.position.z));

    tf::Transform pose_camera_in_board(
            tf::Quaternion(camera_pose.orientation.yaw, camera_pose.orientation.pitch, camera_pose.orientation.roll),
            tf::Vector3(camera_pose.position.x, camera_pose.position.y, camera_pose.position.z));


	tf::Transform velodyne_frame_to_board_frame = pose_velodyne_in_board;
	tf::Transform board_frame_to_camera_frame = pose_camera_in_board.inverse();

	return board_frame_to_camera_frame * velodyne_frame_to_board_frame * p3d_velodyne_reference;
}

//TODO Mover RANGES da referencia da camera para referencia do carro

std::vector<carmen_velodyne_points_in_cam_t>
carmen_velodyne_camera_calibration_lasers_points_in_camera(carmen_velodyne_partial_scan_message *velodyne_message,
														   carmen_camera_parameters camera_parameters,
														   carmen_pose_3D_t velodyne_pose, carmen_pose_3D_t camera_pose,
														   int image_width, int image_height)
		{
	std::vector<carmen_velodyne_points_in_cam_t> laser_points_in_camera;

    double fx_meters = camera_parameters.fx_factor * image_width * camera_parameters.pixel_size;
    double fy_meters = camera_parameters.fy_factor * image_height * camera_parameters.pixel_size;

    double cu = camera_parameters.cu_factor * (double) image_width;
    double cv = camera_parameters.cv_factor * (double) image_height;

	for (int i = 0; i < 32; i++)
	{
		double v_angle = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[i]));

		for (int j = 0; j < velodyne_message->number_of_32_laser_shots; j++)
		{
			double range = (((double) velodyne_message->partial_scan[j].distance[i]) / 500.0);

            double h_angle = carmen_normalize_theta(carmen_degrees_to_radians(velodyne_message->partial_scan[j].angle));

			if (range <= MIN_RANGE)
				range = MAX_RANGE;

			if (range > MAX_RANGE)
				range = MAX_RANGE;

			if (range >= MAX_RANGE)
				continue;

			tf::Point p3d_velodyne_reference = spherical_to_cartesian(h_angle, v_angle, range);

			if (p3d_velodyne_reference.x() > 0)
			{
                tf::Point p3d_camera_reference = move_to_camera_reference(p3d_velodyne_reference,velodyne_pose,camera_pose);

                double px = (fx_meters * (p3d_camera_reference.y() / p3d_camera_reference.x()) / camera_parameters.pixel_size + cu);
                double py = (fy_meters * (-p3d_camera_reference.z() / p3d_camera_reference.x()) / camera_parameters.pixel_size + cv);

                int ipx = (int) px;
                int ipy = (int) py;

				if (ipx >= 0 && ipx <= image_width && ipy >= 0 && ipy <= image_height)
				{
					carmen_velodyne_points_in_cam_t velodyne_in_cam = {ipx, ipy, {h_angle, v_angle, range}};

					laser_points_in_camera.push_back(velodyne_in_cam);
				}

			}
		}
	}
	return laser_points_in_camera;
}


std::vector<carmen_velodyne_points_in_cam_t>
carmen_velodyne_camera_calibration_lasers_points_in_camera_with_obstacle(carmen_velodyne_partial_scan_message *velodyne_message,
                                                                         carmen_camera_parameters camera_parameters,
                                                                         carmen_pose_3D_t velodyne_pose, carmen_pose_3D_t camera_pose,
                                                                         int image_width, int image_height)
{
	std::vector<carmen_velodyne_points_in_cam_t> laser_points_in_camera;

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

				tf::Point p3d_camera_reference = move_to_camera_reference(p3d_velodyne_reference, velodyne_pose, camera_pose);

                double px = (fx_meters * (p3d_camera_reference.y() / p3d_camera_reference.x()) / camera_parameters.pixel_size + cu);
                double py = (fy_meters * (-p3d_camera_reference.z() / p3d_camera_reference.x()) / camera_parameters.pixel_size + cv);

				int ipx = (int) px;
				int ipy = (int) py;

				if (ipx >= 0 && ipx <= image_width && ipy >= 0 && ipy <= image_height && obstacle)
				{

					carmen_velodyne_points_in_cam_t velodyne_in_cam = {ipx, ipy, {h_angle,v_angle,range}};

					laser_points_in_camera.push_back(velodyne_in_cam);

				}

			}
		}
	}
	return laser_points_in_camera;
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


#define CAMERA_FOV 0.44 // In radians ~ 25 degrees TODO por no carmen.ini

std::vector<velodyne_camera_points>
velodyne_camera_calibration_remove_points_out_of_FOV_and_that_hit_ground(carmen_velodyne_partial_scan_message *velodyne_message,
                                                                         carmen_camera_parameters camera_parameters,
                                                                         carmen_pose_3D_t velodyne_pose, carmen_pose_3D_t camera_pose,
                                                                         int image_width, int image_height)
{
	std::vector<velodyne_camera_points> points;

	double fx_meters = camera_parameters.fx_factor * image_width * camera_parameters.pixel_size;
	double fy_meters = camera_parameters.fy_factor * image_height * camera_parameters.pixel_size;

	double cu = camera_parameters.cu_factor * (double) image_width;
	double cv = camera_parameters.cv_factor * (double) image_height;

	double horizontal_angle = 0.0, vertical_angle = 0.0, previous_vertical_angle = 0.0, range = 0.0, previous_range = 0.0;

	unsigned int point_x_on_img = 0, point_y_on_img = 0;

	for (int j = 0; j < velodyne_message->number_of_32_laser_shots; j++)
	{
		// TODO use the angle from measure velodyne_message->partial_scan[j].angle
		horizontal_angle = carmen_normalize_theta(carmen_degrees_to_radians(velodyne_message->partial_scan[j].angle));

		if (horizontal_angle < -CAMERA_FOV || horizontal_angle > CAMERA_FOV) // Discharge measures out of the camera field of view
			continue;

		previous_range = (((double) velodyne_message->partial_scan[j].distance[0]) / 500.0);
		previous_vertical_angle = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[0]));

		for (int i = 1; i < 32; i++)
		{
			range = (((double) velodyne_message->partial_scan[j].distance[i]) / 500.0);

			if (range >= MAX_RANGE)
				continue;

			if (range <= MIN_RANGE || range > MAX_RANGE)
				range = MAX_RANGE;

			vertical_angle = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[i]));

			tf::Point p3d_velodyne_reference = spherical_to_cartesian(horizontal_angle, vertical_angle, range);
			tf::Point p3d_velodyne_reference_1 = spherical_to_cartesian(horizontal_angle, previous_vertical_angle, previous_range);

			if (p3d_velodyne_reference.x() > 0)
			{
				// Jose's method check if a point is obstacle
				double delta_x = p3d_velodyne_reference.x() - p3d_velodyne_reference_1.x();
				double delta_z = (p3d_velodyne_reference.z()) - (p3d_velodyne_reference_1.z()); // TODO verificar calculo do z no mapper

				double line_angle = carmen_radians_to_degrees(fabs(atan2(delta_z, delta_x)));

				if (!(line_angle > MIN_ANGLE_OBSTACLE) && (line_angle < MAX_ANGLE_OBSTACLE))
					continue;

				tf::Point p3d_camera_reference = move_to_camera_reference(p3d_velodyne_reference, velodyne_pose, camera_pose);

				point_x_on_img = (unsigned int) (fx_meters * (p3d_camera_reference.y() / p3d_camera_reference.x()) / camera_parameters.pixel_size + cu);
                point_y_on_img = (unsigned int) (fy_meters * (-p3d_camera_reference.z() / p3d_camera_reference.x()) / camera_parameters.pixel_size + cv);

                // TODO Need spherical coordinates???
                velodyne_camera_points point = {point_x_on_img, point_y_on_img, {horizontal_angle, vertical_angle, range},
                		{p3d_camera_reference.x(), p3d_camera_reference.y(), p3d_camera_reference.z()}};

                points.push_back(point);
			}
			previous_range = range;
			previous_vertical_angle = vertical_angle;
		}
	}
	return points;
}
