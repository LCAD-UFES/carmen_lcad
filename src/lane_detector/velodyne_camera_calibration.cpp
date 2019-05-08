#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
//#include <carmen/carmen.h>
//#include <carmen/visual_tracker_interface.h>
#include "velodyne_camera_calibration.h"

const int MIN_ANGLE_OBSTACLE = 2;
const int MAX_ANGLE_OBSTACLE = 188;

#define MAX_RANGE 50.0
#define MIN_RANGE 0.5

#define SICK_MIN_RANGE 0.5
#define SICK_MAX_RANGE 170.0
using namespace tf;

#define CAMERA_FOV 0.576 // In radians ~ 33 degrees TODO por no carmen.ini


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


Point
spherical_to_cartesian(double hangle, double vangle, double range)
{
	double x, y, z;

	x = range * cos(vangle) * cos(hangle);
	y = range * cos(vangle) * sin(hangle);
	z = range * sin(vangle);

	return tf::Point(x, y, z);
}


Point
move_to_camera_reference(Point p3d_velodyne_reference, carmen_pose_3D_t velodyne_pose, carmen_pose_3D_t camera_pose)
{
    Transform pose_velodyne_in_board(
            Quaternion(velodyne_pose.orientation.yaw, velodyne_pose.orientation.pitch, velodyne_pose.orientation.roll),
            Vector3(velodyne_pose.position.x, velodyne_pose.position.y, velodyne_pose.position.z));

    Transform pose_camera_in_board(
            Quaternion(camera_pose.orientation.yaw, camera_pose.orientation.pitch, camera_pose.orientation.roll),
            Vector3(camera_pose.position.x, camera_pose.position.y, camera_pose.position.z));


	Transform velodyne_frame_to_board_frame = pose_velodyne_in_board;
	Transform board_frame_to_camera_frame = pose_camera_in_board.inverse();

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
			double h_angle = carmen_normalize_theta(carmen_degrees_to_radians(velodyne_message->partial_scan[j].angle));

			if (fabs(carmen_normalize_theta(h_angle - camera_pose.orientation.yaw)) >= CAMERA_FOV)
				continue;

			double range = (((double) velodyne_message->partial_scan[j].distance[i]) / 500.0);

			if (range <= MIN_RANGE || range >= MAX_RANGE)
				continue;

			Point p3d_velodyne_reference = spherical_to_cartesian(h_angle, v_angle, range);

			Point p3d_camera_reference = move_to_camera_reference(p3d_velodyne_reference,velodyne_pose,camera_pose);

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
	return laser_points_in_camera;
}


std::vector<carmen_velodyne_points_in_cam_t>
carmen_sick_camera_calibration_lasers_points_in_camera(carmen_laser_ldmrs_new_message *sick_message,
                                                           carmen_camera_parameters camera_parameters,
                                                           tf::Transformer *sick_transformation_tree,
                                                           int image_width, int image_height)
{
	std::vector<carmen_velodyne_points_in_cam_t> laser_points_in_camera;
	StampedTransform sick_to_camera_pose;
	sick_transformation_tree->lookupTransform("/camera", "/sick", tf::Time(0), sick_to_camera_pose);

	double fx_meters = camera_parameters.fx_factor * image_width * camera_parameters.pixel_size;
	double fy_meters = camera_parameters.fy_factor * image_height * camera_parameters.pixel_size;

	double cu = camera_parameters.cu_factor * (double) image_width;
	double cv = camera_parameters.cv_factor * (double) image_height;
	//cout<<laser_message->scan_points<<endl;
	for (int i = 0; i < sick_message->scan_points; i++)
	{
		double v_angle = sick_message->arraypoints[i].vertical_angle;
		//double v_angle = carmen_normalize_theta(carmen_degrees_to_radians(laser_message->arraypoints[i].vertical_angle));
		double range = sick_message->arraypoints[i].radial_distance;
		//printf("Range: %lf\n", range);
		double h_angle = sick_message->arraypoints[i].horizontal_angle;
		//double h_angle = carmen_normalize_theta(carmen_degrees_to_radians(laser_message->arraypoints[i].horizontal_angle));

		if (range <= SICK_MIN_RANGE || range >= SICK_MAX_RANGE)
						continue;

		Point p3d_velodyne_reference = spherical_to_cartesian(h_angle, v_angle, range);

		if (p3d_velodyne_reference.x() > 0)
		{
			//tf::Point p3d_camera_reference = move_to_camera_reference2(p3d_velodyne_reference,sick_pose,camera_pose);
			Point p3d_camera_reference = sick_to_camera_pose * p3d_velodyne_reference;

			double px = (fx_meters * (p3d_camera_reference.y() / p3d_camera_reference.x()) / camera_parameters.pixel_size + cu);
			double py = (fy_meters * (-p3d_camera_reference.z() / p3d_camera_reference.x()) / camera_parameters.pixel_size + cv);

			int ipx = image_width - (int) px - 1;
			//int ipx = (int) px;
			//int ipy = image_height - (int) py -1;
			int ipy = (int) py;

			if (ipx >= 0 && ipx <= image_width && ipy >= 0 && ipy <= image_height)
			{
				carmen_velodyne_points_in_cam_t velodyne_in_cam = {ipx, ipy, {h_angle, v_angle, range}};

				laser_points_in_camera.push_back(velodyne_in_cam);
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

			Point p3d_velodyne_reference = spherical_to_cartesian(h_angle, v_angle, range);
			Point p3d_velodyne_reference_1 = spherical_to_cartesian(h_angle, v_angle_1, range_1);

			if (p3d_velodyne_reference.x() > 0)
			{
				//metodo de jose
				double delta_x = p3d_velodyne_reference.x() - p3d_velodyne_reference_1.x();

				//obstacle_z = global_point_position_in_the_world.z - (robot_position.z - robot_wheel_radius);
				double delta_z = (p3d_velodyne_reference.z()) - (p3d_velodyne_reference_1.z());//verificar o z no mapper
				double line_angle = carmen_radians_to_degrees(fabs(atan2(delta_z, delta_x)));

				bool obstacle = (line_angle > MIN_ANGLE_OBSTACLE) && (line_angle < MAX_ANGLE_OBSTACLE);

				Point p3d_camera_reference = move_to_camera_reference(p3d_velodyne_reference, velodyne_pose, camera_pose);

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

				Point p3d_camera_reference = move_to_camera_reference(p3d_velodyne_reference,velodyne_pose,camera_pose);

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


vector<image_cartesian>
velodyne_camera_calibration_fuse_camera_lidar(carmen_velodyne_partial_scan_message *velodyne_message, carmen_camera_parameters camera_parameters,
                                              carmen_pose_3D_t velodyne_pose, carmen_pose_3D_t camera_pose, unsigned int image_width,
											  unsigned int image_height, unsigned int crop_x, unsigned int crop_y, unsigned int crop_width, unsigned int crop_height)
{
	std::vector<image_cartesian> points;
	double horizontal_angle = 0.0, vertical_angle = 0.0, previous_vertical_angle = 0.0, range = 0.0, previous_range = 0.0;
	unsigned int image_x = 0, image_y = 0;

	unsigned int max_x = crop_x + crop_width;
	unsigned int max_y = crop_y + crop_height;

	if (velodyne_message == NULL)
		return (points);

	double fx_meters = camera_parameters.fx_factor * camera_parameters.pixel_size * image_width;
	double fy_meters = camera_parameters.fy_factor * camera_parameters.pixel_size * image_height;

	double cu = camera_parameters.cu_factor * (double) image_width;
	double cv = camera_parameters.cv_factor * (double) image_height;

	for (int j = 0; j < velodyne_message->number_of_32_laser_shots; j++)
	{
		horizontal_angle = carmen_normalize_theta(carmen_degrees_to_radians(velodyne_message->partial_scan[j].angle));

		if (horizontal_angle < -CAMERA_FOV || horizontal_angle > CAMERA_FOV) // Discharge measures out of the camera field of view
			continue;

		previous_range = (((double) velodyne_message->partial_scan[j].distance[0]) / 500.0);
		previous_vertical_angle = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[0]));

		for (int i = 1; i < 32; i++)
		{
			range = (((double) velodyne_message->partial_scan[j].distance[i]) / 500.0);

			if (range <= MIN_RANGE || range >= MAX_RANGE)
				continue;

			vertical_angle = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[i]));

			Point p3d_velodyne_reference = spherical_to_cartesian(horizontal_angle, vertical_angle, range);
			Point p3d_velodyne_reference_1 = spherical_to_cartesian(horizontal_angle, previous_vertical_angle, previous_range);

			if (p3d_velodyne_reference.x() > 0)
			{
				// Jose's method check if a point is obstacle
				double delta_x = p3d_velodyne_reference.x() - p3d_velodyne_reference_1.x();
				double delta_z = p3d_velodyne_reference.z() - p3d_velodyne_reference_1.z(); // TODO verificar calculo do z no mapper

				double line_angle = carmen_radians_to_degrees(fabs(atan2(delta_z, delta_x)));

				if (!(line_angle > MIN_ANGLE_OBSTACLE) && (line_angle < MAX_ANGLE_OBSTACLE))
					continue;

				Point p3d_camera_reference = move_to_camera_reference(p3d_velodyne_reference, velodyne_pose, camera_pose);

				image_x = (unsigned int) (fx_meters * (p3d_camera_reference.y() / p3d_camera_reference.x()) / camera_parameters.pixel_size + cu);
                image_y = (unsigned int) (fy_meters * (-p3d_camera_reference.z() / p3d_camera_reference.x()) / camera_parameters.pixel_size + cv);

                if (image_x >= crop_x && image_x <= max_x && image_y >= crop_y && image_y <= max_y)
                {
                	image_cartesian point;
                	point.shot_number = j;
                	point.ray_number  = i;
                	point.image_x     = image_x - crop_x;
                	point.image_y     = image_y - crop_y;
                	point.cartesian_x = p3d_velodyne_reference.x();
                	point.cartesian_y = -p3d_velodyne_reference.y();             // Must be inverted because Velodyne angle is reversed with CARMEN angles
                	point.cartesian_z = p3d_velodyne_reference.z();

                	points.push_back(point);
                }
			}
			previous_range = range;
			previous_vertical_angle = vertical_angle;
		}
	}
	return points;
}


vector<image_cartesian>
sick_camera_calibration_fuse_camera_lidar(carmen_laser_ldmrs_new_message *sick_message, carmen_camera_parameters camera_parameters,
                                                                         tf::Transformer *sick_transformation_tree,
																		 unsigned int image_width, unsigned int image_height, unsigned int crop_x,
																		 unsigned int crop_y, unsigned int crop_width, unsigned int crop_height)
{
	std::vector<image_cartesian> points;
	double horizontal_angle = 0.0, vertical_angle = 0.0, range = 0.0;
	unsigned int image_x = 0, image_y = 0;

	unsigned int max_x = crop_x + crop_width;
	unsigned int max_y = crop_y + crop_height;

	if (sick_message == NULL)
		return (points);

	double fx_meters = camera_parameters.fx_factor * camera_parameters.pixel_size * image_width;
	double fy_meters = camera_parameters.fy_factor * camera_parameters.pixel_size * image_height;

	double cu = camera_parameters.cu_factor * (double) image_width;
	double cv = camera_parameters.cv_factor * (double) image_height;


	tf::StampedTransform sick_to_camera_pose;
	sick_transformation_tree->lookupTransform("/camera", "/sick", tf::Time(0), sick_to_camera_pose);

	for (int i = 0; i < sick_message->scan_points; i++)
	{
		horizontal_angle = sick_message->arraypoints[i].horizontal_angle;

//		if (horizontal_angle < -CAMERA_FOV || horizontal_angle > CAMERA_FOV)// Discharge measures out of the camera field of view
//			continue;

		range = sick_message->arraypoints[i].radial_distance;

		if (range <= SICK_MIN_RANGE || range >= SICK_MAX_RANGE)
			continue;

		vertical_angle = sick_message->arraypoints[i].vertical_angle;

		Point p3d_velodyne_reference = spherical_to_cartesian(horizontal_angle, vertical_angle, range);
		if (p3d_velodyne_reference.x() > 0)
		{
			Point p3d_camera_reference = sick_to_camera_pose * p3d_velodyne_reference;

			image_x = (unsigned int) (fx_meters * (p3d_camera_reference.y() / p3d_camera_reference.x()) / camera_parameters.pixel_size + cu);
			image_y = (unsigned int) (fy_meters * (-p3d_camera_reference.z() / p3d_camera_reference.x()) / camera_parameters.pixel_size + cv);

			if (image_x >= crop_x && image_x <= max_x && image_y >= crop_y && image_y <= max_y)
			{
				image_cartesian point;
				point.shot_number = i;
				point.ray_number  = i;
				point.image_x     = crop_width - image_x + crop_x;
				point.image_y     = image_y - crop_y;
				point.cartesian_x = p3d_velodyne_reference.x();
				point.cartesian_y = p3d_velodyne_reference.y();             // Must be inverted because Velodyne angle is reversed with CARMEN angles
				point.cartesian_z = p3d_velodyne_reference.z();

				points.push_back(point);
			}
		}
	}
	return points;
}


void
initialize_transformations(carmen_pose_3D_t board_pose, carmen_pose_3D_t camera_pose, tf::Transformer *transformer)
{
	//printf("%lf %lf %lf %lf %lf %lf \n", board_pose.position.x, board_pose.position.y, board_pose.position.z, board_pose.orientation.roll, board_pose.orientation.pitch, board_pose.orientation.yaw);

	Transform board_to_camera_pose;
	Transform car_to_board_pose;

	Time::init();

	// board pose with respect to the car
	car_to_board_pose.setOrigin(tf::Vector3(board_pose.position.x, board_pose.position.y, board_pose.position.z));
	car_to_board_pose.setRotation(tf::Quaternion(board_pose.orientation.yaw, board_pose.orientation.pitch, board_pose.orientation.roll)); 				// yaw, pitch, roll
	StampedTransform car_to_board_transform(car_to_board_pose, tf::Time(0), "/car", "/board");
	transformer->setTransform(car_to_board_transform, "car_to_board_transform");

	// camera pose with respect to the board
	board_to_camera_pose.setOrigin(tf::Vector3(camera_pose.position.x, camera_pose.position.y, camera_pose.position.z));
	board_to_camera_pose.setRotation(tf::Quaternion(camera_pose.orientation.yaw + carmen_degrees_to_radians(2.5), camera_pose.orientation.pitch, camera_pose.orientation.roll)); 				// yaw, pitch, roll
	//board_to_camera_pose.setRotation(tf::Quaternion(camera_pose.orientation.yaw, camera_pose.orientation.pitch, camera_pose.orientation.roll)); 				// yaw, pitch, roll
	StampedTransform board_to_camera_transform(board_to_camera_pose, tf::Time(0), "/board", "/camera");
	transformer->setTransform(board_to_camera_transform, "board_to_camera_transform");
}


void
initialize_sick_transformations(carmen_pose_3D_t board_pose, carmen_pose_3D_t camera_pose, carmen_pose_3D_t bullbar_pose,carmen_pose_3D_t sick_pose, tf::Transformer *transformer)
{
	Transform board_to_camera_pose;
	Transform car_to_board_pose;
	Transform bull_to_car_pose;

	Time::init();

	// board pose with respect to the car
	car_to_board_pose.setOrigin(tf::Vector3(board_pose.position.x, board_pose.position.y, board_pose.position.z));
	car_to_board_pose.setRotation(tf::Quaternion(board_pose.orientation.yaw, board_pose.orientation.pitch, board_pose.orientation.roll)); 				// yaw, pitch, roll
	StampedTransform car_to_board_transform(car_to_board_pose, tf::Time(0), "/car", "/board");
	transformer->setTransform(car_to_board_transform, "car_to_board_transform");

	// camera pose with respect to the board
	board_to_camera_pose.setOrigin(tf::Vector3(camera_pose.position.x, camera_pose.position.y, camera_pose.position.z));
	board_to_camera_pose.setRotation(tf::Quaternion(camera_pose.orientation.yaw + carmen_degrees_to_radians(2.5), camera_pose.orientation.pitch, camera_pose.orientation.roll)); 				// yaw, pitch, roll
	//board_to_camera_pose.setRotation(tf::Quaternion(camera_pose.orientation.yaw, camera_pose.orientation.pitch, camera_pose.orientation.roll)); 				// yaw, pitch, roll
	StampedTransform board_to_camera_transform(board_to_camera_pose, tf::Time(0), "/board", "/camera");
	transformer->setTransform(board_to_camera_transform, "board_to_camera_transform");

	// bull pose with respect to the car
	bull_to_car_pose.setOrigin(tf::Vector3(bullbar_pose.position.x, bullbar_pose.position.y, bullbar_pose.position.z));
	bull_to_car_pose.setRotation(tf::Quaternion(bullbar_pose.orientation.yaw, bullbar_pose.orientation.pitch, bullbar_pose.orientation.roll)); // yaw, pitch, roll
	StampedTransform bull_to_car_transform(bull_to_car_pose, tf::Time(0), "/car", "/bull");
	transformer->setTransform(bull_to_car_transform, "bull_to_car_transform");

	// sick pose with respect to the bull
	Transform sick_to_bull_pose;
	sick_to_bull_pose.setOrigin(tf::Vector3(sick_pose.position.x, sick_pose.position.y, sick_pose.position.z));
	sick_to_bull_pose.setRotation(tf::Quaternion(sick_pose.orientation.yaw, sick_pose.orientation.pitch, sick_pose.orientation.roll));
	StampedTransform sick_to_bull_transform(sick_to_bull_pose, tf::Time(0), "/bull", "/sick");
	transformer->setTransform(sick_to_bull_transform, "sick_to_bull_transform");

}

StampedTransform
get_world_to_camera_transformation (tf::Transformer *transformer, carmen_pose_3D_t pose)
{
	Transform world_to_car_pose;
	StampedTransform world_to_camera_pose;
	world_to_car_pose.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
	world_to_car_pose.setRotation(tf::Quaternion(pose.orientation.yaw, pose.orientation.pitch, pose.orientation.roll)); // yaw, pitch, roll

	StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
	transformer->setTransform(world_to_car_transform, "world_to_car_transform");

	transformer->lookupTransform("/world", "/camera", tf::Time(0), world_to_camera_pose);

	return (world_to_camera_pose);
}


StampedTransform
get_world_to_camera_transformer(tf::Transformer *transformer, double x, double y, double z, double roll, double pitch, double yaw)
{
	//printf("%lf %lf %lf %lf %lf %lf \n", x, y, z, roll, pitch, yaw);

	Transform world_to_car_pose;
	StampedTransform world_to_camera_pose;
	world_to_car_pose.setOrigin(tf::Vector3(x, y, z));
	world_to_car_pose.setRotation(tf::Quaternion(yaw, pitch, roll));
	StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
	transformer->setTransform(world_to_car_transform, "world_to_car_transform");

	transformer->lookupTransform("/world", "/camera", tf::Time(0), world_to_camera_pose);

	return (world_to_camera_pose);
}


carmen_position_t
convert_rddf_pose_to_point_in_image(double x, double y, double z, tf::StampedTransform world_to_camera_pose,
									carmen_camera_parameters camera_parameters, int image_width, int image_height)
{
	Point point (x, y, z);
	Point point_transformed;

	carmen_position_t point_in_image;

	// fx and fy are the focal lengths
	double fx_meters = camera_parameters.fx_factor * image_width * camera_parameters.pixel_size;
	double fy_meters = camera_parameters.fy_factor * image_height * camera_parameters.pixel_size;
	//cu, cv represent the camera principal point
	double cu = camera_parameters.cu_factor * (double) image_width;
	double cv = camera_parameters.cv_factor * (double) image_height;
	//printf("Focal Lenght: %lf X %lf\n", fx_meters, fy_meters);
	//printf("Principal Point: %lf X %lf\n", cu, cv);

	point_transformed = world_to_camera_pose.inverse() * point;

	//printf("Pose in camera reference: %lf X %lf X %lf\n", rddf_pose_transformed[0], rddf_pose_transformed[1], rddf_pose_transformed[2]);

	point_in_image.x = (unsigned int) (fx_meters * (point_transformed[1] / point_transformed[0]) / camera_parameters.pixel_size + cu);
	point_in_image.y = (unsigned int) (fy_meters * (-(point_transformed[2]+0.28) / point_transformed[0]) / camera_parameters.pixel_size + cv);
	point_in_image.x = image_width - point_in_image.x;

	//printf("Pose in image: %lf X %lf\n", point.x, point.y);

	return (point_in_image);
}


//carmen_position_t
//world_point_to_image()

