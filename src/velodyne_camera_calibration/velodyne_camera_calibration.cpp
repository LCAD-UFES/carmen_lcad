
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/visual_tracker_interface.h>
#include <string>
#include "velodyne_camera_calibration.h"

#include <tf.h>

using namespace cv;
using namespace tf;

const int RANGE_WINDOW_ZOOM = 5;
const int WINDOW_WIDTH = 1366;

const int MIN_ANGLE_OBSTACLE = 2;
const int MAX_ANGLE_OBSTACLE = 188;

typedef struct {
	carmen_velodyne_points_in_cam_t velodyne_points_in_cam;
	double angle;
} carmen_velodyne_points_in_cam_with_angle_t, *carmen_velodyne_points_in_cam_with_angle_p;


// CAM POSE: 0.245000 0.283000 0.075000 0.017453 0.026300 0.000000 FX: 0.753883
// CAM POSE: 0.245000 0.283000 0.085000 0.017453 0.026300 -0.017453 FX: 0.753883

//CAM POSE: 0.245000 -0.207000 0.085000 0.017453 0.026300 -0.017453 FX: 0.753883

//double camera_x = 0.245;
//double camera_y = -0.207000; //0.283; //-0.287;
//double camera_z = 0.085;
//double camera_roll = 0.017453; //0;
//double camera_pitch = 0.0263;
//double camera_yaw = -0.017453; // 0;

double camera_x = 0.130; //0.25;
double camera_y = -0.131; //0.149; //-0.287;
double camera_z = 0.204; //0.214;
double camera_roll = -0.017453; //0;
double camera_pitch = 0.034907;
double camera_yaw = -0.017453; // 0;


double fx = 0.753883;
double fy = 1.00518;

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
move_to_camera_reference(tf::Point p3d_velodyne_reference)
{
	tf::Transform pose_velodyne_in_board(Quaternion(-0.01, -0.0227, 0), Vector3(0.145, 0, 0.48));
	tf::Transform pose_camera_in_board(Quaternion(camera_yaw, camera_pitch, camera_roll),
			Vector3(camera_x, camera_y, camera_z));

//	tf::Transform pose_velodyne_in_board(Quaternion(-0.01, -0.0227, 0), Vector3(0.145, 0, 0.48));
//	tf::Transform pose_camera_in_board(Quaternion(0, 0, 0),
//			Vector3(0.25, 0.149 + 0.24004, 0.214));

	tf::Transform velodyne_frame_to_board_frame = pose_velodyne_in_board;
	tf::Transform board_frame_to_camera_frame = pose_camera_in_board.inverse();

	return board_frame_to_camera_frame * velodyne_frame_to_board_frame * p3d_velodyne_reference;
}

//TODO Mover RANGES da referencia da camera para referencia do carro

std::vector<carmen_velodyne_points_in_cam_t>
carmen_velodyne_camera_calibration_lasers_points_in_camera(carmen_velodyne_partial_scan_message *velodyne_message,
		int image_width, int image_height)
		{
	std::vector<carmen_velodyne_points_in_cam_t> laser_points_in_camera;

	for (int i = 0; i < 32; i++)
	{
		double v = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[i]));

		for (int j = 0; j < velodyne_message->number_of_32_laser_shots; j++)
		{
			double range = (((double) velodyne_message->partial_scan[j].distance[i]) / 500.0);

			double hangle = velodyne_message->partial_scan[j].angle;
			double hrad = carmen_degrees_to_radians(hangle);
			double hblablabla = carmen_normalize_theta(hrad);

			//float r = 0;
			//float b = 0;

			const float MAX_RANGE = 50.0;
			const float MIN_RANGE = 0.5;

			if (range <= MIN_RANGE)
				range = MAX_RANGE;

			if (range > MAX_RANGE)
				range = MAX_RANGE;

			if (range >= MAX_RANGE)
				continue;

			//r = range / MAX_RANGE;
			//r *= 255;
			//r = 255 - r;

			tf::Point p3d_velodyne_reference = spherical_to_cartesian(hblablabla, v, range);

			if (p3d_velodyne_reference.x() > 0)
			{
				tf::Point p3d_camera_reference = move_to_camera_reference(p3d_velodyne_reference);

				const int XB3_MAX_PIXEL_WIDTH = image_width;//1280;//TODO MESMO DA IMAGEM OU PRECISA SER UM VALOR FIXO?
				const int XB3_MAX_PIXEL_HEIGHT = image_height; //960;//TODO MESMO DA IMAGEM OU PRECISA SER UM VALOR FIXO?
				const double XB3_PIXEL_SIZE = 0.00000375f;//pixel size (in meters)

				double f_meters = fx * XB3_MAX_PIXEL_WIDTH * XB3_PIXEL_SIZE;

				double cu = 0.500662 * (double) image_width;
				double cv = 0.506046 * (double) image_height;

				double px = (f_meters * (p3d_camera_reference.y() / p3d_camera_reference.x()) / XB3_PIXEL_SIZE + cu);
				double py = (f_meters * (-p3d_camera_reference.z() / p3d_camera_reference.x()) / XB3_PIXEL_SIZE + cv);

				int ipx = (int) px;
				int ipy = (int) py;

				if (px >= 0 && px <= image_width && py >= 0 && py <= image_height)
				{
					//if (px < 10 || px >= (bumblebee_message->width - 10))
						//b = 254;

					//if (r > 5)
					{
						carmen_velodyne_points_in_cam_t laser_px_points; //= {ipx, ipy, laser_polar};

						laser_px_points.ipx = ipx;
						laser_px_points.ipy = ipy;
						laser_px_points.laser_polar.horizontal_angle = hblablabla;
						laser_px_points.laser_polar.vertical_angle = v;
						laser_px_points.laser_polar.length = range;

						laser_points_in_camera.push_back(laser_px_points);

						//if (laser_points_in_camera.size() < 5) printf("H: %lf\n", hblablabla);
					}
				}

			}
		}
	}
	return laser_points_in_camera;
}

std::vector<carmen_velodyne_points_in_cam_t>
carmen_velodyne_camera_calibration_lasers_points_in_camera_with_obstacle(carmen_velodyne_partial_scan_message *velodyne_message,
		int image_width, int image_height)
{

	double robot_wheel_radius =  0.28;//parametro do carmen-ford-escape.ini
	std::vector<carmen_velodyne_points_in_cam_t> laser_points_in_camera;

	for (int j = 0; j < velodyne_message->number_of_32_laser_shots; j++)
	{
		double hangle = velodyne_message->partial_scan[j].angle;
		double hrad = carmen_degrees_to_radians(hangle);
		double htetha = carmen_normalize_theta(hrad);

		//		double hangle_1 = velodyne_message->partial_scan[j-1].angle;
		//		double hrad_1 = carmen_degrees_to_radians(hangle);
		//		double htetha_1 = carmen_normalize_theta(hrad);

		for (int i = 1; i < 32; i++)
		{
			double v = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[i]));
			double v_1 = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[i-1]));

			double range = (((double) velodyne_message->partial_scan[j].distance[i]) / 500.0);
			double range_1 = (((double) velodyne_message->partial_scan[j].distance[i-1]) / 500.0);

			const float MAX_RANGE = 50.0;
			const float MIN_RANGE = 0.5;

			if (range <= MIN_RANGE)
				range = MAX_RANGE;

			if (range > MAX_RANGE)
				range = MAX_RANGE;

			if (range >= MAX_RANGE)
				continue;

			tf::Point p3d_velodyne_reference = spherical_to_cartesian(htetha, v, range);
			tf::Point p3d_velodyne_reference_1 = spherical_to_cartesian(htetha, v_1, range_1);

			if (p3d_velodyne_reference.x() > 0)
			{
				carmen_velodyne_points_in_cam_t laser_px_points;

				//metodo de jose
				double delta_x = p3d_velodyne_reference.x() - p3d_velodyne_reference_1.x();

				//obstacle_z = global_point_position_in_the_world.z - (robot_position.z - robot_wheel_radius);
				double delta_z = (p3d_velodyne_reference.z()) - (p3d_velodyne_reference_1.z());//verificar o z no mapper
				double line_angle = carmen_radians_to_degrees(fabs(atan2(delta_z, delta_x)));

				//printf("%lf %lf %lf\n", delta_x, delta_z, line_angle);

				bool obstacle;
				if((line_angle > MIN_ANGLE_OBSTACLE) && (line_angle < MAX_ANGLE_OBSTACLE))
					obstacle = true;
				else
					obstacle = false;

				tf::Point p3d_camera_reference = move_to_camera_reference(p3d_velodyne_reference);

				const int XB3_MAX_PIXEL_WIDTH = image_width;//1280;//TODO MESMO DA IMAGEM OU PRECISA SER UM VALOR FIXO?
				const int XB3_MAX_PIXEL_HEIGHT = image_height; //960;//TODO MESMO DA IMAGEM OU PRECISA SER UM VALOR FIXO?
				const double XB3_PIXEL_SIZE = 0.00000375f;//pixel size (in meters)

				double f_meters = fx * XB3_MAX_PIXEL_WIDTH * XB3_PIXEL_SIZE;

				double cu = 0.500662 * (double) image_width;
				double cv = 0.506046 * (double) image_height;

				double px = (f_meters * (p3d_camera_reference.y() / p3d_camera_reference.x()) / XB3_PIXEL_SIZE + cu);
				double py = (f_meters * (-p3d_camera_reference.z() / p3d_camera_reference.x()) / XB3_PIXEL_SIZE + cv);

				int ipx = (int) px;
				int ipy = (int) py;

				if (px >= 0 && px <= image_width && py >= 0 && py <= image_height && obstacle)
				{
					laser_px_points.ipx = ipx;
					laser_px_points.ipy = ipy;
					laser_px_points.laser_polar.horizontal_angle = htetha;
					laser_px_points.laser_polar.vertical_angle = v;
					laser_px_points.laser_polar.length = range;

					laser_points_in_camera.push_back(laser_px_points);

					//if (laser_points_in_camera.size() < 5) printf("H: %lf\n", hblablabla);
				}

			}
		}
	}
	return laser_points_in_camera;
}

std::vector<carmen_velodyne_points_in_cam_with_obstacle_t>
carmen_velodyne_camera_calibration_lasers_points_in_camera_with_obstacle_and_display(carmen_velodyne_partial_scan_message *velodyne_message,
		int image_width, int image_height)
{

	double robot_wheel_radius =  0.28;//parametro do carmen-ford-escape.ini
	std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> laser_points_in_camera;

	for (int j = 0; j < velodyne_message->number_of_32_laser_shots; j++)
	{
		double hangle = velodyne_message->partial_scan[j].angle;
		double hrad = carmen_degrees_to_radians(hangle);
		double htetha = carmen_normalize_theta(hrad);

		//		double hangle_1 = velodyne_message->partial_scan[j-1].angle;
		//		double hrad_1 = carmen_degrees_to_radians(hangle);
		//		double htetha_1 = carmen_normalize_theta(hrad);

		for (int i = 1; i < 32; i++)
		{
			double v = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[i]));
			double v_1 = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[i-1]));

			double range = (((double) velodyne_message->partial_scan[j].distance[i]) / 500.0);
			double range_1 = (((double) velodyne_message->partial_scan[j].distance[i-1]) / 500.0);

			const float MAX_RANGE = 50.0;
			const float MIN_RANGE = 0.5;

			if (range <= MIN_RANGE)
				range = MAX_RANGE;

			if (range > MAX_RANGE)
				range = MAX_RANGE;

			if (range >= MAX_RANGE)
				continue;

			tf::Point p3d_velodyne_reference = spherical_to_cartesian(htetha, v, range);
			tf::Point p3d_velodyne_reference_1 = spherical_to_cartesian(htetha, v_1, range_1);

			if (p3d_velodyne_reference.x() > 0)
			{
				carmen_velodyne_points_in_cam_with_obstacle_t laser_px_points;
				bool obstacle;

				//metodo de jose
				double delta_x = p3d_velodyne_reference.x() - p3d_velodyne_reference_1.x();

				//obstacle_z = global_point_position_in_the_world.z - (robot_position.z - robot_wheel_radius);
				double delta_z = (p3d_velodyne_reference.z()) - (p3d_velodyne_reference_1.z());//verificar o z no mapper
				double line_angle = carmen_radians_to_degrees(fabs(atan2(delta_z, delta_x)));

				//printf("%lf %lf %lf\n", delta_x, delta_z, line_angle);

				if((line_angle > MIN_ANGLE_OBSTACLE) && (line_angle < MAX_ANGLE_OBSTACLE))
					obstacle = true;
				else
					obstacle = false;

				tf::Point p3d_camera_reference = move_to_camera_reference(p3d_velodyne_reference);

				const int XB3_MAX_PIXEL_WIDTH = image_width;//1280;//TODO MESMO DA IMAGEM OU PRECISA SER UM VALOR FIXO?
				const int XB3_MAX_PIXEL_HEIGHT = image_height; //960;//TODO MESMO DA IMAGEM OU PRECISA SER UM VALOR FIXO?
				const double XB3_PIXEL_SIZE = 0.00000375f;//pixel size (in meters)

				double f_meters = fx * XB3_MAX_PIXEL_WIDTH * XB3_PIXEL_SIZE;

				double cu = 0.500662 * (double) image_width;
				double cv = 0.506046 * (double) image_height;

				double px = (f_meters * (p3d_camera_reference.y() / p3d_camera_reference.x()) / XB3_PIXEL_SIZE + cu);
				double py = (f_meters * (-p3d_camera_reference.z() / p3d_camera_reference.x()) / XB3_PIXEL_SIZE + cv);

				int ipx = (int) px;
				int ipy = (int) py;

				if (px >= 0 && px <= image_width && py >= 0 && py <= image_height)
				{
					laser_px_points.velodyne_points_in_cam.ipx = ipx;
					laser_px_points.velodyne_points_in_cam.ipy = ipy;
					laser_px_points.velodyne_points_in_cam.laser_polar.horizontal_angle = htetha;
					laser_px_points.velodyne_points_in_cam.laser_polar.vertical_angle = v;
					laser_px_points.velodyne_points_in_cam.laser_polar.length = range;
					laser_px_points.hit_in_obstacle = obstacle;

					laser_points_in_camera.push_back(laser_px_points);

					//if (laser_points_in_camera.size() < 5) printf("H: %lf\n", hblablabla);
				}

			}
		}
	}
	return laser_points_in_camera;
}


std::vector<carmen_velodyne_points_in_cam_with_angle_t>
carmen_velodyne_camera_calibration_lasers_points_in_camera_with_angles(carmen_velodyne_partial_scan_message *velodyne_message,
		carmen_bumblebee_basic_stereoimage_message *bumblebee_message)
		{

	std::vector<carmen_velodyne_points_in_cam_with_angle_t> laser_points_in_camera;

	for (int i = 0; i < 32; i++)
	{
		double v = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[i]));

		for (int j = 1; j < velodyne_message->number_of_32_laser_shots; j++)
		{
			double range = (((double) velodyne_message->partial_scan[j].distance[i]) / 500.0);

			double hangle = velodyne_message->partial_scan[j].angle;
			double hrad = carmen_degrees_to_radians(hangle);
			double hblablabla = carmen_normalize_theta(hrad);

			double hangle_1 = velodyne_message->partial_scan[j-1].angle;
			double hrad_1 = carmen_degrees_to_radians(hangle);
			double hblablabla_1 = carmen_normalize_theta(hrad);
			//float r = 0;
			//float b = 0;

			const float MAX_RANGE = 50.0;
			const float MIN_RANGE = 0.5;

			if (range <= MIN_RANGE)
				range = MAX_RANGE;

			if (range > MAX_RANGE)
				range = MAX_RANGE;

			if (range >= MAX_RANGE)
				continue;

			//r = range / MAX_RANGE;
			//r *= 255;
			//r = 255 - r;

			tf::Point p3d_velodyne_reference = spherical_to_cartesian(hblablabla, v, range);

			if (p3d_velodyne_reference.x() > 0)
			{
				tf::Point p3d_camera_reference = move_to_camera_reference(p3d_velodyne_reference);

				const int XB3_MAX_PIXEL_WIDTH = bumblebee_message->width;//1280;//TODO MESMO DA IMAGEM OU PRECISA SER UM VALOR FIXO?
				const int XB3_MAX_PIXEL_HEIGHT = bumblebee_message->height; //960;//TODO MESMO DA IMAGEM OU PRECISA SER UM VALOR FIXO?
				const double XB3_PIXEL_SIZE = 0.00000375f;//pixel size (in meters)

				double f_meters = fx * XB3_MAX_PIXEL_WIDTH * XB3_PIXEL_SIZE;

				double cu = 0.500662 * (double) bumblebee_message->width;
				double cv = 0.506046 * (double) bumblebee_message->height;

				double px = (f_meters * (p3d_camera_reference.y() / p3d_camera_reference.x()) / XB3_PIXEL_SIZE + cu);
				double py = (f_meters * (-p3d_camera_reference.z() / p3d_camera_reference.x()) / XB3_PIXEL_SIZE + cv);

				int ipx = (int) px;
				int ipy = (int) py;

				if (px >= 0 && px <= bumblebee_message->width && py >= 0 && py <= bumblebee_message->height)
				{
					//if (px < 10 || px >= (bumblebee_message->width - 10))
						//b = 254;

					//if (r > 5)
					{
						carmen_velodyne_points_in_cam_with_angle_t laser_px_points; //= {ipx, ipy, laser_polar};

						laser_px_points.velodyne_points_in_cam.ipx = ipx;
						laser_px_points.velodyne_points_in_cam.ipy = ipy;
						laser_px_points.velodyne_points_in_cam.laser_polar.horizontal_angle = hblablabla;
						laser_px_points.velodyne_points_in_cam.laser_polar.vertical_angle = v;
						laser_px_points.velodyne_points_in_cam.laser_polar.length = range;

						laser_points_in_camera.push_back(laser_px_points);

						//if (laser_points_in_camera.size() < 5) printf("H: %lf\n", hblablabla);
					}
				}

			}
		}
	}
	return laser_points_in_camera;
}
/*
std::vector<carmen_velodyne_points_in_cam_t>
carmen_velodyne_camera_calibration_lasers_points_bounding_box(carmen_velodyne_partial_scan_message *velodyne_message,
		carmen_bumblebee_basic_stereoimage_message *bumblebee_message, double *confidence, bounding_box *box)
		{

	//	carmen_visual_tracker_output_message box_message = {visual_tracker_output_message->rect, visual_tracker_output_message->confidence,
	//			visual_tracker_output_message->timestamp ,visual_tracker_output_message->host};
	vector<carmen_velodyne_points_in_cam_t> laser_points_in_camera;
	carmen_sphere_coord_t laser_angles;
	carmen_velodyne_points_in_cam_t laser_ipxy_points;

	carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(velodyne_message);

	for (int i = 0; i < 32; i++)
	{
		double v = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[i]));

		for (int j = 0; j < velodyne_message->number_of_32_laser_shots; j++)
		{
			double range = (((double) velodyne_message->partial_scan[j].distance[i]) / 500.0);

			double h = carmen_normalize_theta(carmen_degrees_to_radians(velodyne_message->partial_scan[j].angle));

			float r = 0;
			float b = 0;

			const float MAX_RANGE = 30.0;
			const float MIN_RANGE = 0.5;

			if (range <= MIN_RANGE)
				range = MAX_RANGE;

			if (range > MAX_RANGE)
				range = MAX_RANGE;

			r = range / MAX_RANGE;
			r *= 255;
			r = 255 - r;

			tf::Point p3d_velodyne_reference = spherical_to_cartesian(h, v, range);

			if (p3d_velodyne_reference.x() > 0)
			{

				tf::Point p3d_camera_reference = move_to_camera_reference(p3d_velodyne_reference);

				const int XB3_MAX_PIXEL_WIDTH = bumblebee_message->width;//1280;//TODO MESMO DA IMAGEM OU PRECISA SER UM VALOR FIXO?
				const int XB3_MAX_PIXEL_HEIGHT = bumblebee_message->height; //960;//TODO MESMO DA IMAGEM OU PRECISA SER UM VALOR FIXO?
				const double XB3_PIXEL_SIZE = 0.00000375f;//pixel size (in meters)

				double f_meters = fx * XB3_MAX_PIXEL_WIDTH * XB3_PIXEL_SIZE;

				double cu = 0.500662 * (double) bumblebee_message->width;
				double cv = 0.506046 * (double) bumblebee_message->height;

				double px = (f_meters * (p3d_camera_reference.y() / p3d_camera_reference.x()) / XB3_PIXEL_SIZE + cu);
				double py = (f_meters * (-p3d_camera_reference.z() / p3d_camera_reference.x()) / XB3_PIXEL_SIZE + cv);

				int ipx = (int) px;
				int ipy = (int) py;

				if (px >= 0 && px <= bumblebee_message->width && py >= 0 && py <= bumblebee_message->height)
				{
					if (px < 10 || px >= (bumblebee_message->width - 10))
						b = 254;


//					if((r > 5) && (ipx > box->x) && ( ipx < (box->x + box->width)) && (ipy > box->y) &&
//							( ipy < (box->y + box->height)))
//					{
						//	a < x0 < a+c and b < y0 < b + d
						laser_angles = {h, v, range};
						laser_ipxy_points = {ipx, ipy, laser_angles};
						laser_points_in_camera.push_back(laser_ipxy_points);
//					}


				}

			}
		}
	}
	return laser_points_in_camera;
		}
*/


