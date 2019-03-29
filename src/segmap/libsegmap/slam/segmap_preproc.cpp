
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv/cv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <carmen/segmap_pose2d.h>
#include <carmen/carmen_lidar_reader.h>
#include <carmen/carmen_image_reader.h>
#include <carmen/carmen_semantic_segmentation_reader.h>
#include <carmen/synchronized_data_package.h>
#include <carmen/segmap_conversions.h>

#include "segmap_preproc.h"

using namespace cv;
using namespace pcl;
using namespace std;
using namespace Eigen;


SensorPreproc::SensorPreproc(CarmenLidarLoader *vloader,
														 CarmenImageLoader *iloader,
														 SemanticSegmentationLoader *sloader,
														 Matrix<double, 4, 4> vel2cam,
														 Matrix<double, 4, 4> vel2car,
														 Matrix<double, 3, 4> projection,
														 Matrix<double, 4, 4> xsens2car,
														 int use_xsens,
														 Pose2d offset,
														 IntensityMode imode)
{
	_vloader = vloader;
	_iloader = iloader;
	_sloader = sloader;
	_vel2cam = vel2cam;
	_vel2car = vel2car;
	_projection = projection;
	_xsens2car = xsens2car;
	_use_xsens = use_xsens;
	_offset = offset;
	_imode = imode;
	_n_lidar_shots = 0;

	_p_sensor(3, 0) = 1.0;
	_p_car(3, 0) = 1.0;
	_p_world(3, 0) = 1.0;
}


void
SensorPreproc::reinitialize(DataSample *sample)
{
	_vloader->reinitialize(sample->velodyne_path, sample->n_laser_shots);

	if (_imode == COLOR)
		_img = _iloader->load(sample);
	else if (_imode == SEMANTIC)
		_img = _sloader->load(sample);

	_compute_transform_car2world(sample);
	_n_lidar_shots = sample->n_laser_shots;
}


vector<pcl::PointXYZRGB>
SensorPreproc::next_points()
{
	int valid;
	double h, v, r, in;
	LidarShot *shot;
	PointXYZRGB point;
	vector<pcl::PointXYZRGB> points;

	if (_vloader->done())
		return vector<pcl::PointXYZRGB>();

	shot = _vloader->next();

	for (int i = 0; i < shot->n; i++)
	{
		h = shot->h_angle;
		v = shot->v_angles[i];
		r = shot->ranges[i];
		in = shot->intensities[i];

		// this test is performed first to prevent additional calculations as soon as possible.
		if (!_spherical_point_is_valid(h, v, r))
			continue;

		_compute_point_in_different_references(h, v, r, &_p_sensor, &_p_car, &_p_world);

		if (!_point3d_is_valid(_p_sensor, _p_car, _p_world))
			continue;

		point = _create_point_and_intensity(_p_sensor, _p_world, in, &valid);

		if (valid)
			points.push_back(point);
	}

	return points;
}


int
SensorPreproc::size()
{
	return _n_lidar_shots;
}


void
SensorPreproc::_compute_transform_car2world(DataSample *sample)
{
	double roll, pitch, yaw;

	yaw = 0.0;
	pitch = 0.0;
	roll = 0.0;

	if (_use_xsens)
	{
		// convert xsens data to roll, pitch, yaw
		_r3x3 = sample->xsens.toRotationMatrix();
		_r3x3 = _move_xsens_to_car(_r3x3);
		getEulerYPR(_r3x3, yaw, pitch, roll);
	}

	_car2world = pose6d_to_matrix((sample->pose.x - _offset.x),
															(sample->pose.y - _offset.y),
															0., roll, pitch,
															sample->pose.th);
}


void
SensorPreproc::_compute_point_in_different_references(double h_angle, double v_angle, double range,
																										 Matrix<double, 4, 1> *p_sensor,
																										 Matrix<double, 4, 1> *p_car,
																										 Matrix<double, 4, 1> *p_world)
{
	double x, y, z;

	spherical2cartersian(v_angle, h_angle, range, &x, &y, &z);

	(*p_sensor)(0, 0) = x;
	(*p_sensor)(1, 0) = y;
	(*p_sensor)(2, 0) = z;
	(*p_sensor)(3, 0) = 1;

	(*p_car) = _vel2car * (*p_sensor);
	(*p_world) = _car2world * (*p_car);
}


int
SensorPreproc::_spherical_point_is_valid(double h_angle, double v_angle, double range)
{
	// the angles are not used now, but they can be used in the future.
	(void) h_angle;
	(void) v_angle;

	int ray_is_range_max = (range >= 70.0) || (range < 1.0);

  if (ray_is_range_max)
  		return 0;

  return 1;
}


Matrix<double, 3, 3>
SensorPreproc::_move_xsens_to_car(Matrix<double, 3, 3> xsens)
{
	// create a 4x4 transformation matrix from xsens 3x3 rotation matrix
	_r4x4(0, 0) = xsens(0, 0);
	_r4x4(0, 1) = xsens(0, 1);
	_r4x4(0, 2) = xsens(0, 2);
	_r4x4(0, 3) = 0;
	_r4x4(1, 0) = xsens(1, 0);
	_r4x4(1, 1) = xsens(1, 1);
	_r4x4(1, 2) = xsens(1, 2);
	_r4x4(1, 3) = 0;
	_r4x4(2, 0) = xsens(2, 0);
	_r4x4(2, 1) = xsens(2, 1);
	_r4x4(2, 2) = xsens(2, 2);
	_r4x4(2, 3) = 0;
	_r4x4(3, 0) = 0;
	_r4x4(3, 1) = 0;
	_r4x4(3, 2) = 0;
	_r4x4(3, 3) = 1;

	// move xsens to car
	_r4x4 = _xsens2car * _r4x4;

	// extract the 3x3 rotation from back the 4x4 transformation matrix.
	_r3x3(0, 0) = _r4x4(0, 0);
	_r3x3(0, 1) = _r4x4(0, 1);
	_r3x3(0, 2) = _r4x4(0, 2);
	_r3x3(1, 0) = _r4x4(1, 0);
	_r3x3(1, 1) = _r4x4(1, 1);
	_r3x3(1, 2) = _r4x4(1, 2);
	_r3x3(2, 0) = _r4x4(2, 0);
	_r3x3(2, 1) = _r4x4(2, 1);
	_r3x3(2, 2) = _r4x4(2, 2);

	return _r3x3;
}


int
SensorPreproc::_point3d_is_valid(Matrix<double, 4, 1> &p_sensor,
																 Matrix<double, 4, 1> &p_car,
																 Matrix<double, 4, 1> &p_world)
{
	// The points in the following frames are not used so far,
	// but they can be used in the future.
	(void) p_car;

	int ray_hit_car = fabs(p_sensor(0, 0)) < 6.0 && fabs(p_sensor(1, 0)) < 4.0;
	int ray_contains_nan = std::isnan(p_world(0, 0)) || std::isnan(p_world(1, 0)) || std::isnan(p_world(2, 0));
	int ray_contains_inf = std::isinf(p_world(0, 0)) || std::isinf(p_world(1, 0)) || std::isinf(p_world(2, 0));
	int ray_is_too_high = p_sensor(2, 0) > 0.0;
	int ray_is_too_low = p_sensor(2, 0) < -2.5;

	if (ray_hit_car
			|| ray_contains_nan
			|| ray_contains_inf
			|| ray_is_too_high
			//|| ray_is_too_low
			)
		return 0;

	return 1;
}


PointXYZRGB
SensorPreproc::_create_point_and_intensity(Matrix<double, 4, 1> &p_sensor,
																					 Matrix<double, 4, 1> &p_world,
																					 unsigned char intensity,
																					 int *valid)
{
	PointXYZRGB point;

	point.x = p_world(0, 0) / p_world(3, 0);
	point.y = p_world(1, 0) / p_world(3, 0);
	point.z = p_world(2, 0) / p_world(3, 0);

	// in INTENSITY mode, the point color is given by
	// the intensity observed by the lidar.
	if (_imode == INTENSITY)
	{
		unsigned char brightened = _brighten(intensity);

		point.r = brightened;
		point.g = brightened;
		point.b = brightened;

		*valid = 1;
	}
	// In the SEMANTIC and VISUAL modes, the point color
	// is obtained by projecting the points in the image,
	// and returning the respective pixel color.
	else
	{
		cv::Point pos_pixel;

		_get_pixel_position(p_sensor, _img, &pos_pixel, valid);

		if (*valid)
		{
			int p = 3 * (pos_pixel.y * _img.cols + pos_pixel.x);

			point.b = _img.data[p];
			point.g = _img.data[p + 1];
			point.r = _img.data[p + 2];
		}
	}

	return point;
}


unsigned char
SensorPreproc::_brighten(unsigned char val, unsigned int multiplier)
{
	unsigned int brightened = val * multiplier;

	if (brightened > 255)
		return 255;
	else
		return brightened;
}


void
SensorPreproc::_get_pixel_position(Matrix<double, 4, 1> &p_sensor,
																	 cv::Mat &img, cv::Point *ppixel,
																	 int *is_valid)
{
	*is_valid = 0;

	_p_cam = _vel2cam * p_sensor;

	// test to check if the point is in front of the camera.
	// points behind the camera can also be projected into the image plan.
	if (_p_cam(0, 0) / _p_cam(3, 0) > 0)
	{
		_p_pixel_homogeneous = _projection * _p_cam;

		ppixel->y = (_p_pixel_homogeneous(1, 0) / _p_pixel_homogeneous(2, 0)) * img.rows;
		ppixel->x = (_p_pixel_homogeneous(0, 0) / _p_pixel_homogeneous(2, 0)) * img.cols;

		// check if the point is visible by the camera.
		if (ppixel->x >= 0 && ppixel->x < img.cols && ppixel->y >= 0 && ppixel->y < img.rows)
			*is_valid = 1;
	}
}
