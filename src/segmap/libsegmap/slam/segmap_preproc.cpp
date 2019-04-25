
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
#include <carmen/segmap_definitions.h>
#include <carmen/ackerman_motion_model.h>
#include <carmen/segmap_semantic_segmentation_viewer.h>

#include "segmap_preproc.h"

using namespace cv;
using namespace pcl;
using namespace std;
using namespace Eigen;


// TODO: include prob_map.h to prevent copies of code
float***
load_calibration_table(const char *calibration_file)
{
	FILE *calibration_file_bin = fopen(calibration_file, "r");
	if (!calibration_file_bin)
		return (NULL);

	float ***table;
	table = (float ***) calloc(32, sizeof(float **));
	for (int i = 0; i < 32; i++)
	{
		table[i] = (float **) calloc(10, sizeof(float *));
		for (int j = 0; j < 10; j++)
			table[i][j] = (float *) calloc(256, sizeof(float));
	}
	int laser, ray_size, intensity;
	long accumulated_intennsity, count;
	float val, max_val = 0.0, min_val = 255.0;
	while (fscanf(calibration_file_bin, "%d %d %d %f %ld %ld", &laser, &ray_size, &intensity, &val, &accumulated_intennsity, &count) == 6)
	{
		table[laser][ray_size][intensity] = val;
		if (val > max_val)
			max_val = val;
		if (val < min_val)
			min_val = val;
	}

	for (int i = 0; i < 32; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			for (int k = 0; k < 256; k++)
			{
				val = table[i][j][k];
				val = (val - min_val) / (max_val - min_val);
				if (val > 1.0)
					val = 1.0;
				if (val < 0.0)
					val = 0.0;
				table[i][j][k] = val;
			}
		}
	}
	fclose(calibration_file_bin);

	return (table);
}


int
get_distance_index(double distance)
{	// Gera um indice para cada faixa de distancias. As faixas crescem de ~50% a cada intervalo. Indices de zero a 9 permitem distancias de zero a ~70 metros
	if (distance < 3.5)
		return (0);

	int distance_index = (int) ((log(distance - 3.5 + 1.0) / log(1.45)) +  0.5);
	if (distance_index > 9)
		return (9);

	return (distance_index);
}


SensorPreproc::SensorPreproc(CarmenLidarLoader *vloader,
                             CarmenImageLoader *iloader,
                             SemanticSegmentationLoader *sloader,
                             Matrix<double, 4, 4> vel2cam,
                             Matrix<double, 4, 4> vel2car,
                             Matrix<double, 3, 4> projection,
                             Matrix<double, 4, 4> xsens2car,
                             int use_xsens,
                             IntensityMode imode,
                             std::string intensity_calib_path,
                             double ignore_above_threshold,
                             double ignore_below_threshold)
{
	_vloader = vloader;
	_iloader = iloader;
	_sloader = sloader;
	_vel2cam = vel2cam;
	_vel2car = vel2car;
	_projection = projection;
	_xsens2car = xsens2car;
	_use_xsens = use_xsens;
	_imode = imode;
	_n_lidar_shots = 0;
	_vel2car_inverse = _vel2car.inverse();

	_ignore_above_threshold = ignore_above_threshold;
	_ignore_below_threshold = ignore_below_threshold;

	_p_sensor(3, 0) = 1.0;
	_p_car(3, 0) = 1.0;
	_p_world(3, 0) = 1.0;

	//_initialize_calibration_table(lidar_calib_path);
	calibration_table = load_calibration_table(intensity_calib_path.c_str());
	_lane_mark_detection_active = 0;
}


SensorPreproc::~SensorPreproc()
{
	delete(_vloader);
	delete(_iloader);
	delete(_sloader);

	for (int i = 0; i < 32; i++)
	{
		for (int r = 0; r < 10; r++)
				free(calibration_table[i][r]);

		free(calibration_table[i]);
	}

	free(calibration_table);
}


void
SensorPreproc::reinitialize(DataSample *sample)
{
	_vloader->reinitialize(sample->velodyne_path, sample->n_laser_shots);

	if (_imode == COLOR)
	{
		_img = read_img(sample);
		_img_with_points = _img.clone();
	}

	if (_imode == SEMANTIC)
	{
		_img = read_segmented_img(sample);
		_img_with_points = segmented_image_view(_img);
	}

	_compute_transform_car2world(sample);
	_n_lidar_shots = sample->n_laser_shots;

	Pose2d step(0, 0, 0);
	ackerman_motion_model(step, sample->v, sample->phi, TIME_SPENT_IN_EACH_SCAN);

	_motion_correction = Pose2d::to_matrix(Pose2d(0, 0, 0));
	_motion_correction_step = Pose2d::to_matrix(step);
}


vector<pcl::PointXYZRGB>
SensorPreproc::_next_points(SensorReference ref)
{
	int valid;
	double h, v, r, in;
	LidarShot *shot;
	PointXYZRGB point;
	vector<pcl::PointXYZRGB> points;

	if (_vloader->done())
		return vector<pcl::PointXYZRGB>();

	_corrected_vel2car = _motion_correction * _vel2car;

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

		if (!_point3d_is_valid(_p_sensor, _p_car, _p_world, _ignore_above_threshold, _ignore_below_threshold))
			continue;

		point = _create_point_and_intensity(_p_sensor, _p_car, _p_world, in, &valid, ref, i);

		if (valid)
			points.push_back(point);
	}

	_motion_correction *= _motion_correction_step;

	return points;
}


std::vector<pcl::PointXYZRGB>
SensorPreproc::next_points_in_sensor()
{
	return _next_points(SENSOR_REFERENCE);
}


std::vector<pcl::PointXYZRGB>
SensorPreproc::next_points_in_car()
{
	return _next_points(CAR_REFERENCE);
}


std::vector<pcl::PointXYZRGB>
SensorPreproc::next_points_in_world()
{
	return _next_points(WORLD_REFERENCE);
}


// how to prevent this method of being a copy of _next_points?
std::vector<SensorPreproc::CompletePointData>
SensorPreproc::next_points()
{
	int valid;
	LidarShot *shot;
	PointXYZRGB point;
	std::vector<SensorPreproc::CompletePointData> points;

	if (_vloader->done())
		return points;

	shot = _vloader->next();
	_corrected_vel2car = _motion_correction * _vel2car;

	for (int i = 0; i < shot->n; i++)
	{
		CompletePointData p;

		p.laser_id = i;
		p.h_angle = shot->h_angle;
		p.v_angle = shot->v_angles[i];
		p.range = shot->ranges[i];
		p.raw_intensity = shot->intensities[i];

		_compute_point_in_different_references(p.h_angle, p.v_angle, p.range,
		                                       &_p_sensor, &_p_car, &_p_world);

		_point_coords_from_mat(_p_car, &p.car);
		_point_coords_from_mat(_p_sensor, &p.sensor);
		_point_coords_from_mat(_p_world, &p.world);

		if (_spherical_point_is_valid(p.h_angle, p.v_angle, p.range)
				&& _point3d_is_valid(_p_sensor, _p_car, _p_world, _ignore_above_threshold, _ignore_below_threshold))
		{
			_adjust_intensity(&p.sensor, _p_sensor, p.raw_intensity, &valid, i);

			if (valid)
			{
				p.world.r = p.car.r = p.sensor.r;
				p.world.g = p.car.g = p.sensor.g;
				p.world.b = p.car.b = p.sensor.b;

				points.push_back(p);
			}
		}
	}

	_motion_correction *= _motion_correction_step;

	return points;
}


int
SensorPreproc::size()
{
	return _n_lidar_shots;
}


Mat
SensorPreproc::read_segmented_img(DataSample *sample)
{
	Mat seg_img = _sloader->load(sample);

	if (_lane_mark_detection_active)
		_segment_lane_marks(seg_img, sample);

	return seg_img;
}


void
SensorPreproc::_segment_lane_marks(Mat &m, DataSample *sample)
{
	double r, g, b, gray;

	int st = (int) (_img.rows * ((double) 450. / 960.));
	int end = (int) (_img.rows * ((double) 750. / 960.));

	Mat img = read_img(sample);

	for (int i = st; i < end; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			b = img.data[3 * (i * img.cols + j)];
			g = img.data[3 * (i * img.cols + j) + 1];
			r = img.data[3 * (i * img.cols + j) + 2];
			gray = (b + g + r) / 3;

			if (gray > 40 && m.data[3 * (i * m.cols + j)] != 19)
			{
				m.data[3 * (i * m.cols + j)] = 20;
				m.data[3 * (i * m.cols + j) + 1] = 20;
				m.data[3 * (i * m.cols + j) + 2] = 20;
			}
		}
	}
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

	_car2world = pose6d_to_matrix((sample->pose.x),
	                              (sample->pose.y),
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

	(*p_car) = _corrected_vel2car * (*p_sensor);
	//(*p_car) = _vel2car * (*p_sensor);

	// compute the corrected point position in sensor's reference
	(*p_sensor) = _vel2car_inverse * (*p_car);
	(*p_world) = _car2world * (*p_car);

	(*p_car)(0, 0) /= (*p_car)(3, 0);
	(*p_car)(1, 0) /= (*p_car)(3, 0);
	(*p_car)(2, 0) /= (*p_car)(3, 0);
	(*p_car)(3, 0) = 1.0;

	(*p_sensor)(0, 0) /= (*p_car)(3, 0);
	(*p_sensor)(1, 0) /= (*p_car)(3, 0);
	(*p_sensor)(2, 0) /= (*p_car)(3, 0);
	(*p_sensor)(3, 0) = 1.0;

	(*p_world)(0, 0) /= (*p_car)(3, 0);
	(*p_world)(1, 0) /= (*p_car)(3, 0);
	(*p_world)(2, 0) /= (*p_car)(3, 0);
	(*p_world)(3, 0) = 1.0;
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
                                 Matrix<double, 4, 1> &p_world,
                                 double ignore_above_threshold,
                                 double ignore_below_threshold)
{
	int ray_hit_car = 0;
	int safe_border_x = 2.0;
	int safe_border_y = 2.0;

	// test if the ray is inside the car area in x-axis direction.
	// the conditions evaluate if the ray is between rear and rear axis, and between rear axis and front.
	int x_test = (p_car(0, 0) > -(distance_between_rear_car_and_rear_wheels + safe_border_x))
								&& (p_car(0, 0) < (car_length - distance_between_rear_car_and_rear_wheels + safe_border_x));

	// test if the ray is inside the car area in y-axis direction.
	int y_test = (p_car(1, 0) > -(car_width / 2. + safe_border_y))
								&& (p_car(1, 0) < (car_width / 2. + safe_border_y));

	if (x_test && y_test)
		ray_hit_car = 1;

	int ray_contains_nan = std::isnan(p_world(0, 0)) || std::isnan(p_world(1, 0)) || std::isnan(p_world(2, 0));
	int ray_contains_inf = std::isinf(p_world(0, 0)) || std::isinf(p_world(1, 0)) || std::isinf(p_world(2, 0));
	int ray_is_too_high = p_sensor(2, 0) > ignore_above_threshold;
	int ray_is_too_low = p_sensor(2, 0) < ignore_below_threshold;

	if (ray_hit_car
			|| ray_contains_nan
			|| ray_contains_inf
			|| ray_is_too_high
			|| ray_is_too_low
	)
		return 0;

	return 1;
}


void
SensorPreproc::_point_coords_from_mat(Eigen::Matrix<double, 4, 1> &mat, PointXYZRGB *point)
{
	point->x = mat(0, 0) / mat(3, 0);
	point->y = mat(1, 0) / mat(3, 0);
	point->z = mat(2, 0) / mat(3, 0);
}


unsigned char
SensorPreproc::_get_calibrated_and_brighten_intensity(unsigned char raw_intensity, Matrix<double, 4, 1> &p_sensor, int laser_id)
{
	unsigned char intensity = raw_intensity;

	double ray_size_in_the_floor = sqrt(pow(p_sensor(0, 0), 2) + pow(p_sensor(1, 0), 2));
	int ray_distance_index = get_distance_index(ray_size_in_the_floor);
	intensity = (unsigned char) (255 * calibration_table[laser_id][ray_distance_index][intensity]);

	if (_imode == INTENSITY)
		intensity = _brighten(intensity, 2);
	else if (_imode == BRIGHT)
		intensity = _brighten(intensity, 4);

	return intensity;
}


void
SensorPreproc::_adjust_intensity(PointXYZRGB *point, Matrix<double, 4, 1> &p_sensor, unsigned char raw_intensity, int *valid, int laser_id)
{
	// in INTENSITY mode, the point color is given by
	// the intensity observed by the lidar.
	if (_imode == INTENSITY || _imode == RAW_INTENSITY || _imode == BRIGHT)
	{
		unsigned char intensity = _get_calibrated_and_brighten_intensity(raw_intensity, p_sensor, laser_id);

		point->r = intensity;
		point->g = intensity;
		point->b = intensity;

		*valid = 1;
	}
	// In the SEMANTIC and VISUAL modes, the point color
	// is obtained by projecting the points in the image,
	// and returning the respective pixel color.
	else
	{
		cv::Point pos_pixel;

		_get_pixel_position(p_sensor, _img.rows, _img.cols, &pos_pixel, valid);

		if (*valid)
		{
			int p = 3 * (pos_pixel.y * _img.cols + pos_pixel.x);

			point->b = _img.data[p];
			point->g = _img.data[p + 1];
			point->r = _img.data[p + 2];

			circle(_img_with_points, Point(pos_pixel.x, pos_pixel.y), 2, Scalar(0, 0, 255), -1);
		}
	}
}


PointXYZRGB
SensorPreproc::_create_point_and_intensity(Matrix<double, 4, 1> &p_sensor,
                                           Matrix<double, 4, 1> &p_car,
                                           Matrix<double, 4, 1> &p_world,
                                           unsigned char intensity,
                                           int *valid,
                                           SensorReference ref,
                                           int laser_id)
{
	PointXYZRGB point;

	if (ref == SENSOR_REFERENCE)
		_point_coords_from_mat(p_sensor, &point);
	else if (ref == CAR_REFERENCE)
		_point_coords_from_mat(p_car, &point);
	else
		_point_coords_from_mat(p_world, &point);

	_adjust_intensity(&point, p_sensor, intensity, valid, laser_id);

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
                                   int img_rows, int img_cols, cv::Point *ppixel,
                                   int *is_valid)
{
	*is_valid = 0;

	_p_cam = _vel2cam * p_sensor;

	// test to check if the point is in front of the camera.
	// points behind the camera can also be projected into the image plan.
	if (_p_cam(0, 0) / _p_cam(3, 0) > 0)
	{
		_p_pixel_homogeneous = _projection * _p_cam;

		ppixel->y = (_p_pixel_homogeneous(1, 0) / _p_pixel_homogeneous(2, 0)) * img_rows;
		ppixel->x = (_p_pixel_homogeneous(0, 0) / _p_pixel_homogeneous(2, 0)) * img_cols;

		// check if the point is in the area to be ignored.
		int point_hit_car = ppixel->y > (img_rows * (380. / 480.));
		int point_hit_sky = ppixel->y < (img_rows * (40. / 480.));		
		
		// check if the point is visible by the camera.
		if (ppixel->x >= 0 && ppixel->x < img_cols && ppixel->y >= 0 && ppixel->y < img_rows && !point_hit_car && !point_hit_sky)
			*is_valid = 1;
	}
}


void load_as_pointcloud(SensorPreproc &preproc,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                        SensorPreproc::SensorReference ref)
{
	vector<PointXYZRGB> points;
	cloud->clear();

	for (int i = 0; i < preproc.size(); i++)
	{
		if (ref == SensorPreproc::SENSOR_REFERENCE)
			points = preproc.next_points_in_sensor();
		else if (ref == SensorPreproc::CAR_REFERENCE)
			points = preproc.next_points_in_car();
		else
			points = preproc.next_points_in_world();

		for (int j = 0; j < points.size(); j++)
			cloud->push_back(points[j]);
	}
}


pcl::PointXYZRGB
transform_point(Eigen::Matrix<double, 4, 4> &t, pcl::PointXYZRGB &p_in)
{
	Eigen::Matrix<double, 4, 1> p_in_mat, p_out_mat;
	pcl::PointXYZRGB p_out;

	p_in_mat(0, 0) = p_in.x;
	p_in_mat(1, 0) = p_in.y;
	p_in_mat(2, 0) = p_in.z;
	p_in_mat(3, 0) = 1.0;

	p_out_mat = t * p_in_mat;

	p_out.x = p_out_mat(0, 0) / p_out_mat(3, 0);
	p_out.y = p_out_mat(1, 0) / p_out_mat(3, 0);
	p_out.z = p_out_mat(2, 0) / p_out_mat(3, 0);

	p_out.r = p_in.r;
	p_out.g = p_in.g;
	p_out.b = p_in.b;

	return p_out;
}


pcl::PointXYZRGB
transform_point(Pose2d &t, pcl::PointXYZRGB &p_in)
{
	pcl::PointXYZRGB p_out;

	double c = cos(t.th);
	double s = sin(t.th);

	p_out.x = p_in.x * c + p_in.y * (-s) + t.x;
	p_out.y = p_in.x * s + p_in.y * c + t.y;

	p_out.z = p_in.z;
	p_out.r = p_in.r;
	p_out.g = p_in.g;
	p_out.b = p_in.b;

	return p_out;
}


