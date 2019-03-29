
#ifndef __SEGMAP_PREPROC_H__
#define __SEGMAP_PREPROC_H__

#include <Eigen/Core>
#include <opencv/cv.hpp>
#include <pcl/point_types.h>
#include <carmen/segmap_pose2d.h>
#include <carmen/carmen_lidar_reader.h>
#include <carmen/carmen_image_reader.h>
#include <carmen/carmen_semantic_segmentation_reader.h>


class SensorPreproc
{
public:

	enum IntensityMode
	{
		INTENSITY = 0,
		COLOR,
		SEMANTIC,
	};

	SensorPreproc(CarmenLidarLoader *vloader,
								CarmenImageLoader *iloader,
								SemanticSegmentationLoader *sloader,
								Eigen::Matrix<double, 4, 4> vel2cam,
								Eigen::Matrix<double, 4, 4> vel2car,
								Eigen::Matrix<double, 3, 4> projection,
								Eigen::Matrix<double, 4, 4> xsens2car,
								int use_xsens,
			          Pose2d offset,
								IntensityMode imode = INTENSITY);

	void reinitialize(DataSample *sample);
	std::vector<pcl::PointXYZRGB> next_points();
	int size();

protected:

	CarmenLidarLoader *_vloader;
	CarmenImageLoader *_iloader;
	SemanticSegmentationLoader *_sloader;
	Eigen::Matrix<double, 4, 4> _vel2cam;
	Eigen::Matrix<double, 4, 4> _vel2car;
	Eigen::Matrix<double, 3, 4> _projection;
	Eigen::Matrix<double, 4, 4> _xsens2car;
	int _use_xsens;
	Pose2d _offset;
	IntensityMode _imode;
	int _n_lidar_shots;

	cv::Mat _img;

	Eigen::Matrix<double, 4, 4> _car2world;

	// auxiliary matrices for internal computations.
	Eigen::Matrix<double, 3, 3> _r3x3;
	Eigen::Matrix<double, 4, 4> _r4x4;

	Eigen::Matrix<double, 4, 1> _p_sensor;
	Eigen::Matrix<double, 4, 1> _p_car;
	Eigen::Matrix<double, 4, 1> _p_cam;
	Eigen::Matrix<double, 3, 1> _p_pixel_homogeneous;
	Eigen::Matrix<double, 4, 1> _p_world;

	void _compute_transform_car2world(DataSample *sample);

	void _compute_point_in_different_references(double h_angle, double v_angle, double range,
																						 Eigen::Matrix<double, 4, 1> *p_sensor,
																						 Eigen::Matrix<double, 4, 1> *p_car,
																						 Eigen::Matrix<double, 4, 1> *p_world);

	static int _spherical_point_is_valid(double h_angle, double v_angle, double range);

	Eigen::Matrix<double, 3, 3> _move_xsens_to_car(Eigen::Matrix<double, 3, 3> xsens);

	static int _point3d_is_valid(Eigen::Matrix<double, 4, 1> &p_sensor,
															 Eigen::Matrix<double, 4, 1> &p_car,
															 Eigen::Matrix<double, 4, 1> &p_world);

	pcl::PointXYZRGB _create_point_and_intensity(Eigen::Matrix<double, 4, 1> &p_sensor,
																							 Eigen::Matrix<double, 4, 1> &p_world,
																							 unsigned char intensity,
																							 int *valid);

	static unsigned char _brighten(unsigned char val, unsigned int multiplier = 5);

	void _get_pixel_position(Eigen::Matrix<double, 4, 1> &p_sensor,
													 cv::Mat &img, cv::Point *ppixel,
													 int *is_valid);

};


#endif
