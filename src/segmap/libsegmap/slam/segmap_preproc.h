
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

	enum SensorReference
	{
		SENSOR_REFERENCE,
		CAR_REFERENCE,
		WORLD_REFERENCE
	};

	enum IntensityMode
	{
		INTENSITY = 0,
		COLOR,
		SEMANTIC,
	};

	class CompletePointData
	{
		int laser_id;
		double h_angle, v_angle, range;
		unsigned char raw_intensity;

		pcl::PointXYZRGB world;
		pcl::PointXYZRGB car;
		pcl::PointXYZRGB sensor;
	};

	SensorPreproc(CarmenLidarLoader *vloader,
								CarmenImageLoader *iloader,
								SemanticSegmentationLoader *sloader,
								Eigen::Matrix<double, 4, 4> vel2cam,
								Eigen::Matrix<double, 4, 4> vel2car,
								Eigen::Matrix<double, 3, 4> projection,
								Eigen::Matrix<double, 4, 4> xsens2car,
								int use_xsens,
								IntensityMode imode = INTENSITY,
								double ignore_above_threshold = DBL_MAX,
								double ignore_below_threshold = -DBL_MAX);

	~SensorPreproc();

	void reinitialize(DataSample *sample);
	std::vector<pcl::PointXYZRGB> next_points_in_sensor();
	std::vector<pcl::PointXYZRGB> next_points_in_world();
	std::vector<pcl::PointXYZRGB> next_points_in_car();
	std::vector<CompletePointData> next_points();
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
	IntensityMode _imode;
	int _n_lidar_shots;

	cv::Mat _img;

	double _ignore_above_threshold;
	double _ignore_below_threshold;

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
															 Eigen::Matrix<double, 4, 1> &p_world,
															 double ignore_above_threshold,
															 double ignore_below_threshold);

	void _adjust_intensity(pcl::PointXYZRGB &point, Eigen::Matrix<double, 4, 1> &p_sensor, int *valid);

	pcl::PointXYZRGB _create_point_and_intensity(Eigen::Matrix<double, 4, 1> &p_sensor,
																							 Eigen::Matrix<double, 4, 1> &p_car,
																							 Eigen::Matrix<double, 4, 1> &p_world,
																							 unsigned char intensity,
																							 int *valid,
																							 SensorReference ref);

	static unsigned char _brighten(unsigned char val, unsigned int multiplier = 5);

	void _get_pixel_position(Eigen::Matrix<double, 4, 1> &p_sensor,
													 cv::Mat &img, cv::Point *ppixel,
													 int *is_valid);

	static void _point_coords_from_mat(Eigen::Matrix<double, 4, 1> &mat, pcl::PointXYZRGB *point);

	std::vector<pcl::PointXYZRGB> _next_points(SensorReference ref);
};


void load_as_pointcloud(SensorPreproc &preproc,
												pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
												SensorPreproc::SensorReference ref);

pcl::PointXYZRGB
transform_point(Eigen::Matrix<double, 4, 4> &t, pcl::PointXYZRGB &p_in);

pcl::PointXYZRGB
transform_point(Pose2d &t, pcl::PointXYZRGB &p_in);

#endif
