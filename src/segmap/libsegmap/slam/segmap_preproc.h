
#ifndef __SEGMAP_PREPROC_H__
#define __SEGMAP_PREPROC_H__


#include <map>
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
		REFLECTIVITY = 0,
		CAMERA,
		SEMANTIC, 
	};

	enum SensorReference
	{
		SENSOR_REFERENCE = 0,
		CAR_REFERENCE,
		WORLD_REFERENCE
	};

	class CompletePointData
	{
	public:
		int laser_id;
		double h_angle, v_angle, range;
		unsigned char raw_intensity;

		pcl::PointXYZRGB world;
		pcl::PointXYZRGB car;
		pcl::PointXYZRGB sensor;

		int valid;
	};

	SensorPreproc(CarmenLidarLoader *vloader,
								CarmenImageLoader *iloader,
								SemanticSegmentationLoader *sloader,
								Eigen::Matrix<double, 4, 4> vel2cam,
								Eigen::Matrix<double, 4, 4> vel2car,
								Eigen::Matrix<double, 3, 4> projection,
								Eigen::Matrix<double, 4, 4> xsens2car,
								int use_xsens,
								IntensityMode sync_sensor = REFLECTIVITY,
								std::string intensity_calib_path = "none",
								double ignore_above_threshold = DBL_MAX,
								double ignore_below_threshold = -DBL_MAX);

	~SensorPreproc();

	void reinitialize(DataSample *sample);
	std::vector<pcl::PointXYZRGB> next_points_in_sensor();
	std::vector<pcl::PointXYZRGB> next_points_in_world();
	std::vector<pcl::PointXYZRGB> next_points_in_car();
	std::vector<CompletePointData> next_points();
	int size();

	void set_semantic_remapping_flag(int flag) { _use_semantic_remapping = flag; }

	cv::Mat get_sample_img() { return _img; }
	cv::Mat get_sample_img_with_points() { return _img_with_points; }

	void set_lane_mark_detection(int on_or_off) { _lane_mark_detection_active = on_or_off; }

	IntensityMode _intensity_mode;

	cv::Mat read_img(DataSample *sample) { return _iloader->load(sample); }
	cv::Mat read_segmented_img(DataSample *sample);

	int _lane_mark_detection_active;
	void _segment_lane_marks(cv::Mat &m, DataSample *sample);

	CarmenLidarLoader *_vloader;
	CarmenImageLoader *_iloader;
	SemanticSegmentationLoader *_sloader;
	Eigen::Matrix<double, 4, 4> _vel2cam;
	Eigen::Matrix<double, 4, 4> _vel2car;
	Eigen::Matrix<double, 4, 4> _vel2car_inverse;
	Eigen::Matrix<double, 3, 4> _projection;
	Eigen::Matrix<double, 4, 4> _xsens2car;
	Eigen::Matrix<double, 4, 4> _motion_correction;
	Eigen::Matrix<double, 4, 4> _motion_correction_step;
	Eigen::Matrix<double, 4, 4> _corrected_car2world;

	int _use_semantic_remapping;
	int _use_xsens;
	int _n_lidar_shots;

	cv::Mat _img;
	cv::Mat _img_with_points;

  static const int _n_distance_indices = 10;
  float ***calibration_table;
  float calibration_table_tf[32][256];

	double _ignore_above_threshold;
	double _ignore_below_threshold;

	Eigen::Matrix<double, 4, 4> _car2world;

	// auxiliary matrices for internal computations.
	Eigen::Matrix<double, 3, 3> _r3x3;
	Eigen::Matrix<double, 4, 4> _r4x4;

	// point in different coordinate systems
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

	void _adjust_intensity(pcl::PointXYZRGB *point, Eigen::Matrix<double, 4, 1> &p_sensor, unsigned char raw_intensity, int *valid, int laser_id);

	pcl::PointXYZRGB _create_point_and_intensity(Eigen::Matrix<double, 4, 1> &p_sensor,
																							 Eigen::Matrix<double, 4, 1> &p_car,
																							 Eigen::Matrix<double, 4, 1> &p_world,
																							 unsigned char intensity,
																							 int *valid,
																							 SensorReference ref,
																							 int laser_id);

	static unsigned char _brighten(unsigned char val, unsigned int multiplier = 5);

	void _get_pixel_position(Eigen::Matrix<double, 4, 1> &p_sensor,
	                         int img_rows, int img_cols, cv::Point *ppixel,
													 int *is_valid);

	static void _point_coords_from_mat(Eigen::Matrix<double, 4, 1> &mat, pcl::PointXYZRGB *point);

	std::vector<pcl::PointXYZRGB> _next_points(SensorReference ref);

	unsigned char _get_calibrated_intensity(unsigned char raw_intensity, Eigen::Matrix<double, 4, 1> &p_sensor, int laser_id);
	unsigned char _get_calibrated_intensity_tf(unsigned char raw_intensity, Eigen::Matrix<double, 4, 1> &p_sensor, int laser_id);
};


void load_as_pointcloud(SensorPreproc &preproc,
												pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
												SensorPreproc::SensorReference ref);

pcl::PointXYZRGB
transform_point(Eigen::Matrix<double, 4, 4> &t, pcl::PointXYZRGB &p_in);

pcl::PointXYZRGB
transform_point(Pose2d &t, pcl::PointXYZRGB &p_in);

#endif
