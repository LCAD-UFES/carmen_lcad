
#ifndef _SEGMAP_LIBSEGMAP_SEGMAP_DATASET_H_
#define _SEGMAP_LIBSEGMAP_SEGMAP_DATASET_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <string>
#include <vector>
#include <opencv/cv.hpp>
#include "segmap_util.h"


class OdomCalib
{
public:
	double mult_v, add_v, mult_phi, add_phi, init_angle;
};


class DataSample
{
public:
	double velodyne_time;
	double image_time;
	double odom_time;
	double xsens_time;
	double gps_time;

	double v, phi;
	int gps_quality;
	int gps_orientation_quality;
	int image_width, image_height;
	int n_laser_shots;

	Eigen::Quaterniond xsens;
	
	Pose2d pose;
	Pose2d pose_with_loop_closure;
	Pose2d pose_registered_to_map;
	Pose2d gps;

	std::string velodyne_path;
	std::string image_path;
};


class DatasetInterface
{
public:
	int image_height;
	int image_width;
	std::vector<DataSample> data;
	OdomCalib odom_calib;
	std::string _path;
	int _use_segmented;

	virtual cv::Mat load_image(int i) = 0;
	virtual void load_pointcloud(int i, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double v, double phi) = 0;
	void load_fused_pointcloud_and_camera(int i, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double v, double phi, int view = 0, cv::Mat *output_img_view=0);

	virtual Eigen::Matrix<double, 3, 1> transform_vel2cam(pcl::PointXYZRGB &p) = 0;
	virtual Eigen::Matrix<double, 4, 4> transform_vel2car() = 0;
	virtual void load_data() = 0;

	DatasetInterface(std::string path, int use_segmented)
	{
		_path = path;
		_use_segmented = use_segmented;
		image_height = image_width = 0;
	}

	virtual ~DatasetInterface() {}
};


class DatasetCarmen : public DatasetInterface
{
	int _unknown_class;
	Eigen::Matrix<double, 3, 4> _vel2cam;
	Eigen::Matrix<double, 4, 4> _vel2car;

	bool _vel2cam_initialized;

	void _init_vel2cam_transform(int image_height, int image_width);
	void _init_vel2car_transform();

	void _segment_lane_marks(cv::Mat &m, int i);

public:

	virtual cv::Mat load_image(int i);
	virtual void load_pointcloud(int i, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double v, double phi);
	virtual Eigen::Matrix<double, 3, 1> transform_vel2cam(pcl::PointXYZRGB &p);
	virtual Eigen::Matrix<double, 4, 4> transform_vel2car();
	virtual void load_data();

	DatasetCarmen(std::string path, int use_segmented);
	~DatasetCarmen() {}
};


class DatasetKitti : public DatasetInterface
{
protected:
	char _name[1024]; // utility attribute for creating file names.

	void _load_oxts(std::vector<double> &times, std::vector<std::vector<double>> &data);
	void _load_timestamps(std::vector<double> &times);

	// assumes for simplicity that phi is zero.
	void _estimate_v(std::vector<Eigen::Matrix<double, 4, 4>> &poses,
									 std::vector<double> &ts,
									 std::vector<std::pair<double, double>> &odom);

	Eigen::Matrix<double, 3, 4> _vel2cam;
	void _init_vel2cam_transform();

public:
	virtual cv::Mat load_image(int i);
	virtual void load_pointcloud(int i, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double v, double phi);
	virtual Eigen::Matrix<double, 3, 1> transform_vel2cam(pcl::PointXYZRGB &p);
	virtual Eigen::Matrix<double, 4, 4> transform_vel2car();

	// TODO: load odometry biases from file.
	virtual void load_data();

	DatasetKitti(std::string path, int use_segmented) :
		DatasetInterface(path, use_segmented)
	{
		load_data();
		_init_vel2cam_transform();
	}

	~DatasetKitti() {}
};


class NewCarmenDataset
{
public:
	
	static const int SYNC_BY_CAMERA = 0;
	static const int SYNC_BY_VELODYNE = 1;
	
	NewCarmenDataset(std::string path,
	                 std::string odom_calib_path = "",
									 std::string fused_odom_path = "",
	                 int sync_type = SYNC_BY_CAMERA,
	                 std::string lidar_calib_path = "");

	~NewCarmenDataset();

	// Returns a matrix to transform from lidar to camera frame.
	// Points shall be in homogeneous coordinates.
	// To obtain the pixel positions, multiply the transformed x-coordinate
	// by the image width, and y-coordinate by the image height.
	Eigen::Matrix<double, 4, 4> vel2cam();

	// matrix to project from camera frame to image coordinates.
	Eigen::Matrix<double, 3, 4> projection_matrix();

	// Returns a matrix to transfrom from lidar to car frame.
	Eigen::Matrix<double, 4, 4> vel2car();

	unsigned char ***intensity_calibration;

	// Number of messages of the sensor used for synchronization.
	// For example, if the velodyne is the reference sensor, the method returns
	// the number of velodyne messages in the log.
	int size() const;

	// returns the i-th synchronized data package.
	DataSample* operator[](int i) const;
	DataSample* at(int i) const;

	double initial_angle() const;

protected:

	static const long _MAX_LINE_LENGTH = (5*4000000);

	std::string _images_dir;
	std::string _velodyne_dir;

	int _sync_type;
	OdomCalib _calib;
	std::vector<DataSample*> _data;
	std::vector<Pose2d> _fused_odom;

	std::vector<std::string> _imu_messages;
	std::vector<std::string> _gps_position_messages;
	std::vector<std::string> _gps_orientation_messages;
	std::vector<std::string> _odom_messages;
	std::vector<std::string> _camera_messages;
	std::vector<std::string> _velodyne_messages;

	std::vector<double> _imu_times;
	std::vector<double> _gps_position_times;
	std::vector<double> _gps_orientation_times;
	std::vector<double> _odom_times;
	std::vector<double> _camera_times;
	std::vector<double> _velodyne_times;

	void _load_log(std::string &path);
	void _load_odometry_calibration(std::string &path);
	void _load_intensity_calibration(std::string &path);
	void _load_poses(std::string &path, std::vector<Pose2d> *poses);
	void _update_data_with_poses();

	DataSample* _next_data_package(FILE *fptr);
	DataSample* _assemble_data_package_from_queues();

	void _clear_synchronization_queues();
	void _add_message_to_queue(std::string);
	static std::vector<std::string> _find_nearest(std::vector<std::string> &queue, std::vector<double> &times, double ref_time);

	static unsigned char*** _allocate_calibration_table();
	static void _free_calibration_table(unsigned char ***table);

	static void _parse_odom(std::vector<std::string> data, DataSample *sample);
	static void _parse_imu(std::vector<std::string> data, DataSample *sample);
	static void _parse_velodyne(std::vector<std::string> data, DataSample *sample, std::string velodyne_path);
	static void _parse_camera(std::vector<std::string> data, DataSample *sample, std::string image_path);
	static void _parse_gps_position(std::vector<std::string> data, DataSample *sample);
	static void _parse_gps_orientation(std::vector<std::string> data, DataSample *sample);

	void _read_log_msgs(FILE *fptr);
};

#endif


