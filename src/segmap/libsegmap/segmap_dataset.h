
#ifndef _SEGMAP_LIBSEGMAP_SEGMAP_DATASET_H_
#define _SEGMAP_LIBSEGMAP_SEGMAP_DATASET_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <string>
#include <vector>
#include <cstring>
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
	static const int SYNC_BY_LIDAR = 1;
	
	NewCarmenDataset(const char *path, const char *odom_calib_path,
			int sync_type = SYNC_BY_CAMERA,
			const char *lidar_calib_path = "data/calibration_table.txt");

	~NewCarmenDataset();

	void reset();
	DataSample* next_data_package();

	// Returns a matrix to transform from lidar to camera frame.
	// Points shall be in homogeneous coordinates.
	// To obtain the pixel positions, multiply the transformed x-coordinate
	// by the image width, and y-coordinate by the image height.
	Eigen::Matrix<double, 4, 4> vel2cam();

	// matrix to project from camera frame to image coordinates.
	Eigen::Matrix<double, 3, 4> projection_matrix();

	// Returns a matrix to transfrom from lidar to car frame.
	Eigen::Matrix<double, 4, 4> vel2car();

	OdomCalib calib;
    unsigned char ***intensity_calibration;

protected:

	static const long _MAX_LINE_LENGTH = (5*4000000);

	std::string _images_path;
	std::string _velodyne_path;
	DataSample *_sample;
	int _sync_type;
	FILE *_fptr;

	std::vector<char*> _imu_queue;
	std::vector<char*> _gps_position_queue;
	std::vector<char*> _gps_orientation_queue;
	std::vector<char*> _odom_queue;
	std::vector<char*> _camera_queue;
	std::vector<char*> _velodyne_queue;

	void _load_odometry_calibration(const char *path);
	void _load_intensity_calibration(const char *path);
	void _clear_synchronization_queues();
	void _add_message_to_queue(char *data);
	void _assemble_data_package_from_queues();

	static void _free_queue(std::vector<char*> queue);
	static std::vector<char*> _find_nearest(std::vector<char*> &queue, double ref_time);

	static unsigned char*** _allocate_calibration_table();
	static void _free_calibration_table(unsigned char ***table);

	static void _parse_odom(std::vector<char*> data, DataSample *sample);
	static void _parse_imu(std::vector<char*> data, DataSample *sample);
	static void _parse_velodyne(std::vector<char*> data, DataSample *sample, std::string velodyne_path);
	static void _parse_camera(std::vector<char*> data, DataSample *sample, std::string image_path);
	static void _parse_gps_position(std::vector<char*> data, DataSample *sample);
	static void _parse_gps_orientation(std::vector<char*> data, DataSample *sample);
};

#endif


