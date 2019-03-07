
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

using namespace std;
using namespace pcl;
using namespace cv;
using namespace Eigen;


class OdomCalib
{
public:
	double mult_v, mult_phi, add_phi, init_angle;
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

	Quaterniond xsens;
	
	Pose2d pose;
	Pose2d pose_with_loop_closure;
	Pose2d pose_registered_to_map;
	Pose2d gps;

	string velodyne_path;
	string image_path;
};


class DatasetInterface
{
public:
	int image_height;
	int image_width;
	vector<DataSample> data;
	OdomCalib odom_calib;
	string _path;
	int _use_segmented;

	virtual Mat load_image(int i) = 0;
	virtual void load_pointcloud(int i, PointCloud<PointXYZRGB>::Ptr cloud, double v, double phi) = 0;
	void load_fused_pointcloud_and_camera(int i, PointCloud<PointXYZRGB>::Ptr cloud, double v, double phi, int view = 0, Mat *output_img_view=0);

	virtual Matrix<double, 3, 1> transform_vel2cam(PointXYZRGB &p) = 0;
	virtual Matrix<double, 4, 4> transform_vel2car() = 0;
	virtual void load_data() = 0;

	DatasetInterface(string path, int use_segmented)
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
	Matrix<double, 3, 4> _vel2cam;
	Matrix<double, 4, 4> _vel2car;

	uchar ***_velodyne_intensity_calibration;

	bool _vel2cam_initialized;

	void _load_velodyne_intensity_calibration();
	void _init_vel2cam_transform(int image_height, int image_width);
	void _init_vel2car_transform();

	void _segment_lane_marks(Mat &m, int i);

public:

	virtual Mat load_image(int i);
	virtual void load_pointcloud(int i, PointCloud<PointXYZRGB>::Ptr cloud, double v, double phi);
	virtual Matrix<double, 3, 1> transform_vel2cam(PointXYZRGB &p);
	virtual Matrix<double, 4, 4> transform_vel2car();
	virtual void load_data();

	DatasetCarmen(string path, int use_segmented);
	~DatasetCarmen() {}
};


class DatasetKitti : public DatasetInterface
{
protected:
	char _name[1024]; // utility attribute for creating file names.

	void _load_oxts(vector<double> &times, vector<vector<double>> &data);
	void _load_timestamps(vector<double> &times);

	// assumes for simplicity that phi is zero.
	void _estimate_v(vector<Matrix<double, 4, 4>> &poses, vector<double> &ts, vector<pair<double, double>> &odom);

	Matrix<double, 3, 4> _vel2cam;
	void _init_vel2cam_transform();

public:
	virtual Mat load_image(int i);
	virtual void load_pointcloud(int i, PointCloud<PointXYZRGB>::Ptr cloud, double v, double phi);
	virtual Matrix<double, 3, 1> transform_vel2cam(PointXYZRGB &p);
	virtual Matrix<double, 4, 4> transform_vel2car();

	// TODO: load odometry biases from file.
	virtual void load_data();

	DatasetKitti(string path, int use_segmented) :
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
	
	NewCarmenDataset(char *path, int sync_type = SYNC_BY_CAMERA);
	~NewCarmenDataset();
	void reset();
	DataSample* next_data_package();
	static Mat read_image(DataSample *sample);
	static PointCloud<PointXYZRGB>::Ptr read_pointcloud(DataSample *sample);

protected:

	static const long _MAX_LINE_LENGTH = (5*4000000);

	string _images_path;
	string _velodyne_path;
	DataSample *_sample;
	int _sync_type;
	FILE *_fptr;

	vector<char*> _imu_queue;
	vector<char*> _gps_position_queue;
	vector<char*> _gps_orientation_queue;
	vector<char*> _odom_queue;
	vector<char*> _camera_queue;
	vector<char*> _velodyne_queue;

	void _clear_synchronization_queues();
	void _add_message_to_queue(char *data);
	void _assemble_data_package_from_queues();
	static void _free_queue(vector<char*> queue);

	static vector<char*> _find_nearest(vector<char*> &queue, double ref_time);

	static void _parse_odom(vector<char*> data, DataSample *sample);
	static void _parse_imu(vector<char*> data, DataSample *sample);
	static void _parse_velodyne(vector<char*> data, DataSample *sample, string velodyne_path);
	static void _parse_camera(vector<char*> data, DataSample *sample, string image_path);
	static void _parse_gps_position(vector<char*> data, DataSample *sample);
	static void _parse_gps_orientation(vector<char*> data, DataSample *sample);

	static PointXYZRGB _compute_point_from_velodyne(double v_angle, double h_angle, double radius, unsigned char intensity);

};


#endif


