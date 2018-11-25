
#ifndef _SEGMAP_LIBSEGMAP_SEGMAP_DATASET_H_
#define _SEGMAP_LIBSEGMAP_SEGMAP_DATASET_H_

#include <Eigen/Core>
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


class DatasetInterface
{
protected:
	char _name[1024]; // utility attribute for creating file names.
	string _path;
	int _use_segmented;

public:
	virtual Mat load_image(int i) = 0;
	virtual void load_pointcloud(int i, PointCloud<PointXYZRGB>::Ptr cloud) = 0;
	void load_fused_pointcloud_and_camera(int i, PointCloud<PointXYZRGB>::Ptr cloud, int view=0);

	virtual Matrix<double, 3, 1> transform_vel2cam(PointXYZRGB &p) = 0;
	virtual Matrix<double, 4, 4> transform_vel2car() = 0;
	virtual void load_data(vector<double> &times,
			vector<Matrix<double, 4, 4>> &poses,
			vector<pair<double, double>> &odom) = 0;

	DatasetInterface(string path, int use_segmented)
	{
		_path = path;
		_use_segmented = use_segmented;
		strcpy(_name, "");
	}

	virtual ~DatasetInterface() {}
};


class DatasetCarmen : public DatasetInterface
{
	int _image_height;
	int _image_width;
	int _unknown_class;
	Matrix<double, 3, 4> _vel2cam;
	Matrix<double, 4, 4> _vel2car;

	void _init_vel2cam_transform();
	void _init_vel2car_transform();

public:
	virtual Mat load_image(int i);
	virtual void load_pointcloud(int i, PointCloud<PointXYZRGB>::Ptr cloud);
	virtual Matrix<double, 3, 1> transform_vel2cam(PointXYZRGB &p);
	virtual Matrix<double, 4, 4> transform_vel2car();

	// TODO: load odometry biases from file.
	virtual void load_data(vector<double> &times,
			vector<Matrix<double, 4, 4>> &poses,
			vector<pair<double, double>> &odom);

	DatasetCarmen(int image_height, int image_width, string path, int use_segmented);
	~DatasetCarmen() {}
};


class DatasetKitti : public DatasetInterface
{
	void _load_oxts(vector<double> &times, vector<vector<double>> &data);
	void _load_timestamps(vector<double> &times);

	// assumes for simplicity that phi is zero.
	void _estimate_v(vector<Matrix<double, 4, 4>> &poses, vector<double> &ts, vector<pair<double, double>> &odom);

	Matrix<double, 3, 4> _vel2cam;
	void _init_vel2cam_transform();

public:
	virtual Mat load_image(int i);
	virtual void load_pointcloud(int i, PointCloud<PointXYZRGB>::Ptr cloud);
	virtual Matrix<double, 3, 1> transform_vel2cam(PointXYZRGB &p);
	virtual Matrix<double, 4, 4> transform_vel2car();

	// TODO: load odometry biases from file.
	virtual void load_data(vector<double> &times,
			vector<Matrix<double, 4, 4>> &poses,
			vector<pair<double, double>> &odom);

	DatasetKitti(string path, int use_segmented) :
		DatasetInterface(path, use_segmented)
	{
		_init_vel2cam_transform();
	}

	~DatasetKitti() {}
};


#endif


