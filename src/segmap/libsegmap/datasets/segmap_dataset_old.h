
#ifndef __SEGMAP_DATASET_OLD_H__
#define __SEGMAP_DATASET_OLD_H__

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv/cv.hpp>

#include <carmen/synchronized_data_package.h>
#include <carmen/odom_calib_data.h>


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
	void load_fused_pointcloud_and_camera(int i, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
	                                      double v, double phi, int use_remission = 1,
	                                      int view = 0, cv::Mat *output_img_view=0);

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


#endif
