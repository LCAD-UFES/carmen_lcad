
#ifndef _SEGMAP_UTIL_H_
#define _SEGMAP_UTIL_H_

#include <opencv/cv.h>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include "segmap_pose2d.h"


enum SEGMAP_MODE
{
	SEGMAP_MODE_REMISSION,
	SEGMAP_MODE_VISUAL,
	SEGMAP_MODE_SEMANTIC
};

const double MAX_RANGE = 100.;


class CityScapesColorMap
{
	std::vector<cv::Scalar> _colors;

public:
	int n_classes;

	CityScapesColorMap()
	{
		n_classes = 21;

		_colors.push_back(cv::Scalar(128, 64, 128));
		_colors.push_back(cv::Scalar(244, 35, 232));
		_colors.push_back(cv::Scalar(70, 70, 70));
		_colors.push_back(cv::Scalar(102, 102, 156));
		_colors.push_back(cv::Scalar(190, 153, 153));
		_colors.push_back(cv::Scalar(153, 153, 153));
		_colors.push_back(cv::Scalar(250, 170, 30));
		_colors.push_back(cv::Scalar(220, 220, 0));
		_colors.push_back(cv::Scalar(107, 142, 35));
		_colors.push_back(cv::Scalar(152, 251, 152));
		_colors.push_back(cv::Scalar(70, 130, 180));
		_colors.push_back(cv::Scalar(220, 20, 60));
		_colors.push_back(cv::Scalar(255, 0, 0));
		_colors.push_back(cv::Scalar(0, 0, 142));
		_colors.push_back(cv::Scalar(0, 0, 70));
		_colors.push_back(cv::Scalar(0, 60, 100));
		_colors.push_back(cv::Scalar(0, 80, 100));
		_colors.push_back(cv::Scalar(0, 0, 230));
		_colors.push_back(cv::Scalar(119, 11, 32));
		_colors.push_back(cv::Scalar(0, 0, 0)); // unknown class
		_colors.push_back(cv::Scalar(255, 255, 255)); // lane marks
	}

	cv::Scalar color(int index)
	{
		return _colors[index % _colors.size()];
	}
};

cv::Mat segmented_image_view(cv::Mat &m);

double normalize_theta(double theta);
double radians_to_degrees(double theta);
double degrees_to_radians(double theta);

Eigen::Matrix<double, 4, 4> pose6d_to_matrix(double x, double y, double z, double roll, double pitch, double yaw);
Eigen::Matrix<double, 4, 4> pose3d_to_matrix(double x, double y, double theta);

// pose[i] contains the transformation which takes a
// 3D pcl::Point in the i'th frame and projects it into the oxts
// coordinates of the first frame.
void oxts2Mercartor(std::vector<std::vector<double>> &data, std::vector<Eigen::Matrix<double, 4, 4>> &poses);

std::vector<double> read_vector(char *name);

// debug
void print_vector(std::vector<double> &v);

int argmax(double *v, int size);
int argmin(double *v, int size);

void draw_rectangle(cv::Mat &img,
		double x, double y, double theta,
		double height, double width, cv::Scalar color,
		double x_origin, double y_origin, double pixels_by_meter);

void print_poses(std::vector<Eigen::Matrix<double, 4, 4>> &poses);

double dist2d(double x1, double y1, double x2, double y2);

void ackerman_motion_model(double &x, double &y, double &th, double v, double phi, double dt);
void ackerman_motion_model(Pose2d &pose, double v, double phi, double dt);

void
correct_point(Pose2d &correction,
							Eigen::Matrix<double, 4, 4> vel2car,
		pcl::PointXYZRGB &point);

/*
void
transform_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud,
		Pose2d &pose,
		Matrix<double, 4, 4> &vel2car,
		double v, double phi);
*/

FILE* safe_fopen(const char *path, const char *mode);

void spherical2cartersian(double v_angle, double h_angle, double radius, 
						  double *x, double *y, double *z);

void cartersian2spherical(double x, double y, double z,
							double *v_angle, double *h_angle, double *radius);

std::vector<std::string> string_split(std::string s, std::string pattern);

std::string file_name_from_path(const char *path);
std::string default_odom_calib_path(const char *log_path);
std::string default_fused_odom_path(const char *log_path);

#endif
