#ifndef CARMEN_VOSLAM_UTIL_H_
#define CARMEN_VOSLAM_UTIL_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>

#include <carmen/carmen.h>
#include <carmen/stereo_util.h>
#include <tf.h>
#include <Eigen/Core>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#define STEREO_RANGE_MAX 20.0

typedef struct {
	tf::Vector3 position;
	tf::Vector3 orientation;
	tf::Matrix3x3 rotation;
}carmen_voslam_pose_t;

typedef struct {
	std::vector<int> loop_partners_indexes;
	int pointcloud_size;
	carmen_voslam_pose_t pose;
	double v;
	double phi;
	unsigned short* depth;
	unsigned char* image;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud;
	double timestamp;
}carmen_voslam_pointcloud_t;

void apply_bilateral_filter(unsigned short* src, unsigned short* dest, stereo_util stereo_reprojection_params);
Eigen::Matrix<float, 4, 4> voslam_pointcloud_pose_to_eigen_transform(carmen_voslam_pose_t pose);
carmen_voslam_pose_t eigen_transform_to_voslam_pointcloud_pose(Eigen::Matrix<float, 4, 4> transform);
Eigen::Matrix<float, 4, 4> compute_pcl_transform_between_voslam_pointclouds(carmen_voslam_pointcloud_t *source_voslam_pointcloud, carmen_voslam_pointcloud_t *target_voslam_pointcloud);
void print_eigen_matrix(Eigen::Matrix<float, 4, 4>& matrix, char* name);
void convert_rgb_image_to_gray_image(unsigned char *rgb_image, unsigned char* gray_image, int width, int height);
void print_voslam_graph(std::vector<carmen_voslam_pointcloud_t> list);
void save_keyframe_pointclouds(std::vector<carmen_voslam_pointcloud_t> list);

#endif /* VOSLAM_UTIL_H_ */
