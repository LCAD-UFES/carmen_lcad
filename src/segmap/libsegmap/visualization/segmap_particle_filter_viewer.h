
#ifndef __PARTICLE_FILTER_VIEWER_H__
#define __PARTICLE_FILTER_VIEWER_H__

#include <vector>
#include <Eigen/Core>
#include <opencv/cv.hpp>
#include <carmen/segmap_grid_map.h>
#include <carmen/segmap_particle_filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


void draw_rectangle(cv::Mat &img,
                    double x, double y, double theta,
                    double height, double width, cv::Scalar color,
                    double x_origin, double y_origin, double pixels_by_meter);

void
draw_pose(GridMap &map, cv::Mat &map_img, Pose2d &p, cv::Scalar color);

void
draw_pose(GridMap &map, cv::Mat &map_img, Eigen::Matrix<double, 4, 4> &pose, cv::Scalar color);

void
draw_poses(GridMap &map, cv::Mat &map_img, std::vector<Eigen::Matrix<double, 4, 4>> &poses, cv::Scalar color);

void
draw_particle(cv::Mat &map_img, Pose2d &p, GridMap &map, cv::Scalar color);

void
draw_pointcloud(cv::Mat &m, pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud,
                GridMap &map, int paint_points = 1, cv::Scalar color = cv::Scalar(0,0,0));

void
view(ParticleFilter &pf, GridMap &map, Pose2d current_pose,
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud, Eigen::Matrix<double, 4, 4> *vel2car,
     double v, double phi, cv::Mat *pf_view_img);

void
colorize_cloud_according_to_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

#endif
