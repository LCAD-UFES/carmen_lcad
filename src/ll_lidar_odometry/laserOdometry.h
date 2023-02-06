#ifndef LASER_ODOMETRY_H
#define LASER_ODOMETRY_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include "common.h"

#ifdef __cplusplus
extern "C"
{
#endif

std::pair<carmen_vector_3D_t, carmen_quaternion_t> process_odom(std::vector<pcl::PointCloud<PointType>> vector_cloud_in, std::vector<pcl::PointCloud<PointType>> &vector_cloud_out, int frameCount);

#ifdef __cplusplus
}
#endif

#endif

