#ifndef SCAN_REGISTRATION_H
#define SCAN_REGISTRATION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include "common.h"

std::vector<pcl::PointCloud<PointType>> laserCloudHandler(pcl::PointCloud<PointType> laserCloudMsg);



#endif

