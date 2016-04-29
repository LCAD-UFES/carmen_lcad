#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>
#include<vector>
#include <utility> 
#include <string>
#include <string.h>
#include <dirent.h>


std::pair<int,float> classifier_point_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


#endif
