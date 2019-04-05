
#ifndef __GICP_H__
#define __GICP_H__

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_preproc.h>
#include <carmen/command_line.h>


void
run_gicp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source,
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr target,
         Eigen::Matrix<double, 4, 4> *correction,
         int *converged,
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr output,
         double leaf_size=0.);


void
add_default_gicp_args(CommandLineArguments &args);


#endif

