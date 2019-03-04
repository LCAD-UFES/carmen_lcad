
#ifndef __GICP__
#define __GICP__

#include <pcl/registration/gicp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>

using namespace Eigen;

void
run_gicp(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, 
	Matrix<double, 4, 4> *correction, 
	int *converged, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output,
	double leaf_size=0.);

#endif

