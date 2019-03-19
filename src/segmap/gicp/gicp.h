
#ifndef __GICP_H__
#define __GICP_H__

#include <Eigen/Core>
#include <pcl/registration/gicp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include "g2o/types/slam2d/se2.h"


class LoopRestriction
{
	public:
		g2o::SE2 transform;
		int converged;
		int from;
		int to;
};


class LoopClosuresConfig
{
	public:
		char log_file[256];
		char fused_odom_file[256];
		char odom_calib_file[256];
         
};


void
run_gicp(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, 
	Eigen::Matrix<double, 4, 4> *correction,
	int *converged, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output,
	double leaf_size=0.);


#endif

