
#include "gicp.h"
#include <pcl/registration/gicp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

using namespace Eigen;


void
run_gicp(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, 
	Matrix<double, 4, 4> *correction, 
	int *converged, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output,
	double leaf_size)
{
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_leafed(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_leafed(new pcl::PointCloud<pcl::PointXYZRGB>);

	gicp.setMaximumIterations(200);
	//gicp.setTransformationEpsilon(1e-5);
	gicp.setMaxCorrespondenceDistance(20.0);
	//gicp.setRANSACOutlierRejectionThreshold(0.02);

	if (leaf_size > 0.)
	{
		pcl::VoxelGrid<pcl::PointXYZRGB> grid;
		grid.setLeafSize(leaf_size, leaf_size, leaf_size);
		grid.setInputCloud(source);
		grid.filter(*source_leafed);
		grid.setInputCloud(target);
		grid.filter(*target_leafed);
	}
	else
	{
		source_leafed = source;
		target_leafed = target;
	}

	gicp.setInputCloud(source_leafed);
	gicp.setInputTarget(target_leafed);
	gicp.align(*output);

	*correction = gicp.getFinalTransformation().cast<double>();
	*converged = gicp.hasConverged();
}


