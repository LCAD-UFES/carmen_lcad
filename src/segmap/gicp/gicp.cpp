
#include "gicp.h"
#include <string>
#include <pcl/registration/gicp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_preproc.h>
#include <carmen/segmap_sensor_viewer.h>
#include <carmen/segmap_conversions.h>
#include <carmen/command_line.h>
#include <carmen/util_math.h>

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


void
add_default_gicp_args(CommandLineArguments &args)
{
	args.add_positional<std::string>("output", "Path of the output file", 1);
	args.add<double>("voxel_size,x", "Size of voxels in voxel grid filter", 0.05);
	args.add<double>("loop_dist,d", "Maximum distance (in meters) to assume two poses form a loop closure", 2.0);
	args.add<double>("time_dist,t", "Minimum temporal difference (in seconds) to assume two poses form a loop closure (instead of being consecutive poses)", 60.0);
	args.add<int>("subsampling,s", "Number of data packages to skip when looking for loop closures (<= 1 for using all packages)", 0);
	args.add<std::string>("report_file,r", "Path to a file to save debug information", "/tmp/loop_closure_report.txt");
	args.add<double>("v_thresh", "Skip data packages with absolute velocity below this theshold", 1.0);
	args.add<double>("dist_to_accumulate", "Distance to accumulate clouds", 2.0);
}


