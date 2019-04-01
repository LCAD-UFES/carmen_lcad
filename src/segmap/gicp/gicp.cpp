
#include "gicp.h"
#include <pcl/registration/gicp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_preproc.h>
#include <carmen/segmap_sensor_viewer.h>
#include <carmen/segmap_conversions.h>

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
search_for_loop_closure_using_pose_dist(NewCarmenDataset &dataset,
																				Pose2d reference_pose,
																				double reference_pose_time,
																				int from,
																				double max_dist_threshold,
																				double min_time_threshold,
																				int *nn_id)
{
	*nn_id = -1;
	double min_dist = DBL_MAX;

	for (int j = from; j < dataset.size(); j++)
	{
		double dx = reference_pose.x - dataset[j]->pose.x;
		double dy = reference_pose.y - dataset[j]->pose.y;
		double dt = fabs(reference_pose_time - dataset[j]->time);

		double dist = sqrt(pow(dx, 2) + pow(dy, 2));

		if ((dist < min_dist) && (dist < max_dist_threshold) && (dt > min_time_threshold))
		{
			min_dist = dist;
			(*nn_id) = j;
		}
	}
}


Matrix<double, 4, 4>
compute_source2target_transform(Pose2d target_pose, Pose2d source_pose)
{
	source_pose.x -= target_pose.x;
	source_pose.y -= target_pose.y;

	Matrix<double, 4, 4> world2target = pose3d_to_matrix(0., 0., target_pose.th).inverse();
	Matrix<double, 4, 4> source2world = Pose2d::to_matrix(source_pose);
	Matrix<double, 4, 4> source2target = world2target * source2world;

	return source2target;
}


void
run_icp_step(DataSample *target_sample,
						 DataSample *source_sample,
						 Matrix<double, 4, 4> *relative_transform,
						 int *convergence_flag,
						 SensorPreproc &target_preproc,
						 SensorPreproc &source_preproc,
						 bool view)
{
	Matrix<double, 4, 4> source2target;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZRGB>);

	target_preproc.reinitialize(target_sample);
	load_as_pointcloud(target_preproc, target, SensorPreproc::CAR_REFERENCE);

	source_preproc.reinitialize(source_sample);
	load_as_pointcloud(source_preproc, source, SensorPreproc::CAR_REFERENCE);

	source2target = compute_source2target_transform(target_sample->pose,
																									source_sample->pose);

	pcl::transformPointCloud(*source, *source, source2target);
	run_gicp(source, target, relative_transform, convergence_flag, aligned, 0.1);

	if (view)
	{
		static PointCloudViewer *viewer = NULL;

		if (viewer == NULL)
			viewer = new PointCloudViewer(3);

		viewer->clear();
		viewer->show(target, 0, 1, 0);
		viewer->show(source, 1, 0, 0);
		viewer->show(aligned, 0, 0, 1);
		viewer->loop();
	}
}

