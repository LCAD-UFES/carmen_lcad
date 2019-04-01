
#ifndef __GICP_H__
#define __GICP_H__

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "g2o/types/slam2d/se2.h"
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_preproc.h>


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
search_for_loop_closure_using_pose_dist(NewCarmenDataset &dataset,
																				Pose2d reference_pose,
																				double reference_pose_time,
																				int from,
																				double max_dist_threshold,
																				double min_time_threshold,
																				int *nn_id);

void
run_gicp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source,
				 pcl::PointCloud<pcl::PointXYZRGB>::Ptr target,
				 Eigen::Matrix<double, 4, 4> *correction,
				 int *converged,
				 pcl::PointCloud<pcl::PointXYZRGB>::Ptr output,
				 double leaf_size=0.);

Eigen::Matrix<double, 4, 4>
compute_source2target_transform(Pose2d target_pose, Pose2d source_pose);

void
run_icp_step(DataSample *target_sample,
						 DataSample *source_sample,
						 Eigen::Matrix<double, 4, 4> *relative_transform,
						 int *convergence_flag,
						 SensorPreproc &target_preproc,
						 SensorPreproc &source_preproc,
						 bool view = false);


#endif

