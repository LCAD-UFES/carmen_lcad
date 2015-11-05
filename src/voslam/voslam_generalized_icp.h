#ifndef VOSLAM_GENERALIZED_ICP_H_
#define VOSLAM_GENERALIZED_ICP_H_


#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "voslam_util.h"

class VoslamGeneralizedICP {

private:
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	pcl::VoxelGrid<pcl::PointXYZRGB> grid;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_pcl_pointcloud_sampled;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_pcl_pointcloud_sampled;
	pcl::PointCloud<pcl::PointXYZRGB> out_pcl_pointcloud;

public:
	void initializeGeneralizedICP(float leaf_sample_size, float max_iterations, float max_distance, float transformation_epsilon);
	double runGeneralizedICP(carmen_voslam_pointcloud_t *source_voslam_pointcloud, carmen_voslam_pointcloud_t *target_voslam_pointcloud, Eigen::Matrix<float, 4, 4> *guess);
	int runICP(carmen_voslam_pointcloud_t *source_voslam_pointcloud, carmen_voslam_pointcloud_t *target_voslam_pointcloud, Eigen::Matrix<float, 4, 4> *guess);
	void show_pointclouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt);

	VoslamGeneralizedICP(float leaf_sample_size, float max_iterations, float max_distance, float transformation_epsilon);
	virtual ~VoslamGeneralizedICP();
};

#endif /* VOSLAM_GENERALIZED_ICP_H_ */
