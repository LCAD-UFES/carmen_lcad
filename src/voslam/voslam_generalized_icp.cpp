#include "voslam_generalized_icp.h"

VoslamGeneralizedICP::VoslamGeneralizedICP(float leaf_sample_size, float max_iterations, float max_distance, float transformation_epsilon) {
	this->initializeGeneralizedICP(leaf_sample_size, max_iterations, max_distance, transformation_epsilon);
}

VoslamGeneralizedICP::~VoslamGeneralizedICP() {
	// TODO Auto-generated destructor stub
}

void VoslamGeneralizedICP::initializeGeneralizedICP(float leaf_sample_size, float max_iterations, float max_distance, float transformation_epsilon)
{
	this->grid.setLeafSize(leaf_sample_size, leaf_sample_size, leaf_sample_size);

	this->gicp.setMaximumIterations(max_iterations);
	this->gicp.setTransformationEpsilon(transformation_epsilon);
	this->gicp.setMaxCorrespondenceDistance(max_distance);

	this->icp.setMaximumIterations(max_iterations);
	this->icp.setTransformationEpsilon(1e-5);
	this->icp.setMaxCorrespondenceDistance(2.0);

	this->src_pcl_pointcloud_sampled = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);
	this->tgt_pcl_pointcloud_sampled = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);
}

double VoslamGeneralizedICP::runGeneralizedICP(carmen_voslam_pointcloud_t *source_voslam_pointcloud, carmen_voslam_pointcloud_t *target_voslam_pointcloud, Eigen::Matrix<float, 4, 4> *guess)
{
	Eigen::Matrix<float, 4, 4> out_transform;
	Eigen::Matrix<float, 4, 4> src_transform;
	Eigen::Matrix<float, 4, 4> final_transform;

	this->grid.setInputCloud(source_voslam_pointcloud->pointcloud);
	this->grid.filter(*this->src_pcl_pointcloud_sampled);

	this->grid.setInputCloud(target_voslam_pointcloud->pointcloud);
	this->grid.filter(*this->tgt_pcl_pointcloud_sampled);

	src_transform = voslam_pointcloud_pose_to_eigen_transform(source_voslam_pointcloud->pose);

	/**
	 * TODO:
	 *
	 * Em http://pointclouds.org/documentation/tutorials/iterative_closest_point.php
	 * esta escrito o seguinte texto:
	 *
	 * "icp.setInputCloud(cloud_in); sets cloud_in as the PointCloud to begin from and
	 * icp.setInputTarget(cloud_out); sets cloud_out as the PointCloud which we want
	 * cloud_in to look like."
	 *
	 * Como nos queremos alinhar uma nuvem no tempo (t + 1) com uma nuvem no tempo t,
	 * nos deveriamos fazer o align passando a nuvem do tempo (t + 1) como source e
	 * a nuvem do tempo t como target. Hoje, nos estamos fazendo o contrario. Ao mudar
	 * a reconstrucao visualmente ficou pior, entao eu mantive o codigo como esta, mas
	 * eh importante dar uma olhada nisso.
	 *
	 */

	this->gicp.setInputCloud(this->src_pcl_pointcloud_sampled);
	this->gicp.setInputTarget(this->tgt_pcl_pointcloud_sampled);

	if(guess)
		this->gicp.align(this->out_pcl_pointcloud, *guess);
	else
		this->gicp.align(this->out_pcl_pointcloud);

	out_transform = gicp.getFinalTransformation();

	final_transform = out_transform.inverse() * voslam_pointcloud_pose_to_eigen_transform(target_voslam_pointcloud->pose);
	Eigen::Matrix<float, 4, 4> out_transform_inv = out_transform.inverse();

	target_voslam_pointcloud->pose = eigen_transform_to_voslam_pointcloud_pose(final_transform);
	pcl::transformPointCloud(*target_voslam_pointcloud->pointcloud, *target_voslam_pointcloud->pointcloud, out_transform_inv);

	if(gicp.hasConverged())
	{
//		std::cout << "Fitness: " << gicp.getFitnessScore() << "\t";
		return gicp.getFitnessScore();
	}

	return -1;
}

int VoslamGeneralizedICP::runICP(carmen_voslam_pointcloud_t *source_voslam_pointcloud, carmen_voslam_pointcloud_t *target_voslam_pointcloud, Eigen::Matrix<float, 4, 4> *guess)
{
	Eigen::Matrix<float, 4, 4> out_transform;
	Eigen::Matrix<float, 4, 4> src_transform;
	Eigen::Matrix<float, 4, 4> final_transform;

	this->grid.setInputCloud(source_voslam_pointcloud->pointcloud);
	this->grid.filter(*this->src_pcl_pointcloud_sampled);

	this->grid.setInputCloud(target_voslam_pointcloud->pointcloud);
	this->grid.filter(*this->tgt_pcl_pointcloud_sampled);

	src_transform = voslam_pointcloud_pose_to_eigen_transform(source_voslam_pointcloud->pose);

	this->icp.setInputCloud(this->src_pcl_pointcloud_sampled);
	this->icp.setInputTarget(this->tgt_pcl_pointcloud_sampled);

	if(guess)
		this->icp.align(this->out_pcl_pointcloud, *guess);
	else
		this->icp.align(this->out_pcl_pointcloud);

	out_transform = this->icp.getFinalTransformation();

	final_transform = out_transform * src_transform;

	source_voslam_pointcloud->pose = eigen_transform_to_voslam_pointcloud_pose(final_transform);
	pcl::transformPointCloud(*source_voslam_pointcloud->pointcloud, *source_voslam_pointcloud->pointcloud, out_transform);

	if(icp.hasConverged())
		return 1;

	return 0;
}


void VoslamGeneralizedICP::show_pointclouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt)
{
	pcl::visualization::PCLVisualizer viewer ("Cloud Viewer");

	viewer.setBackgroundColor (0, 0, 0);

	viewer.addPointCloud<pcl::PointXYZRGB> (src, "Source Cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0 , 0, "Source Cloud");

	viewer.addPointCloud<pcl::PointXYZRGB> (tgt, "Target Cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0 , 1.0, "Target Cloud");

	viewer.addCoordinateSystem (1.0);
	viewer.initCameraParameters ();

	while (!viewer.wasStopped ()){
		    viewer.spinOnce (100);
	};
}
