
#include <cstdlib>
#include <iostream>

#include "icp_processor.h"


struct icp_processor
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2;
};

icp_processor* create_icp_processor(void)
{	
	icp_processor* icp = (icp_processor*)calloc(1,sizeof(icp_processor));

	icp->cloud_1 = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	icp->cloud_2 = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);

	return icp;
}

void set_icp_cloud_1(icp_processor* icp, carmen_vector_3D_t* points, int num_points, int downsample)
{	
	icp->cloud_1->width    = num_points/downsample;
	icp->cloud_1->height   = 1;
	icp->cloud_1->is_dense = false;
	icp->cloud_1->points.resize (icp->cloud_1->width * icp->cloud_1->height);
	
	for (size_t i = 0; i < icp->cloud_1->points.size(); i++)
	{
		icp->cloud_1->points[i].x = points[i*downsample].x;
		icp->cloud_1->points[i].y = points[i*downsample].y;
		icp->cloud_1->points[i].z = points[i*downsample].z;
	}
}

void add_to_cloud_1(icp_processor* icp, carmen_vector_3D_t* points, int num_points, int downsample)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	tmp_cloud->width    = num_points/downsample;
	tmp_cloud->height   = 1;
	tmp_cloud->is_dense = false;
	tmp_cloud->points.resize (tmp_cloud->width * tmp_cloud->height);
	
	for (size_t i = 0; i < tmp_cloud->points.size(); i++)
	{
		tmp_cloud->points[i].x = points[i*downsample].x;
		tmp_cloud->points[i].y = points[i*downsample].y;
		tmp_cloud->points[i].z = points[i*downsample].z;
	}

	*(icp->cloud_1) += *tmp_cloud;
}

void set_icp_cloud_2(icp_processor* icp, carmen_vector_3D_t* points, int num_points, int downsample)
{
	// Fill in the CloudIn data
	icp->cloud_2->width    = num_points/downsample;
	icp->cloud_2->height   = 1;
	icp->cloud_2->is_dense = false;
	icp->cloud_2->points.resize (icp->cloud_2->width * icp->cloud_2->height);
	
	for (size_t i = 0; i < icp->cloud_2->points.size(); i++)
	{
		icp->cloud_2->points[i].x = points[i*downsample].x;
		icp->cloud_2->points[i].y = points[i*downsample].y;
		icp->cloud_2->points[i].z = points[i*downsample].z;
	}	
}

void add_to_cloud_2(icp_processor* icp, carmen_vector_3D_t* points, int num_points, int downsample)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	tmp_cloud->width    = num_points/downsample;
	tmp_cloud->height   = 1;
	tmp_cloud->is_dense = false;
	tmp_cloud->points.resize (tmp_cloud->width * tmp_cloud->height);
	
	for (size_t i = 0; i < tmp_cloud->points.size(); i++)
	{
		tmp_cloud->points[i].x = points[i*downsample].x;
		tmp_cloud->points[i].y = points[i*downsample].y;
		tmp_cloud->points[i].z = points[i*downsample].z;
	}

	*(icp->cloud_2) += *tmp_cloud;
}

void get_cloud_transformation(icp_processor* icp_p, Eigen::Matrix4f transform_guess)
{
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	printf("%d ", icp_p->cloud_1->points.size());	
	printf("%d\n", icp_p->cloud_2->points.size());

	icp.setInputCloud(icp_p->cloud_1);
	icp.setInputTarget(icp_p->cloud_2);

	pcl::PointCloud<pcl::PointXYZ> Final;
	
	icp.setMaxCorrespondenceDistance(0.1);
	icp.setMaximumIterations(5);
	double timeStart = carmen_get_time();
	//icp.align(Final, transform_guess);
	icp.align(Final);
	double timeEnd = carmen_get_time();

	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << " time: " << (timeEnd - timeStart) << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
}


void destroy_icp_processor(icp_processor* icp)
{
	free(icp);
}
