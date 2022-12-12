/*********************************************************
	---  Moving Objects Module ---
**********************************************************/

#include "moving_objects.h"

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>

#include <unistd.h>
#include <stdio.h>
#include <string>


std::list<color_palette_and_association_data_t> color_palette_and_association;
/* List of point clouds used for association */
std::list<object_point_cloud_data_t> list_previous_point_clouds;
std::vector<object_model_features_t> object_models;
std::vector<object_features_3d_t> object_models_3d;
int num_of_models;

int first_associate_object_point_clouds_flag = 1;

#ifdef AJUSTE
extern FILE * parametro;
#endif
///////////////////////////////////////////////////////////////////////////////////////////////


void
update_label_associate(std::list<object_point_cloud_data_t> &list_point_clouds)
{
	for (std::list<object_point_cloud_data_t>::iterator pit = list_point_clouds.begin(); pit != list_point_clouds.end(); pit++)
		pit->label_associate = 0;
}


void
set_association_list_point_clouds(std::list<object_point_cloud_data_t> &list_point_clouds)
{
	update_label_associate(list_point_clouds);
	list_previous_point_clouds = list_point_clouds;
}


std::list<object_point_cloud_data_t>
get_association_list_point_clouds()
{
	return list_previous_point_clouds;
}


void
set_list_color_palette_and_association()
{
	int variation = 15; // proportional to 255(r,g,b)
	int total_color = 255; //510;//3060; // proportional to 255(r,g,b)
	Eigen::Vector3f color_palette;
	int num_color = 0;
	int num;
	color_palette_and_association_data_t aux_color_palette_and_association;

	srand(time(NULL));
	num = (255/variation) * 3;

	for (int i = 0; i < (total_color/num); i++)
	{

		color_palette[0] = 255;
		color_palette[1] = 0;
		color_palette[2] = 0;
		for (int j = 0; j < (255/variation); j++)
		{
			aux_color_palette_and_association.color_palette[0] = color_palette[0];
			aux_color_palette_and_association.color_palette[1] = color_palette[1];
			aux_color_palette_and_association.color_palette[2] = color_palette[2];
			aux_color_palette_and_association.num_association = 0;
			aux_color_palette_and_association.num_color = num_color;//(total_color - num_color);
			color_palette[0] -= variation;
			color_palette[1] = (rand() % 255);
			color_palette[2] = (rand() % 255);
			color_palette_and_association.push_back(aux_color_palette_and_association);
			num_color++;
		}

		color_palette[0] = 0;
		color_palette[1] = 255;
		color_palette[2] = 0;
		for (int j = 0; j < (255/variation); j++)
		{
			aux_color_palette_and_association.color_palette[0] = color_palette[0];
			aux_color_palette_and_association.color_palette[1] = color_palette[1];
			aux_color_palette_and_association.color_palette[2] = color_palette[2];
			aux_color_palette_and_association.num_association = 0;
			aux_color_palette_and_association.num_color = num_color;//(total_color - num_color);
			color_palette[0] = (rand() % 255);
			color_palette[1] -= variation;
			color_palette[2] = (rand() % 255);
			color_palette_and_association.push_back(aux_color_palette_and_association);
			num_color++;
		}

		color_palette[0] = 0;
		color_palette[1] = 0;
		color_palette[2] = 255;
		for (int j = 0; j < (255/variation); j++)
		{
			aux_color_palette_and_association.color_palette[0] = color_palette[0];
			aux_color_palette_and_association.color_palette[1] = color_palette[1];
			aux_color_palette_and_association.color_palette[2] = color_palette[2];
			aux_color_palette_and_association.num_association = 0;
			aux_color_palette_and_association.num_color = num_color;//(total_color - num_color);
			color_palette[0] = (rand() % 255);
			color_palette[1] = (rand() % 255);
			color_palette[2] -= variation;
			color_palette_and_association.push_back(aux_color_palette_and_association);
			num_color++;
		}
	}
}


int
search_free_color()
{
	int num_color = -1;

	for (std::list<color_palette_and_association_data_t>::iterator it = color_palette_and_association.begin();
			it != color_palette_and_association.end(); it++)
	{
		if (it->num_association < 0)
		{
			printf("error: associate negative value \n");
			return num_color;
		}

		if (it->num_association == 0)
		{
			num_color = it->num_color;
			break;
		}
	}
	if (num_color == -1)
	{
		printf("error: number of point clouds greater than the number of colors \n");
		exit(1);
	}

	return num_color;
}


void
increment_color_palette_and_association(int num_color)
{
	for (std::list<color_palette_and_association_data_t>::iterator it = color_palette_and_association.begin();
			it != color_palette_and_association.end(); it++)
	{
		if (num_color == it->num_color)
		{
			it->num_association += 1;
			break;
		}
	}
}


void
decrement_color_palette_and_association(int num_color)
{

	for (std::list<color_palette_and_association_data_t>::iterator it = color_palette_and_association.begin();
			it != color_palette_and_association.end(); it++)
	{
		if (num_color == it->num_color)
		{
			it->num_association -= 1;
			break;
		}
	}
}


std::list<color_palette_and_association_data_t>
get_color_palette_and_association()
{
	return color_palette_and_association;
}


std::vector<pcl::PointIndices>
pcl_euclidean_cluster_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	std::vector<pcl::PointIndices> cluster_indices;

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);

	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.80); //Set the spatial cluster tolerance as a measure in the L2 Euclidean space.
	ec.setMinClusterSize(15);     //Set the minimum number of points that a cluster needs to contain in order to be considered valid.
	ec.setMaxClusterSize(20000);  //Set the maximum number of points that a cluster needs to contain in order to be considered valid.
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	if (cluster_indices.size() > 0)
	{
//		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
//		{
//			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
//
//			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
//				cloud_cluster->points.push_back(cloud->points[*pit]);
//
//			cloud_cluster->width = cloud_cluster->points.size();
//			cloud_cluster->height = 1;
//			cloud_cluster->is_dense = true;
//		}
	}
	else
	{
		printf(" Cluster indices <= 0 \n");
	}
	return cluster_indices;
}


void
subtract_global_pose_from_point_cloud(carmen_vector_3D_t *cloud, int cloud_length, carmen_pose_3D_t car_global_pose)
{
	for (int k = 0; k < cloud_length; k++)
	{
		cloud[k].x -= car_global_pose.position.x;
		cloud[k].y -= car_global_pose.position.y;
		cloud[k].z -= car_global_pose.position.z;
	}
}


void
sum_global_pose_from_point_cloud(carmen_vector_3D_t *cloud, int cloud_length, carmen_pose_3D_t car_global_pose)
{
	for (int k = 0; k < cloud_length; k++)
	{
		cloud[k].x += car_global_pose.position.x;
		cloud[k].y += car_global_pose.position.y;
		cloud[k].z += car_global_pose.position.z;
	}
}


void
convert_carmen_3D_point_to_pcl_point_xyz(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud_ptr, carmen_vector_3D_t point)
{
	pcl::PointXYZ pcl_point_3D;

	pcl_point_3D.x = point.x;
	pcl_point_3D.y = point.y;
	pcl_point_3D.z = point.z;

	pcl_point_cloud_ptr->push_back(pcl_point_3D);
}


void
carmen_add_pcl_PointCloud_point_to_vector_3D(pcl::PointXYZ pointcloud_point, carmen_vector_3D_t *pointcloud_vector, int pos)
{
	pointcloud_vector[pos].x = pointcloud_point.x;
	pointcloud_vector[pos].y = pointcloud_point.y;
	pointcloud_vector[pos].z = pointcloud_point.z;
}


void
carmen_convert_vector_3d_point_cloud_to_pcl_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_pcl,
		carmen_vector_3D_t *pointcloud_vector, int pointcloud_length)
{
	for (int k = 0; k < pointcloud_length; k++)
		convert_carmen_3D_point_to_pcl_point_xyz(pointcloud_pcl, pointcloud_vector[k]);
}


void
carmen_convert_pcl_point_cloud_to_vector_3d_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_pcl,
		carmen_vector_3D_t *pointcloud_vector, int pointcloud_length)
{
	for (int k = 0; k < pointcloud_length; k++)
		carmen_add_pcl_PointCloud_point_to_vector_3D(pointcloud_pcl->points[k], pointcloud_vector, k);
}


void
set_point_cloud_cluster_i_from_point_cloud(int i, pcl::PointCloud<pcl::PointXYZ>::Ptr aux_pcl_cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_pcl, std::vector<pcl::PointIndices> cluster_indices)
{
	for (std::vector<int>::const_iterator pit = cluster_indices[i].indices.begin(); pit != cluster_indices[i].indices.end(); pit++)
		aux_pcl_cloud->push_back(pointcloud_pcl->points[*pit]);

	aux_pcl_cloud->width = aux_pcl_cloud->points.size();
	aux_pcl_cloud->height = 1;
	aux_pcl_cloud->is_dense = true;
}


void
get_point_cloud_cluster_with_cluster_indices(pcl::PointCloud<pcl::PointXYZ>::Ptr aux_pcl_cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_pcl, pcl::PointIndices cluster_indices)
{
	for (std::vector<int>::const_iterator pit = cluster_indices.indices.begin(); pit != cluster_indices.indices.end(); pit++)
		aux_pcl_cloud->push_back(pointcloud_pcl->points[*pit]);

	aux_pcl_cloud->width = aux_pcl_cloud->points.size();
	aux_pcl_cloud->height = 1;
	aux_pcl_cloud->is_dense = true;
}


double
distance_between_3d_point_and_car_global_pose(Eigen::Vector4f point_3D, carmen_pose_3D_t car_global_pose,
		carmen_pose_3D_t associated_car_global_pose)
{
	carmen_vector_3D_t point_3D_global_pose;

	point_3D_global_pose.x = point_3D[0] + associated_car_global_pose.position.x;
	point_3D_global_pose.y = point_3D[1] + associated_car_global_pose.position.y;
	point_3D_global_pose.z = point_3D[2] + associated_car_global_pose.position.z;

	double distance = sqrt (pow((point_3D_global_pose.x - car_global_pose.position.x), 2) +
						    pow((point_3D_global_pose.y - car_global_pose.position.y), 2) +
						    pow((point_3D_global_pose.z - car_global_pose.position.z), 2));

	return distance;
}


void
exclude_unecessary_objects_from_point_clouds(std::list<object_point_cloud_data_t> &list_point_clouds,
		carmen_pose_3D_t car_global_pose)
{
	Eigen::Vector4f centroid;
	carmen_pose_3D_t associated_car_global_pose_point_cloud;


	for (std::list<object_point_cloud_data_t>::iterator it = list_point_clouds.begin(); it != list_point_clouds.end(); )
	{
		if (it->count_idle >= threshold_idle_count)
		{
			decrement_color_palette_and_association(it->num_color_associate);
			it = list_point_clouds.erase(it);
			continue;
		}
		centroid = it->centroid;
		associated_car_global_pose_point_cloud.position.x = it->car_global_pose.position.x;
		associated_car_global_pose_point_cloud.position.y = it->car_global_pose.position.y;
		associated_car_global_pose_point_cloud.position.z = it->car_global_pose.position.z;

		double distance = distance_between_3d_point_and_car_global_pose(centroid, car_global_pose, associated_car_global_pose_point_cloud);
		it->distance_object_pose_and_car_global_pose = distance;
		if (distance >= threshold_max_dist_from_car || distance <= threshold_min_dist_from_car)
		{
			decrement_color_palette_and_association(it->num_color_associate);
			it = list_point_clouds.erase(it);
			continue;
		}
		it++;
	}
}


void
include_unassociated_objects_point_clouds(std::list<object_point_cloud_data_t> &list_point_clouds,
		std::list<object_point_cloud_data_t> &list_current_point_clouds)
{
	object_point_cloud_data_t aux_objects_data;
	int num_color_associate;

	for (std::list<object_point_cloud_data_t>::iterator it = list_current_point_clouds.begin();
			it != list_current_point_clouds.end(); it++)
	{
		if (it->num_color_associate == -1)
		{
			num_color_associate = search_free_color();

//			aux_objects_data = (*it);
			aux_objects_data.num_color_associate = num_color_associate;
			aux_objects_data.delta_time_t_and_t_1 = 0.0;
			aux_objects_data.object_pose.position.x = it->centroid[0];
			aux_objects_data.object_pose.position.y = it->centroid[1];
			aux_objects_data.object_pose.position.z = it->centroid[2];
			aux_objects_data.count_idle = 0.0;
			aux_objects_data.point_cloud = it->point_cloud;
			aux_objects_data.label_associate = it->label_associate;
			aux_objects_data.car_global_pose = it->car_global_pose;
			aux_objects_data.centroid = it->centroid;
			aux_objects_data.orientation = it->orientation;
			aux_objects_data.distance_object_pose_and_car_global_pose = it->distance_object_pose_and_car_global_pose;
			aux_objects_data.geometric_model = it->geometric_model;
			aux_objects_data.geometry = it->geometry;
			aux_objects_data.linear_velocity = it->linear_velocity;
			aux_objects_data.timestamp = it->timestamp;
			aux_objects_data.mean_particle = it->mean_particle;
			aux_objects_data.model_features = it->model_features;

			list_point_clouds.push_back(aux_objects_data);

			increment_color_palette_and_association(num_color_associate);
		}
	}
}


bool
is_point_cloud_the_current_autonomous_vehicle(Eigen::Vector4f position, double length, double width, double height,
		double x_offset, double z_offset, double theta)
{
	/* Checks whether the point cloud belongs to the autonomous vehicle */
	double theta_rot = -theta;
	/* - Centroid must be relative to car_global_pose (not global!)
	 * - Rotate points only in x-y plan (no need to transform z for the meanwhile) */
	double x = double(position[0])*cos(theta_rot) - double(position[1])*sin(theta_rot);
	double y = double(position[0])*sin(theta_rot) + double(position[1])*cos(theta_rot);
	double z = double(position[2]);

	bool is_in_y = (fabs(y) < 1.2*width/2);
	bool is_in_x = (x > -1.2*(x_offset) && x < 1.2*(length - x_offset));
	bool is_in_z = (z >= 0.0 && z < 1.2*(height - z_offset));

	return (is_in_y && is_in_x && is_in_z);
}


void
flattern_points_in_z(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); ++it)
	{
		it->z = 0.0;
	}
}


void
get_geometry_and_center_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		object_geometry_t &point_cloud_geometry, Eigen::Vector4f &center_point_cloud)
{
	/* GETS MINIMUM BOUNDING BOX OF POINT CLOUD */
	//ref.: http://codextechnicanum.blogspot.com.br/2015/04/find-minimum-oriented-bounding-box-of.html
	//ref.: http://www.pcl-users.org/Finding-oriented-bounding-box-of-a-cloud-td4024616.html

	// Compute principal directions
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud, pcaCentroid);
//	flattern_points_in_z(cloud);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);

//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PCA<pcl::PointXYZ> pca;
//	pca.setInputCloud(cloud);
//	pca.project(*cloud, *cloudPCAprojection);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

	// Transform the original cloud to the origin where the principal components correspond to the axes.
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
//	projectionTransform.block<3,3>(0,0) = pca.getEigenVectors().transpose();
	projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);
	// Get the minimum and maximum points of the transformed cloud.
	pcl::PointXYZ minPoint, maxPoint;
	pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
	const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

	double d_x, d_y, d_z;

	d_x = maxPoint.x - minPoint.x;
	d_y = maxPoint.y - minPoint.y;
	d_z = maxPoint.z - minPoint.z;

	point_cloud_geometry.width  = fabs(d_x);
	point_cloud_geometry.length = fabs(d_z);
	point_cloud_geometry.height = fabs(d_y);

	const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

	center_point_cloud[0] = bboxTransform[0];
	center_point_cloud[1] = bboxTransform[1];
	center_point_cloud[2] = bboxTransform[2];

	// Final transform (only for visualization)
//	const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
//	const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

	// This viewer has 4 windows, but is only showing images in one of them as written here.
//	pcl::visualization::PCLVisualizer *visu;
//	visu = new pcl::visualization::PCLVisualizer("argc", "argv", "PlyViewer");
//	int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
//	visu->createViewPort (0.0, 0.5, 0.5, 1.0,  mesh_vp_1);
//	visu->createViewPort (0.5, 0.5, 1.0, 1.0,  mesh_vp_2);
//	visu->createViewPort (0.0, 0, 0.5, 0.5,  mesh_vp_3);
//	visu->createViewPort (0.5, 0, 1.0, 0.5, mesh_vp_4);
//	visu->addPointCloud(cloud, ColorHandlerXYZ(cloud, 30, 144, 255), "bboxedCloud", mesh_vp_3);
//	visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox", mesh_vp_3);
}


void
get_geom_and_center_by_moie(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		object_geometry_t &point_cloud_geometry, Eigen::Vector4f &center_point_cloud)
{
	// ref.: http://pointclouds.org/documentation/tutorials/moment_of_inertia.php
	/* Get geometry and center using moment of inertia */
	flattern_points_in_z(cloud);

	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();

	std::vector <float> moment_of_inertia;
	std::vector <float> eccentricity;
	pcl::PointXYZ min_point_AABB;
	pcl::PointXYZ max_point_AABB;
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center;

	feature_extractor.getMomentOfInertia(moment_of_inertia);
	feature_extractor.getEccentricity(eccentricity);
	feature_extractor.getAABB(min_point_AABB, max_point_AABB);
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	feature_extractor.getEigenValues(major_value, middle_value, minor_value);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	feature_extractor.getMassCenter(mass_center);

//	point_cloud_geometry.width = fabs(max_point_AABB.x - min_point_AABB.x);
//	point_cloud_geometry.length = fabs(max_point_AABB.y - min_point_AABB.y);
//	point_cloud_geometry.height = fabs(max_point_AABB.z - min_point_AABB.z);
	center_point_cloud[0] = 0.5f*(min_point_AABB.x + max_point_AABB.x);
	center_point_cloud[1] = 0.5f*(min_point_AABB.y + max_point_AABB.y);
	center_point_cloud[2] = 0.5f*(min_point_AABB.z + max_point_AABB.z);
	point_cloud_geometry.width = fabs(max_point_OBB.y - min_point_OBB.y);
	point_cloud_geometry.length = fabs(max_point_OBB.x - min_point_OBB.x);
	point_cloud_geometry.height = fabs(max_point_OBB.z - min_point_OBB.z);
//	center_point_cloud[0] = position_OBB.x;
//	center_point_cloud[1] = position_OBB.y;
//	center_point_cloud[2] = position_OBB.z;
//	center_point_cloud[0] = mass_center[0];
//	center_point_cloud[1] = mass_center[1];
//	center_point_cloud[2] = mass_center[2];

	/* VISUALIZING RESULTS */
//	// These lines simply create the instance of PCLVisualizer class for result visualization. Here we also add the cloud and the AABB for visualization.
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//	viewer->setBackgroundColor(0, 0, 0);
//	viewer->addCoordinateSystem(1.0);
//	viewer->initCameraParameters();
//	viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
//	viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
//
//	// Visualization of the OBB is little more complex. So here we create a quaternion from the rotational matrix, set OBBs position and pass it to the visualizer.
//	Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
//	Eigen::Quaternionf quat(rotational_matrix_OBB);
//	viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
//
//	// This lines are responsible for eigen vectors visualization.
////	pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
////	pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
////	pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
////	pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
//	pcl::PointXYZ center(position_OBB.x, position_OBB.y, position_OBB.z);
//	pcl::PointXYZ x_axis(major_vector(0) + position_OBB.x, major_vector(1) + position_OBB.y, major_vector(2) + position_OBB.z);
//	pcl::PointXYZ y_axis(middle_vector(0) + position_OBB.x, middle_vector(1) + position_OBB.y, middle_vector(2) + position_OBB.z);
//	pcl::PointXYZ z_axis(minor_vector(0) + position_OBB.x, minor_vector(1) + position_OBB.y, minor_vector(2) + position_OBB.z);
//	viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
//	viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
//	viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
//
//	while(!viewer->wasStopped())
//	{
//		viewer->spinOnce(100);
//		boost::this_thread::sleep(boost::posix_time::microseconds (100000));
//	}
}


void
get_current_list_point_clouds(std::list<object_point_cloud_data_t> &list_point_clouds,
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud_ptr,
		vector<pcl::PointIndices> cluster_indices, carmen_pose_3D_t car_global_pose, double timestamp)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr aux_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f centroid;
//	Eigen::Vector4f centroid2;
	object_point_cloud_data_t aux_objects_data;
	Eigen::Vector4f min_point;
	Eigen::Vector4f max_point;
	object_geometry_t point_cloud_geometry;

	std::vector<pcl::PointIndices>::const_iterator end = cluster_indices.end();

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != end; ++it)
	{
		get_point_cloud_cluster_with_cluster_indices(aux_pcl_cloud, pcl_point_cloud_ptr, *it);
		/*** PROPOSAL XY POSE OF POINT CLOUD ***/
//		pcl::compute3DCentroid(*aux_pcl_cloud, centroid2);
//		pcl::getMinMax3D(*aux_pcl_cloud, min_point, max_point);

//		double d_x = max_point[0] - min_point[0];
//		double d_y = max_point[1] - min_point[1];
//		double d_z = max_point[2] - min_point[2];
		//centroid[0] = 0.5f * (max_point[0] + min_point[0]);
		//centroid[1] = 0.5f * (max_point[1] + min_point[1]);
		//centroid[2] = 0.5f * (max_point[2] + min_point[2]);
		aux_objects_data.point_cloud = *aux_pcl_cloud;
		//get_geometry_and_center_point_cloud(aux_pcl_cloud, point_cloud_geometry, centroid);
		get_geom_and_center_by_moie(aux_pcl_cloud, point_cloud_geometry, centroid);
//		point_cloud_geometry.height = fabs(d_z);
//		centroid2[0] = 0.5f * (max_point[0] + min_point[0]);
//		centroid2[1] = 0.5f * (max_point[1] + min_point[1]);
//		centroid2[2] = 0.5f * (max_point[2] + min_point[2]);

//		if (is_point_cloud_the_current_autonomous_vehicle(centroid, 4.437, 2.065, 1.720, 0.909, 0.280, car_global_pose.orientation.yaw))
//			continue;

		aux_objects_data.distance_object_pose_and_car_global_pose = 0.0;
		aux_objects_data.num_color_associate    = -1; //-1 = not associated
		aux_objects_data.label_associate        = 0;
		aux_objects_data.car_global_pose        = car_global_pose;
		aux_objects_data.linear_velocity        = 0.0;
		aux_objects_data.delta_time_t_and_t_1   = 0.0;
		aux_objects_data.centroid               = centroid;
//		aux_objects_data.object_pose.position.x = centroid2[0];
//		aux_objects_data.object_pose.position.y = centroid2[1];
//		aux_objects_data.object_pose.position.z = centroid2[2];
		aux_objects_data.object_pose.position.x = centroid[0];
		aux_objects_data.object_pose.position.y = centroid[1];
		aux_objects_data.object_pose.position.z = centroid[2];
		//aux_objects_data.geometry.length = fabs(d_x);
		//aux_objects_data.geometry.width = fabs(d_y);
		//aux_objects_data.geometry.height = fabs(d_z);
		aux_objects_data.geometry.width = point_cloud_geometry.width;
		aux_objects_data.geometry.length = point_cloud_geometry.length;
		aux_objects_data.geometry.height = point_cloud_geometry.height;
		aux_objects_data.geometric_model = -1;
		aux_objects_data.model_features.model_id = -1;
		aux_objects_data.model_features.model_name = (char *)"";
		aux_objects_data.model_features.geometry.length = 0.0;
		aux_objects_data.model_features.geometry.width = 0.0;
		aux_objects_data.model_features.geometry.height = 0.0;
		aux_objects_data.model_features.red = 0.0;
		aux_objects_data.model_features.green = 0.0;
		aux_objects_data.model_features.blue = 0.0;
		aux_objects_data.orientation = 0.0;
		aux_objects_data.object_density = 0.0;
		aux_objects_data.count_idle	= 0;
		aux_objects_data.is_associated = false;
		aux_objects_data.timestamp = timestamp;

//		aux_objects_data.particle_set.resize(num_of_particles); //not necessary

		list_point_clouds.push_back(aux_objects_data);
		aux_pcl_cloud->clear();
	}

}


std::list<object_point_cloud_data_t>
get_current_list_point_clouds2(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud_ptr,
		vector<pcl::PointIndices> cluster_indices, carmen_pose_3D_t car_global_pose, double timestamp)
{
	std::list<object_point_cloud_data_t> list_point_clouds;
	pcl::PointCloud<pcl::PointXYZ>::Ptr aux_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f centroid;
	object_point_cloud_data_t aux_objects_data;
	Eigen::Vector4f min_point;
	Eigen::Vector4f max_point;
//	object_geometry_t point_cloud_geometry;

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		get_point_cloud_cluster_with_cluster_indices(aux_pcl_cloud, pcl_point_cloud_ptr, *it);
		/*** PROPOSAL XY POSE OF POINT CLOUD ***/
//		pcl::compute3DCentroid(*aux_pcl_cloud, centroid);
		pcl::getMinMax3D(*aux_pcl_cloud, min_point, max_point);

		double d_x = max_point[0] - min_point[0];
		double d_y = max_point[1] - min_point[1];
		double d_z = max_point[2] - min_point[2];
		centroid[0] = 0.5f * (max_point[0] + min_point[0]);
		centroid[1] = 0.5f * (max_point[1] + min_point[1]);
		centroid[2] = 0.5f * (max_point[2] + min_point[2]);

		aux_objects_data.point_cloud = *aux_pcl_cloud;
		aux_objects_data.distance_object_pose_and_car_global_pose = 0.0;
		aux_objects_data.num_color_associate = -1; //-1 = not associated
		aux_objects_data.label_associate = 0;
		aux_objects_data.car_global_pose = car_global_pose;
		aux_objects_data.linear_velocity = 0.0;
		aux_objects_data.delta_time_t_and_t_1 = 0.0;
		aux_objects_data.centroid = centroid;
		aux_objects_data.object_pose.position.x = centroid[0];
		aux_objects_data.object_pose.position.y = centroid[1];
		aux_objects_data.object_pose.position.z = centroid[2];
		aux_objects_data.geometry.length = fabs(d_x);
		aux_objects_data.geometry.width = fabs(d_y);
		aux_objects_data.geometry.height = fabs(d_z);
		aux_objects_data.geometric_model = -1;
		aux_objects_data.model_features.model_id = -1;
		aux_objects_data.model_features.model_name = (char *)"";
		aux_objects_data.model_features.geometry.length = 0.0;
		aux_objects_data.model_features.geometry.width = 0.0;
		aux_objects_data.model_features.geometry.height = 0.0;
		aux_objects_data.model_features.red = 0.0;
		aux_objects_data.model_features.green = 0.0;
		aux_objects_data.model_features.blue = 0.0;
		aux_objects_data.orientation = 0.0;
		aux_objects_data.object_density = 0.0;
		aux_objects_data.count_idle	= 0;
		aux_objects_data.is_associated = false;
		aux_objects_data.timestamp = timestamp;

		list_point_clouds.push_back(aux_objects_data);
		aux_pcl_cloud->clear();
	}

	return list_point_clouds;
}


double
distance_between_3d_points(Eigen::Vector4f current_point, carmen_pose_3D_t c_associated_car_global_pose_point_cloud,
		Eigen::Vector4f previous_point, carmen_pose_3D_t p_associated_car_global_pose_point_cloud)
{
	double d_x, d_y, d_z;

	carmen_vector_3D_t current_point_global_pose;
	carmen_vector_3D_t previous_point_global_pose;

	current_point_global_pose.x = current_point[0] + c_associated_car_global_pose_point_cloud.position.x;
	current_point_global_pose.y = current_point[1] + c_associated_car_global_pose_point_cloud.position.y;
	current_point_global_pose.z = current_point[2] + c_associated_car_global_pose_point_cloud.position.z;

	previous_point_global_pose.x = previous_point[0] + p_associated_car_global_pose_point_cloud.position.x;
	previous_point_global_pose.y = previous_point[1] + p_associated_car_global_pose_point_cloud.position.y;
	previous_point_global_pose.z = previous_point[2] + p_associated_car_global_pose_point_cloud.position.z;

	d_x = current_point_global_pose.x - previous_point_global_pose.x;
	d_y = current_point_global_pose.y - previous_point_global_pose.y;
	d_z = current_point_global_pose.z - previous_point_global_pose.z;

	return sqrt(d_x*d_x + d_y*d_y + d_z*d_z);
}


double
distance_between_2d_points(Eigen::Vector4f current_point, carmen_pose_3D_t current_car_global_pose,
		Eigen::Vector4f previous_point, carmen_pose_3D_t previous_car_global_pose)
{
	double d_x, d_y;

	carmen_vector_3D_t current_point_global_pose;
	carmen_vector_3D_t previous_point_global_pose;

	current_point_global_pose.x = current_point[0] + current_car_global_pose.position.x;
	current_point_global_pose.y = current_point[1] + current_car_global_pose.position.y;

	previous_point_global_pose.x = previous_point[0] + previous_car_global_pose.position.x;
	previous_point_global_pose.y = previous_point[1] + previous_car_global_pose.position.y;

	d_x = current_point_global_pose.x - previous_point_global_pose.x;
	d_y = current_point_global_pose.y - previous_point_global_pose.y;

	return sqrt(d_x*d_x + d_y*d_y);

}


double
calculate_velocity_of_the_object(double distance, double delta_time)
{
	double velocity;

	if (delta_time <= 0.0)
		velocity = 0.0;
	else
		velocity = (distance / delta_time);

	return velocity;
}


double
orientation_by_displacement_between_two_points(Eigen::Vector4f current_point,
		carmen_pose_3D_t c_associated_car_global_pose_point_cloud,
		Eigen::Vector4f previous_point, carmen_pose_3D_t p_associated_car_global_pose_point_cloud)
{
	double delta_x, delta_y, orientation;

	carmen_vector_3D_t current_point_global_pose, previous_point_global_pose;

	current_point_global_pose.x = current_point[0] + c_associated_car_global_pose_point_cloud.position.x;
	current_point_global_pose.y = current_point[1] + c_associated_car_global_pose_point_cloud.position.y;

	previous_point_global_pose.x = previous_point[0] + p_associated_car_global_pose_point_cloud.position.x;
	previous_point_global_pose.y = previous_point[1] + p_associated_car_global_pose_point_cloud.position.y;


	delta_x = current_point_global_pose.x - previous_point_global_pose.x;
	delta_y = current_point_global_pose.y - previous_point_global_pose.y;

	orientation = atan2(delta_y, delta_x); //returns in range [-pi,pi]

	return orientation;
}


void
init_particle_set(object_point_cloud_data_t &object_pcloud, int num_particles, double x, double y, int num_models)
{
	for (int i = 0; i < num_particles; i++)
	{
		particle_datmo_t particle_t_1;
		particle_t_1.pose.x = x;
		particle_t_1.pose.y = y;

		// Inicializa 50% das vezes aleatório e 50% com os dados do centroid
		if ((rand() % 100) <= 50)
		{
			particle_t_1.pose.theta = carmen_uniform_random(-M_PI, M_PI);
			particle_t_1.velocity = carmen_uniform_random(0.0, 25.0);
		} else
		{
			particle_t_1.pose.theta = carmen_normalize_theta(object_pcloud.orientation + carmen_uniform_random(-M_PI/6, M_PI/6));
			particle_t_1.velocity = object_pcloud.linear_velocity + carmen_uniform_random(-5.0, 5.0);
		}

		//particle_t_1.weight = (1.0 / double(num_of_particles)); //not necessary
		particle_t_1.class_id = get_random_model_id(num_models);
		particle_t_1.model_features = get_obj_model_features(particle_t_1.class_id);
		//particle_t_1.model_features_3d = get_obj_model_features_3d(particle_t_1.class_id, object_models_3d);
		particle_t_1.timestamp = object_pcloud.timestamp;

		object_pcloud.particle_set.push_back(particle_t_1);
	}
}


void
associate_point_clouds_by_centroids_distance(std::list<object_point_cloud_data_t> &list_point_clouds,
		std::list<object_point_cloud_data_t> &list_current_point_clouds)
{
	double distance_3d;
	double distance_2d;
	object_point_cloud_data_t aux_objects_data;
	double delta_time;

	for (std::list<object_point_cloud_data_t>::iterator it = list_current_point_clouds.begin();
			it != list_current_point_clouds.end(); it++)
	{
		double min_dist = 99999.0;
		int min_dist_reading = -1;

		distance_2d = 99999.0;

		for (std::list<object_point_cloud_data_t>::iterator pit = list_point_clouds.begin(); pit != list_point_clouds.end(); pit++)
		{
			if  (pit->label_associate == 0)
			{
				distance_3d = distance_between_3d_points(it->centroid, it->car_global_pose, pit->centroid, pit->car_global_pose);
				if (distance_3d < min_dist)
				{
					pit->delta_time_t_and_t_1 = -1.0;
					min_dist = distance_3d;
					min_dist_reading = pit->num_color_associate;
					distance_2d = distance_between_2d_points(it->centroid, it->car_global_pose, pit->centroid, pit->car_global_pose);
				}
			}
		}
		if (distance_2d < threshold_association_dist)
		{
			for (std::list<object_point_cloud_data_t>::iterator pit = list_point_clouds.begin(); pit != list_point_clouds.end(); pit++)
			{
				if (pit->num_color_associate == min_dist_reading)
				{
					delta_time = (it->timestamp - pit->timestamp);

					pit->label_associate = 1;
					pit->delta_time_t_and_t_1 = delta_time;
					pit->linear_velocity = calculate_velocity_of_the_object(distance_2d, delta_time);
					pit->orientation = orientation_by_displacement_between_two_points(it->centroid, it->car_global_pose, pit->centroid, pit->car_global_pose);
					pit->car_global_pose = it->car_global_pose;
					pit->centroid = it->centroid;
					pit->point_cloud = it->point_cloud;
					pit->distance_object_pose_and_car_global_pose = it->distance_object_pose_and_car_global_pose;
					pit->geometric_model = it->geometric_model;
					pit->geometry = it->geometry;
					pit->object_density = it->object_density;
					pit->mean_particle = it->mean_particle;
					pit->timestamp = it->timestamp;
					pit->object_pose.position.x = it->centroid[0];
					pit->object_pose.position.y = it->centroid[1];
					pit->object_pose.position.z = it->centroid[2];
					pit->is_associated = true;
					it->num_color_associate = min_dist_reading;
					if (pit->particle_set.empty())
					{
						init_particle_set(*pit, num_of_particles,
								pit->object_pose.position.x + pit->car_global_pose.position.x,
								pit->object_pose.position.y + pit->car_global_pose.position.y,
								num_of_models);
					}
				}
			}
		}
		else
			it->num_color_associate = -1;
	}

	/* verify who's idle... */
	for (std::list<object_point_cloud_data_t>::iterator it = list_point_clouds.begin(); it != list_point_clouds.end(); it++)
	{
		if (it->label_associate == 0 /*|| it->linear_velocity < threshold_min_velocity*/)
		{
			it->count_idle++;
		}
	}
}


void
filter_curbs_point_cloud(std::list<object_point_cloud_data_t> &list_current_point_clouds)
{
	double sum;
	double average_height;
	double variance;
	double z;
	double threshold_average_height = 0.5;
	double threshold_variance = 0.02;

	for (std::list<object_point_cloud_data_t>::iterator it = list_current_point_clouds.begin(); it != list_current_point_clouds.end();)
	{
		sum = 0;
		for (pcl::PointCloud<pcl::PointXYZ>::iterator pit = it->point_cloud.begin(); pit != it->point_cloud.end(); pit++)
		{
			sum += pit->z;
		}
		average_height = (sum / it->point_cloud.size());

		sum = 0;
		for (pcl::PointCloud<pcl::PointXYZ>::iterator pit = it->point_cloud.begin(); pit != it->point_cloud.end(); pit++)
		{
			z = pit->z;
			sum += (z - average_height) * (z - average_height);
		}
		variance = (sum / it->point_cloud.size());

		if ((average_height < threshold_average_height) && (variance < threshold_variance))
		{
			it = list_current_point_clouds.erase(it);
		}
		else
		{
			it++;
		}
	}
}


void
filter_static_objects(std::list<object_point_cloud_data_t> &list_current_point_clouds, carmen_map_p & occupancy_grid_map,
		moving_objects_input_data_t moving_objects_input)
{
	/* Check if global position is set */
	if (moving_objects_input.first_offline_map_message != -1)
		return;

	if (occupancy_grid_map == NULL)
		return;

	double map_resolution = occupancy_grid_map->config.resolution;
	double map_x_origin = occupancy_grid_map->config.x_origin;
	double map_y_origin = occupancy_grid_map->config.y_origin;
	int map_x_size = occupancy_grid_map->config.x_size;
	int map_y_size = occupancy_grid_map->config.y_size;

	for (std::list<object_point_cloud_data_t>::iterator it = list_current_point_clouds.begin(); it != list_current_point_clouds.end();)
	{
		int points_in_occupied_grid = 0;
		bool off_limits = false;
		for (pcl::PointCloud<pcl::PointXYZ>::iterator pit = it->point_cloud.begin(); pit != it->point_cloud.end(); pit++)
		{
			int x = carmen_round((pit->x + it->car_global_pose.position.x - map_x_origin)/map_resolution);
			int y = carmen_round((pit->y + it->car_global_pose.position.y - map_y_origin)/map_resolution);

			/* Check point cloud off limits of gridmap */
			if (x < 0 || y < 0 || x >= map_x_size || y >= map_y_size)
			{
				off_limits = true;
				break;
			}

			double occupancy_rate = occupancy_grid_map->map[x][y];

			if (occupancy_rate >= threshold_occupancy_rate)
				points_in_occupied_grid++;
		}

		//if (((double)points_in_occupied_grid)/((double)it->point_cloud.size()) >= threshold_points_in_occupied_grid_rate)
		if (points_in_occupied_grid > 1)
		{
			it = list_current_point_clouds.erase(it);
			continue;
		}
		if (off_limits)
		{
			it = list_current_point_clouds.erase(it);
			continue;
		}
		it++;
	}
}

void
filter_static_points(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr, carmen_map_p & occupancy_grid_map,
		moving_objects_input_data_t moving_objects_input)
{

	/* Check if global position is set */
	if (moving_objects_input.first_offline_map_message != -1)
		return;

	if (occupancy_grid_map == NULL)
		return;

	double map_resolution = occupancy_grid_map->config.resolution;
	double map_x_origin = occupancy_grid_map->config.x_origin;
	double map_y_origin = occupancy_grid_map->config.y_origin;
	int map_x_size = occupancy_grid_map->config.x_size;
	int map_y_size = occupancy_grid_map->config.y_size;

	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = pcl_cloud_ptr->points.begin(); it != pcl_cloud_ptr->points.end(); )
	{
		int x = carmen_round((it->x + moving_objects_input.car_global_pose.position.x - map_x_origin)/map_resolution);
		int y = carmen_round((it->y + moving_objects_input.car_global_pose.position.y - map_y_origin)/map_resolution);

		/* Check point cloud off limits of gridmap */
		if (x < 0 || y < 0 || x >= map_x_size || y >= map_y_size)
		{
			it = pcl_cloud_ptr->points.erase(it);
			continue;
		}

		double occupancy_rate = occupancy_grid_map->map[x][y];

		if (occupancy_rate >= threshold_occupancy_rate)
			it = pcl_cloud_ptr->points.erase(it);
		else
			it++;
	}


}

void
associate_object_point_clouds(std::list<object_point_cloud_data_t> &list_point_clouds,
		std::list<object_point_cloud_data_t> &list_current_point_clouds, carmen_map_p &occupancy_grid_map,
		moving_objects_input_data_t moving_objects_input)
{
	//filter_curbs_point_cloud(list_current_point_clouds);
	//filter_static_objects(list_current_point_clouds, occupancy_grid_map, moving_objects_input);
	if (first_associate_object_point_clouds_flag)
	{
		set_list_color_palette_and_association();
		first_associate_object_point_clouds_flag = 0;
	}
	else
	{
		list_point_clouds = get_association_list_point_clouds();
		associate_point_clouds_by_centroids_distance(list_point_clouds, list_current_point_clouds);
	}
}


void
association_list_point_clouds(std::list<object_point_cloud_data_t> &list_point_clouds,
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr, carmen_pose_3D_t car_global_pose,
		std::vector<pcl::PointIndices> cluster_indices, double timestamp, carmen_map_p & occupancy_grid_map,
		moving_objects_input_data_t moving_objects_input)
{
	std::list<object_point_cloud_data_t> list_current_point_clouds;
	//std::list<object_point_cloud_data_t> list_point_clouds;

//	get_current_list_point_clouds(list_current_point_clouds, pcl_cloud_ptr, cluster_indices, car_global_pose, timestamp);
	/*** VERSÃO DO EDUARDO: ***/
	list_current_point_clouds = get_current_list_point_clouds2(pcl_cloud_ptr, cluster_indices, car_global_pose, timestamp);
	associate_object_point_clouds(list_point_clouds, list_current_point_clouds, occupancy_grid_map, moving_objects_input);
	include_unassociated_objects_point_clouds(list_point_clouds, list_current_point_clouds);
	exclude_unecessary_objects_from_point_clouds(list_point_clouds, car_global_pose);

}


std::vector<pcl::PointIndices>
find_objects_in_point_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr)
{
	std::vector<pcl::PointIndices> cluster_indices;
	if (!pcl_cloud_ptr->points.empty())
	{
		cluster_indices = pcl_euclidean_cluster_extraction(pcl_cloud_ptr);
	}

	return cluster_indices;
}


void
set_list_point_clouds_geometry_and_pose(std::list<object_point_cloud_data_t> &list_point_clouds)
{
	Eigen::Vector4f min_point;
	Eigen::Vector4f max_point;
	double d_x, d_y, d_z;

	for (std::list<object_point_cloud_data_t>::iterator it = list_point_clouds.begin(); it != list_point_clouds.end(); it++)
	{
		pcl::getMinMax3D(it->point_cloud, min_point, max_point);
		d_x = (max_point[0] + it->car_global_pose.position.x) - (min_point[0] + it->car_global_pose.position.x);
		d_y = (max_point[1] + it->car_global_pose.position.y) - (min_point[1] + it->car_global_pose.position.y);
		d_z = (max_point[2] + it->car_global_pose.position.z) - (min_point[2] + it->car_global_pose.position.z);

		it->object_pose.position.x = it->centroid[0];//(max_point[0] + min_point[0]) / 2.0;
		it->object_pose.position.y = it->centroid[1];//(max_point[1] + min_point[1]) / 2.0;
		it->object_pose.position.z = it->centroid[2];//(max_point[2] + min_point[2]) / 2.0;

		it->geometry.length = fabs(d_x);
		it->geometry.width = fabs(d_y);
		it->geometry.height = fabs(d_z);
	}
}


void
set_orientation_from_eigen_vectors(std::list<object_point_cloud_data_t> &list_point_clouds)
{
	Eigen::Matrix3f covariance_matrix;
	Eigen::Vector4f xyz_centroid;
	Eigen::Vector3f eigen_values;
	Eigen::Matrix3f eigen_vectors;
	double yaw;

	for (std::list<object_point_cloud_data_t>::iterator it = list_point_clouds.begin(); it != list_point_clouds.end(); it++)
	{
		xyz_centroid = it->centroid;
//		pcl::compute3DCentroid(it->point_cloud, xyz_centroid);

//		pcl::computeCovarianceMatrix(it->point_cloud, xyz_centroid, covariance_matrix);
		pcl::computeCovarianceMatrixNormalized(it->point_cloud, xyz_centroid, covariance_matrix);
		pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);

		// calculate angle of orientation
		yaw = atan2 ((double) eigen_vectors (1,2), (double) eigen_vectors (0,2));
		it->orientation = yaw;
	}
}


void
set_object_density_from_number_of_points_by_volume(std::list<object_point_cloud_data_t> &list_point_clouds)
{
	double density;
	int num_points;
	double volume;

	for (std::list<object_point_cloud_data_t>::iterator it = list_point_clouds.begin(); it != list_point_clouds.end(); it++)
	{
		num_points = it->point_cloud.size();
		volume = it->geometry.width * it->geometry.length * it->geometry.height;
		density = (num_points / volume);
		it->object_density = density;
	}
}


void
set_list_poins_clouds_density_number_of_points_by_area(std::list<object_point_cloud_data_t> &list_point_clouds)
{
	double density;
	int num_points;
	double area;

	for (std::list<object_point_cloud_data_t>::iterator it = list_point_clouds.begin(); it != list_point_clouds.end(); it++)
	{
		num_points = it->point_cloud.size();
		area = it->geometry.width * it->geometry.length;
		density = (num_points / area);
		it->object_density = density;
	}
}


void
get_features_objects_from_point_cloud(std::list<object_point_cloud_data_t> &list_point_clouds)
{
	set_list_point_clouds_geometry_and_pose(list_point_clouds);
//	set_orientation_from_eigen_vectors(list_point_clouds);
//	set_object_density_from_number_of_points_by_volume(list_point_clouds);
	set_list_poins_clouds_density_number_of_points_by_area(list_point_clouds);
}


object_geometry_t
get_geom_based_on_class(int particle_class) {
	object_geometry_t obj_model;
	if (particle_class < 0 || particle_class > int(object_models.size()))
	{
		obj_model.width = 0.0;
		obj_model.length = 0.0;
		obj_model.height = 0.0;
	}
	else
	{
		obj_model = object_models[particle_class].geometry;
	}
	return obj_model;
}


void
clear_obj_model_features(object_model_features_t &obj_model)
{
	obj_model.model_id = -1;
	obj_model.model_name = (char *)"";
	obj_model.geometry.width = 0.0;
	obj_model.geometry.length = 0.0;
	obj_model.geometry.height = 0.0;
	obj_model.red = 0.0;
	obj_model.green = 0.0;
	obj_model.blue = 0.0;
}


void
clear_obj_model_features(object_features_3d_t &obj_model)
{
	obj_model.model_id = -1;
	obj_model.geometry.width = 0.0;
	obj_model.geometry.length = 0.0;
	obj_model.geometry.height = 0.0;
	obj_model.red = 0.0;
	obj_model.green = 0.0;
	obj_model.blue = 0.0;
	obj_model.cloud->clear();
}


object_model_features_t
get_obj_model_features(int model_id)
{
	object_model_features_t obj_model;

	if (model_id >= 0 && model_id < int(object_models.size()))
		obj_model = object_models[model_id];
	else
		clear_obj_model_features(obj_model);

	return obj_model;
}


object_features_3d_t
get_obj_model_features_3d(int model_id, vector<object_features_3d_t> object_models)
{
	object_features_3d_t obj_model;

	if (model_id >= 0 && model_id < int(object_models.size()))
		obj_model = object_models[model_id];
	else
		clear_obj_model_features(obj_model);

	return obj_model;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr
get_model_point_cloud(int model_id, vector<object_features_3d_t> object_models)
{
	object_features_3d_t obj_model;

	if (model_id >= 0 && model_id < int(object_models.size()))
		obj_model = object_models[model_id];
	else
		clear_obj_model_features(obj_model);

	return obj_model.cloud;
}


particle_datmo_t
compute_average_state_and_update_timestamp(std::vector<particle_datmo_t> &particle_set_t, double timestamp)
{
	particle_datmo_t mean_particle;
	double total_weight = 0.0, highest_weight = 0.0;
	double mean_x, mean_y, mean_theta_x, mean_theta_y, mean_velocity, mean_dist;

	/* most frequent object class */
	std::vector<int> vec_class_counter(num_of_models);
	std::vector<int>::iterator most_frequent;

	/* compute mean particle pose */
	mean_x = 0.0;
	mean_y = 0.0;
	mean_theta_x = 0.0;
	mean_theta_y = 0.0;
	mean_velocity = 0.0;
	mean_dist = 0.0;

	for (std::vector<particle_datmo_t>::iterator it = particle_set_t.begin(); it != particle_set_t.end(); it++)
	{
		mean_x += it->pose.x * it->weight;
		mean_y += it->pose.y * it->weight;
		mean_theta_x += cos(it->pose.theta) * it->weight;
		mean_theta_y += sin(it->pose.theta) * it->weight;
		mean_velocity += it->velocity * it->weight;
		mean_dist += it->dist * it->weight;
		total_weight += it->weight;

		vec_class_counter[it->class_id] += 1;
		it->timestamp = timestamp;

		if (it->weight > highest_weight)
			highest_weight = it->weight;
	}

	double inv_total_weight = 1/total_weight; //multiplication >=20 times faster than division operation
	mean_particle.pose.x = mean_x*inv_total_weight;
	mean_particle.pose.y = mean_y*inv_total_weight;
	mean_particle.pose.theta = carmen_normalize_theta(atan2(mean_theta_y, mean_theta_x));
	mean_particle.velocity = mean_velocity*inv_total_weight;
	mean_particle.dist = mean_dist*inv_total_weight;
	mean_particle.weight = highest_weight*inv_total_weight;

	most_frequent = max_element(vec_class_counter.begin(), vec_class_counter.end());
	mean_particle.class_id = most_frequent - vec_class_counter.begin();
	mean_particle.model_features = get_obj_model_features(mean_particle.class_id);
	mean_particle.timestamp = timestamp;

	return mean_particle;
}


double
get_object_density_by_area(object_point_cloud_data_t obj)
{
	return double(obj.point_cloud.size()) / (obj.geometry.width*obj.geometry.length);
}


double
get_object_3d_diagonal_measurement(object_point_cloud_data_t obj)
{
	return sqrt(obj.geometry.width*obj.geometry.width + obj.geometry.length*obj.geometry.length
			+ obj.geometry.height*obj.geometry.height);
}


int
get_random_model_id(int num_max)
{
	int model_id = -1;

	if (num_max > 0)
		model_id = carmen_int_random(num_max);

	return model_id;
}


void
update_object_pose_and_features(object_point_cloud_data_t &object_point_cloud, particle_datmo_t particle_ref,
		std::vector<particle_datmo_t> particle_set)
{
	object_point_cloud.object_pose.position.x = particle_ref.pose.x;
	object_point_cloud.object_pose.position.y = particle_ref.pose.y;
	object_point_cloud.orientation = particle_ref.pose.theta;
	object_point_cloud.linear_velocity = particle_ref.velocity;
	object_point_cloud.geometric_model = particle_ref.class_id;
	object_point_cloud.model_features = particle_ref.model_features;
	object_point_cloud.mean_particle = particle_ref;
	object_point_cloud.particle_set = particle_set;
}


void
clear_obj_model_features(object_point_cloud_data_t &object_point_cloud)
{
	object_point_cloud.geometric_model = -1;
	clear_obj_model_features(object_point_cloud.model_features);
}

#ifdef AJUSTE
void
print_object_details(object_point_cloud_data_t obj_point_cloud, double x, double y)
{
	/* Analysis pose and dimensions
	double dist = measurement_model(obj_point_cloud.mean_particle,
			obj_point_cloud.mean_particle.pose.x,
			obj_point_cloud.mean_particle.pose.y,
			obj_point_cloud.point_cloud,
			obj_point_cloud.geometry,
			obj_point_cloud.car_global_pose.position);

	printf("%d) %s\n"
			"xc=%.6f yc=%.6f\n"
			"theta=%.6f\n"
			"W=%.6f L=%.6f H=%.6f\n"
			"Dist=%.6f  Weight=%.6f\n"
			"--\n",
			obj_point_cloud.num_color_associate,
			obj_point_cloud.model_features.model_name,
			obj_point_cloud.mean_particle.pose.x,
			obj_point_cloud.mean_particle.pose.y,
			obj_point_cloud.mean_particle.pose.theta,
			obj_point_cloud.geometry.width, obj_point_cloud.geometry.length, obj_point_cloud.geometry.height,
			dist,
			obj_point_cloud.mean_particle.weight);

	*
	*/

	if (obj_point_cloud.mean_particle.pose.x == obj_point_cloud.mean_particle.pose.x && parametro != NULL) // verifica se é NaN
		fprintf(parametro,"%d %s %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
			obj_point_cloud.num_color_associate,
			obj_point_cloud.model_features.model_name,
			x - obj_point_cloud.car_global_pose.position.x,
			y - obj_point_cloud.car_global_pose.position.y,
			obj_point_cloud.mean_particle.pose.x - obj_point_cloud.car_global_pose.position.x,
			obj_point_cloud.mean_particle.pose.y - obj_point_cloud.car_global_pose.position.y,
			obj_point_cloud.mean_particle.pose.theta,
			obj_point_cloud.mean_particle.velocity,
			obj_point_cloud.model_features.geometry.width,
			obj_point_cloud.model_features.geometry.length,
			obj_point_cloud.model_features.geometry.height);

	/* Statistics purposes */
//	for (std::vector<particle_datmo>::iterator it = obj_point_cloud.particle_set.begin();
//			it != obj_point_cloud.particle_set.end(); ++it)
//	{
//		//frame id, particle weight, particle distance, particle class, point cloud dimensions (W,L,H), num_color_associate
//		//"frame_id,part_weight,part_dist,part_class,pcloud_w,pcloud_l,pcloud_h\n"
//		printf("%d,%.10f,%.10f,%d,%.10f,%.10f,%.10f,%d\n",
//				frame, it->weight, it->dist, it->model_features.model_id,
//				obj_point_cloud.geometry.width, obj_point_cloud.geometry.length, obj_point_cloud.geometry.height,
//				obj_point_cloud.num_color_associate);
//	}
}
#endif

bool
is_moving_object(object_point_cloud_data_t obj_point_cloud)
{
	/*** MOVING OBJECT CONDITIONS/THRESHOLDS ***/
	// Minimum velocity
	if (obj_point_cloud.linear_velocity < threshold_min_velocity)
	{
		return false;
	}

	// FIXME Alguns carros não são trackeados (baixa densidade)
	// PointCloud density
//	double density = get_object_density_by_area(obj_point_cloud);
//	if (density < 10.0)
//	{
//		return false;
//	}

	// Object's minimum bounding box diagonal measurement
	double diagonal_measurement = get_object_3d_diagonal_measurement(obj_point_cloud);
	if (diagonal_measurement < 1.0 || diagonal_measurement > 13.5)
	{
		return false;
	}

	// FIXME Objetos não são trackeados caso isso esteja habilitado.
	// Count points below 0.9m remove árvores baixas
//	int count_down_points = 0;
//	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator pit = obj_point_cloud.point_cloud.begin();
//			pit != obj_point_cloud.point_cloud.end(); pit++)
//		if (pit->z < 0.90)
//			count_down_points += 1;
//
//	if (count_down_points < 35)
//	{
//		return false;
//	}

	return true;
}


void
particle_filter_moving_objects_tracking(std::list<object_point_cloud_data_t> &list_point_clouds)
{
	double x, y;
	double delta_time;
	particle_datmo_t particle_reference;

	for (std::list<object_point_cloud_data_t>::iterator it = list_point_clouds.begin(); it != list_point_clouds.end(); it++)
	{
		/* object global pose */
		x = it->object_pose.position.x + it->car_global_pose.position.x;
		y = it->object_pose.position.y + it->car_global_pose.position.y;

		if (it->particle_set.size() == 0)
		{
			/*** INITIALIZE PARTICLES ***/
//			printf("velocity: %.6f\n", it->linear_velocity);
//			init_particle_set(*it, num_of_particles, x, y);
		}
		else if (it->particle_set[0].timestamp != it->timestamp)
		{
			std::vector<particle_datmo_t> particle_set_t_1 = it->particle_set;
			std::vector<particle_datmo_t> particle_set_t;

			delta_time = it->timestamp - it->particle_set[0].timestamp;

			/* KNOWN ISSUE: Car global position included due to lost of precision problem with PCL point types */
			particle_set_t = algorithm_monte_carlo(particle_set_t_1, x, y, delta_time, it->point_cloud,
				it->geometry, it->car_global_pose.position, it->num_color_associate, it->mean_particle);

			particle_reference = compute_average_state_and_update_timestamp(particle_set_t, it->timestamp);

			update_object_pose_and_features(*it, particle_reference, particle_set_t);

			/*** ANALYSE MOVING OBJECT CONDITIONS/THRESHOLDS ***/
			if (is_moving_object(*it))
			{
#ifdef AJUSTE
				print_object_details(*it,x,y);
#endif
			}
			else
			{
				clear_obj_model_features(*it);
				continue;
			}
		}
		else
		{
			clear_obj_model_features(*it);
		}
	}
#ifdef AJUSTE
	if (parametro != NULL)
		fprintf(parametro,"\n");
#endif
}


void
convert_carmen_vector_3d_to_pcl_point_subtracting_global_pose(carmen_vector_3D_t *cloud, int point_cloud_size,
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr, moving_objects_input_data_t moving_objects_input)
{
	if (moving_objects_input.first_offline_map_message == -1)// Agora a global pos eh valida
	{
		for (int k = 0; k < point_cloud_size; k++)
		{
			pcl::PointXYZ pcl_point_3D;

			pcl_point_3D.x = cloud[k].x - moving_objects_input.car_global_pose.position.x;
			pcl_point_3D.y = cloud[k].y - moving_objects_input.car_global_pose.position.y;
			pcl_point_3D.z = cloud[k].z - moving_objects_input.car_global_pose.position.z;

			pcl_cloud_ptr->push_back(pcl_point_3D);
		}
	}
	printf("pcl_cloud_ptr->size: %d\n", (int)pcl_cloud_ptr->size());
}


void
convert_carmen_vector_3d_to_pcl_point(carmen_vector_3D_t *cloud, int point_cloud_size,
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud_ptr, moving_objects_input_data_t moving_objects_input)
{
	if (moving_objects_input.first_offline_map_message == -1)// Agora a global pos eh valida
	{
		for (int k = 0; k < point_cloud_size; k++)
		{
			pcl::PointXYZ pcl_point_3D;

			pcl_point_3D.x = cloud[k].x;
			pcl_point_3D.y = cloud[k].y;
			pcl_point_3D.z = cloud[k].z;

			pcl_point_cloud_ptr->push_back(pcl_point_3D);
		}
	}
}


void
pcl_pointcloud_to_vector_3d_pointcloud_with_sum_global_pose(carmen_vector_3D_t *cloud, int cloud_length,
		carmen_pose_3D_t car_global_pose, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_pcl,
		moving_objects_input_data_t moving_objects_input)
{
	if (moving_objects_input.first_offline_map_message == -1)// Agora a global pos eh valida
	{
		carmen_convert_pcl_point_cloud_to_vector_3d_point_cloud(pointcloud_pcl, cloud, cloud_length);
		sum_global_pose_from_point_cloud(cloud, cloud_length, car_global_pose);
	}
}


void
build_sensor_point_cloud(spherical_point_cloud **points, unsigned char **intensity, int *point_cloud_index,
		int num_points, int max_point_buffer)
{
	(*point_cloud_index)++;
	if ((*point_cloud_index) >= max_point_buffer)
		*point_cloud_index = 0;

	if ((*points)[*point_cloud_index].num_points != num_points)
		intensity[*point_cloud_index] = (unsigned char *)realloc((void *)intensity[*point_cloud_index], num_points * sizeof(unsigned char));

	carmen_alloc_spherical_point_cloud(*points, num_points, *point_cloud_index);
}


int
detect_points_above_ground_in_vertical_beam(int i, const moving_objects_input_data_t &moving_objects_input,
		sensor_data_t *velodyne_data, sensor_parameters_t *velodyne_params, carmen_vector_3D_t *point_clouds, int last_num_points)
{
	carmen_prob_models_get_occuppancy_log_odds_via_unexpeted_delta_range(velodyne_data, velodyne_params, i,
			moving_objects_input.highest_sensor, moving_objects_input.safe_range_above_sensors, 1, 0);

	for (int k = 0; k < velodyne_params->vertical_resolution; k++)
	{
		if (velodyne_data->obstacle_height[0][k] >= 0.5
				&& velodyne_data->obstacle_height[0][k] <= MAXIMUM_HEIGHT_OF_OBSTACLE
				&& !velodyne_data->ray_hit_the_robot[0][k])
		{
			point_clouds[last_num_points].x = velodyne_data->ray_position_in_the_floor[0][k].x;
			point_clouds[last_num_points].y = velodyne_data->ray_position_in_the_floor[0][k].y;
			point_clouds[last_num_points].z = velodyne_data->obstacle_height[0][k];
			last_num_points++;
		}
		else if (velodyne_data->occupancy_log_odds_of_each_ray_target[0][k] > velodyne_params->log_odds.log_odds_l0
				&& velodyne_data->obstacle_height[0][k] >= 0.30
				&& velodyne_data->obstacle_height[0][k] <= MAXIMUM_HEIGHT_OF_OBSTACLE
				&& !velodyne_data->ray_hit_the_robot[0][k])
		{
			point_clouds[last_num_points].x = velodyne_data->ray_position_in_the_floor[0][k].x;
			point_clouds[last_num_points].y = velodyne_data->ray_position_in_the_floor[0][k].y;
			point_clouds[last_num_points].z = velodyne_data->obstacle_height[0][k];
			last_num_points++;
		}

	}
	return last_num_points;
}


int
detect_points_above_ground(sensor_parameters_t *velodyne_params, sensor_data_t *velodyne_data,
		rotation_matrix *r_matrix_car_to_global, carmen_pose_3D_t *robot_pose, carmen_vector_3D_t *robot_velocity,
		double x_origin, double y_origin, int point_cloud_index, double phi, moving_objects_input_data_t moving_objects_input,
		carmen_vector_3D_t *point_clouds)
{
	int i;
	spherical_point_cloud v_zt = velodyne_data->points[point_cloud_index];

	// Ray-trace the grid
	int last_num_points = 0;
	int N = v_zt.num_points / velodyne_params->vertical_resolution;
	double dt = 0.0494 / (double)N;
	carmen_pose_3D_t robot_interpolated_position = *robot_pose;


	for (i = 0; i < v_zt.num_points; i = i + velodyne_params->vertical_resolution)
	{
		robot_interpolated_position = carmen_ackerman_interpolated_robot_position_at_time(robot_interpolated_position, dt, robot_velocity->x,
				phi, moving_objects_input.car_config.distance_between_front_and_rear_axles);

		r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_interpolated_position.orientation);

		carmen_prob_models_compute_relevant_map_coordinates(velodyne_data, velodyne_params, i, robot_interpolated_position.position,
				moving_objects_input.sensor_board_1_pose, r_matrix_car_to_global, moving_objects_input.sensor_board_1_to_car_matrix,
				moving_objects_input.robot_wheel_radius, x_origin, y_origin, &moving_objects_input.car_config, 0, 0);

		last_num_points = detect_points_above_ground_in_vertical_beam(i, moving_objects_input, velodyne_data, velodyne_params,
				point_clouds, last_num_points);
	}
	return last_num_points;
}


int
build_point_cloud_using_velodyne_message(carmen_velodyne_partial_scan_message *velodyne_message,
		sensor_parameters_t *velodyne_params, sensor_data_t *velodyne_data, carmen_vector_3D_t *robot_velocity,
		double phi, moving_objects_input_data_t moving_objects_input, carmen_vector_3D_t *point_clouds)
{
	static rotation_matrix *r_matrix_car_to_global = NULL;
	static int velodyne_message_id;
	int current_point_cloud_index;
	int num_points = velodyne_message->number_of_32_laser_shots * velodyne_params->vertical_resolution;
	int last_num_points = 0;

	if (velodyne_data->last_timestamp == 0.0)
	{
		velodyne_data->last_timestamp = velodyne_message->timestamp;
		velodyne_message_id = 0; // correntemente sao necessarias pelo menos 2 mensagens para se ter uma volta completa de velodyne
	}

	velodyne_data->current_timestamp = velodyne_message->timestamp;

	build_sensor_point_cloud(&(velodyne_data->points), velodyne_data->intensity, &(velodyne_data->point_cloud_index), num_points,
			moving_objects_input.num_velodyne_point_clouds);

	carmen_velodyne_partial_scan_update_points(velodyne_message,
			velodyne_params->vertical_resolution,
			&(velodyne_data->points[velodyne_data->point_cloud_index]),
			velodyne_data->intensity[velodyne_data->point_cloud_index],
			velodyne_params->ray_order,
			velodyne_params->vertical_correction,
			velodyne_params->range_max,
			velodyne_message->timestamp);

	if (velodyne_message_id >= 0)
	{
		carmen_pose_3D_t local_pose;

		local_pose.position = moving_objects_input.car_fused_pose.position;
		local_pose.orientation.yaw = moving_objects_input.car_fused_pose.orientation.yaw;
		local_pose.orientation.pitch = local_pose.orientation.roll = 0.0;

		r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global, local_pose.orientation);
		current_point_cloud_index = velodyne_data->point_cloud_index;
		last_num_points = detect_points_above_ground(velodyne_params, velodyne_data, r_matrix_car_to_global, &local_pose,
				robot_velocity, 0.0, 0.0, current_point_cloud_index, phi, moving_objects_input, point_clouds);

		if (velodyne_message_id > 1000000)
			velodyne_message_id = 0;
	}
	velodyne_message_id++;
	velodyne_data->last_timestamp = velodyne_message->timestamp;

	return last_num_points;
}


void
set_model(object_model_features_t &obj_model, int model_id, char *model_type, double width, double length, double height,
		double red, double green, double blue)
{
	obj_model.model_id = model_id;
	obj_model.model_name = model_type;
	obj_model.geometry.width = width;
	obj_model.geometry.length = length;
	obj_model.geometry.height = height;
	obj_model.red = red;
	obj_model.green = green;
	obj_model.blue = blue;
}


void
set_model(object_model_features_t &obj_model, int model_id, double width, double length, double height,
		double red, double green, double blue)
{
	obj_model.model_id = model_id;
	obj_model.model_name = (char *)"";
	obj_model.geometry.width = width;
	obj_model.geometry.length = length;
	obj_model.geometry.height = height;
	obj_model.red = red;
	obj_model.green = green;
	obj_model.blue = blue;
}


void
set_model(object_features_3d_t &obj_model, int model_id, double width, double length, double height,
		double red, double green, double blue, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	obj_model.model_id = model_id;
	obj_model.geometry.width = width;
	obj_model.geometry.length = length;
	obj_model.geometry.height = height;
	obj_model.red = red;
	obj_model.green = green;
	obj_model.blue = blue;
	obj_model.cloud = cloud;
}


void
set_object_models(std::vector<object_model_features_t> &obj_models)
{
	/* Currently included objects, separated by subclasses
	 * 00 - 10 = cars
	 * 0) sedan; 1) hatch; 2) SUV; 3) station wagon (caminhoneta); 4) kombi =)
	 *
	 * 11 - 20 = motorbikes/bikes
	 * 11) bike/motorbike;;
	 *
	 * 21 - 30 = truck/lorry
	 * 21) small truck;
	 *
	 * 31 - 40 = buses
	 * 31) bus 1;
	 *
	 * 41 - 50 = pedestrians
	 * 41) pedestrian 1;
	 *
	 * 90 - 100 = signs/posts (wrongly classified as a moving objects)
	 *
	 * OBS.: these values were obtained from the 3D point cloud models
	 *
	 * */

	object_model_features_t obj_class;

	/* 0) sedan */
//	set_model(obj_class, 0, 2.1, 4.5, 1.48, 1.0, 0.0, 0.8);
	//	set_model(obj_class, 0, 1.8, 4.4, 1.4, 1.0, 0.0, 0.8);
	set_model(obj_class, 0, (char *)"car", 1.8, 4.4, 1.4, 1.0, 0.0, 0.8);
	obj_models.push_back(obj_class);
	/* 1) hatch */
//	set_model(obj_class, 1, (char *)"hatch", 1.7, 3.8, 1.4, 1.0, 0.0, 0.8);
//	set_model(obj_class, 1, 1.7, 3.8, 1.4, 1.0, 0.0, 0.8);
//	obj_models.push_back(obj_class);
	/* 2) SUV */
//	set_model(obj_class, 2, (char *)"suv", 1.9, 4.4, 1.8, 1.0, 0.0, 0.8);
//	set_model(obj_class, 2, 1.9, 4.4, 1.8, 1.0, 0.0, 0.8);
//	obj_models.push_back(obj_class);
	/* 3) station wagon */
//	set_model(obj_class, 3, (char *)"pickup", 1.8, 5.1, 1.8, 1.0, 0.0, 0.8);
//	set_model(obj_class, 3, 1.8, 5.1, 1.8, 1.0, 0.0, 0.8);
//	obj_models.push_back(obj_class);
	/* 4) VW Kombi */
//	set_model(obj_class, 4, (char *)"kombi", 1.9, 4.5, 2.1, 1.0, 0.0, 0.8);
//	obj_models.push_back(obj_class);

	/* 11) bike/motorbike 1 */
	set_model(obj_class, 11, 0.8, 2.3, 1.6, 0.0, 1.0, 1.0);
	//	set_model(obj_class, 11, 0.7, 2.2, 1.4, 0.0, 1.0, 1.0);
	set_model(obj_class, 11, (char *)"bike", 0.7, 2.2, 1.4, 0.0, 1.0, 1.0);
	obj_models.push_back(obj_class);

	/* 21) small truck */
	//set_model(obj_class, 21, (char *)"truck", 2.5, 5.0, 2.3, 0.5, 0.5, 1.0);
	//set_model(obj_class, 21, (char *)"truck", 2.2, 7.9, 3.1, 0.5, 0.5, 1.0);
	set_model(obj_class, 21, (char *)"truck", 2.2, 6.8, 2.6, 0.5, 0.5, 1.0);
	obj_models.push_back(obj_class);

	/* 31) bus (average estimative) */
//	set_model(obj_class, 3, 2.7, 12.0, 3.0, 1.0, 1.0, 0.0);
	//	set_model(obj_class, 31, 2.9, 12.6, 3.5, 1.0, 1.0, 0.0);
	set_model(obj_class, 31, (char *)"bus", 2.9, 12.6, 3.5, 1.0, 1.0, 0.0);
	obj_models.push_back(obj_class);

	/* 41) pedestrian 1 */
	//	set_model(obj_class, 41, 0.6, 0.6, 1.7, 0.0, 1.0, 0.0);
	set_model(obj_class, 41, (char *)"pedestrian", 0.6, 0.6, 1.7, 0.0, 1.0, 0.0);
	obj_models.push_back(obj_class);

	/* 91) signs/posts (wrongly classified) */
//	set_model(obj_class, 91, "posts", 0.4, 0.4, 2.0, 0.8, 0.0, 1.0);
//	obj_models.push_back(obj_class);

	num_of_models = int(obj_models.size());
}


void
set_object_models2(std::vector<object_model_features_t> &obj_models)
{
	/* Currently included objects, separated by subclasses
	 * 00 - 10 = cars
	 * 0) sedan;
	 * */

	object_model_features_t obj_class;

	/* 0) sedan */
	set_model(obj_class, 0, (char *)"car", 1.8, 4.4, 1.4, 1.0, 0.0, 0.8);
	obj_models.push_back(obj_class);

	num_of_models = int(obj_models.size());
}


void
load_point_cloud(std::string file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name, *cloud) == -1) //* load the file
	{
//		PCL_ERROR("Couldn't read file sedan_500.pcd\n");
		cout << "ERROR: Couldn't read file "<< file_name << endl;
		exit(1);
	}
}


void
set_object_models(std::vector<object_features_3d_t> &obj_models)
{
	/* Currently included objects, separated by subclasses
	 * 00 - 10 = cars
	 * 0) sedan; 1) hatch; 2) SUV; 3) station wagon (caminhoneta); 4) kombi =)
	 *
	 * 11 - 20 = motorbikes/bikes
	 * 11) bike/motorbike;
	 *
	 * 21 - 30 = truck/lorry
	 * 21) small truck;
	 *
	 * 31 - 40 = buses
	 * 31) bus 1;
	 *
	 * 41 - 50 = pedestrians
	 * 41) pedestrian 1;
	 *
	 * 90 - 100 = signs/posts (wrongly classified as a moving objects)
	 *
	 * OBS.: these values were obtained from the 3D point cloud models
	 *
	 * */

	object_features_3d_t obj_class;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	char current_path[FILENAME_MAX];
	if (!getcwd(current_path, sizeof(current_path)))
	{
		printf("ERROR: Couldn't get current path.\n");
		exit(1);
	}

	/* 0) sedan */
	std::string file_absolute_path = string(current_path) + string("/models/sedan_500.pcd");
	load_point_cloud("/home/kenzo/carmen/src/moving_objects/models/sedan_500.pcd", cloud);
	set_model(obj_class, 0, 1.8, 4.4, 1.4, 1.0, 0.0, 0.8, cloud);
	obj_models.push_back(obj_class);

	/* 1) hatch */
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
//	file_absolute_path = string(current_path) + string("/models/hatch_500.pcd");
//	load_point_cloud("/home/kenzo/carmen/src/moving_objects/models/hatch_500.pcd", cloud1);
//	set_model(obj_class, 1, 1.7, 3.8, 1.4, 1.0, 0.0, 0.8, cloud1);
//	obj_models.push_back(obj_class);

	/* 2) SUV */
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
//	file_absolute_path = string(current_path) + string("/models/suv_500.pcd");
//	load_point_cloud("/home/kenzo/carmen/src/moving_objects/models/suv_500.pcd", cloud2);
//	set_model(obj_class, 2, 1.9, 4.4, 1.8, 1.0, 0.0, 0.8, cloud2);
//	obj_models.push_back(obj_class);

	/* 3) station wagon/caminhonete*/
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZ>);
//	file_absolute_path = string(current_path) + string("/models/caminhonete_500.pcd");
//	load_point_cloud("/home/kenzo/carmen/src/moving_objects/models/caminhonete_500.pcd", cloud3);
//	set_model(obj_class, 3, 1.8, 5.1, 1.8, 1.0, 0.0, 0.8, cloud3);
//	obj_models.push_back(obj_class);

	/* 4) VW Kombi */
//	set_model(obj_class, 4, 1.9, 4.5, 2.1, 1.0, 0.0, 0.8);
//	obj_models.push_back(obj_class);

	/* 11) bike/motorbike 1 */
////	set_model(obj_class, 11, 0.8, 2.3, 1.6, 0.0, 1.0, 1.0);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud11 (new pcl::PointCloud<pcl::PointXYZ>);
//	file_absolute_path = string(current_path) + string("/models/motociclista_500.pcd");
//	load_point_cloud("/home/kenzo/carmen/src/moving_objects/models/motociclista_500.pcd", cloud11);
//	set_model(obj_class, 11, 0.7, 2.2, 1.4, 0.0, 1.0, 1.0, cloud11);
//	obj_models.push_back(obj_class);
//
//	/* 21) small truck */
////	set_model(obj_class, 21, 2.5, 5.0, 2.3, 0.5, 0.5, 1.0);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud21 (new pcl::PointCloud<pcl::PointXYZ>);
//	file_absolute_path = string(current_path) + string("/models/truck_500.pcd");
//	load_point_cloud("/home/kenzo/carmen/src/moving_objects/models/truck_500.pcd", cloud21);
//	set_model(obj_class, 21, 2.2, 7.9, 3.1, 0.5, 0.5, 1.0, cloud21);
//	obj_models.push_back(obj_class);
//
//	/* 31) bus (average estimative) */
////	set_model(obj_class, 3, 2.7, 12.0, 3.0, 1.0, 1.0, 0.0);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud31 (new pcl::PointCloud<pcl::PointXYZ>);
//	file_absolute_path = string(current_path) + string("/models/bus_500.pcd");
//	load_point_cloud("/home/kenzo/carmen/src/moving_objects/models/bus_500.pcd", cloud31);
//	set_model(obj_class, 31, 2.9, 12.6, 3.5, 1.0, 1.0, 0.0, cloud31);
//	obj_models.push_back(obj_class);
//
//	/* 41) pedestrian 1 */
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud41 (new pcl::PointCloud<pcl::PointXYZ>);
//	file_absolute_path = string(current_path) + string("/models/pedestre_500.pcd");
//	load_point_cloud("/home/kenzo/carmen/src/moving_objects/models/pedestre_500.pcd", cloud41);
//	set_model(obj_class, 41, 0.6, 0.6, 1.7, 0.0, 1.0, 0.0, cloud41);
//	obj_models.push_back(obj_class);

	/* 91) signs/posts (wrongly classified) */
//	set_model(obj_class, 91, 0.4, 0.4, 2.0, 0.8, 0.0, 1.0);
//	obj_models.push_back(obj_class);

	num_of_models = int(obj_models.size());
}


void
detect_and_follow_moving_objects(std::list<object_point_cloud_data_t> &list_point_clouds,
		carmen_velodyne_partial_scan_message *velodyne_message, sensor_parameters_t *velodyne_params,
		sensor_data_t *velodyne_data, carmen_vector_3D_t *robot_velocity, double phi,
		moving_objects_input_data_t moving_objects_input, carmen_vector_3D_t *carmen_vector_3d_point_cloud,
		carmen_map_p & occupancy_grid_map)
{
	int size_of_point_cloud = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::PointIndices> cluster_indices;

	if (object_models.empty())
	{
		set_object_models(object_models);
		/*** VERSÃO DO EDUARDO: ***/
		// set_object_models2(object_models);
	}
	/*if (object_models_3d.empty())
	{
		//set_object_models(object_models_3d);
	}*/

	/*** GET POINTS FROM LASER SCAN ***/
	size_of_point_cloud = build_point_cloud_using_velodyne_message(velodyne_message, velodyne_params, velodyne_data,
			robot_velocity, phi, moving_objects_input, carmen_vector_3d_point_cloud);
	printf("size_of_point_cloud: %d\n", size_of_point_cloud);

	/*** CONVERT TO PCL POINT CLOUD FORMAT SUBTRACTING GLOBAL POSE ***/
	convert_carmen_vector_3d_to_pcl_point_subtracting_global_pose(carmen_vector_3d_point_cloud, size_of_point_cloud,
			pcl_cloud_ptr, moving_objects_input);

	size_t num_points_pcl_cloud_ptr = pcl_cloud_ptr->size();
	printf("num_points_pcl_cloud_ptr: %d\n", num_points_pcl_cloud_ptr);

	/*** SEGMENT POINT CLOUDS - RETURNS CLUSTER INDICES ***/
	cluster_indices = find_objects_in_point_clouds(pcl_cloud_ptr);

	/*** ASSOCIATE AND CONFIGURE POINT CLOUDS ***/
	association_list_point_clouds(list_point_clouds, pcl_cloud_ptr, moving_objects_input.car_global_pose, cluster_indices,
			velodyne_message->timestamp, occupancy_grid_map, moving_objects_input);

#ifdef AJUSTE
	if (parametro != NULL)
		fprintf(parametro,"%f\n",velodyne_message->timestamp);
#endif

	/*** PARTICLE FILTER FOR DATMO ***/
	particle_filter_moving_objects_tracking(list_point_clouds);

	/* Set the global variable list_previous_point_clouds for next frame */
	set_association_list_point_clouds(list_point_clouds);
}

// TODO Versão para funcionar com a base KITTI

int
build_point_cloud_using_variable_velodyne_message(carmen_velodyne_variable_scan_message *velodyne_message,
		sensor_parameters_t *velodyne_params, sensor_data_t *velodyne_data, carmen_vector_3D_t *robot_velocity,
		double phi, moving_objects_input_data_t moving_objects_input, carmen_vector_3D_t *point_clouds)
{
	static rotation_matrix *r_matrix_car_to_global = NULL;
	static int velodyne_message_id;
	int current_point_cloud_index;
	int num_points = velodyne_message->number_of_shots * velodyne_params->vertical_resolution;
	int last_num_points = 0;

	if (velodyne_data->last_timestamp == 0.0)
	{
		velodyne_data->last_timestamp = velodyne_message->timestamp;
	//	velodyne_message_id = -2; // correntemente sao necessarias pelo menos 2 mensagens para se ter uma volta completa de velodyne
	}

	velodyne_data->current_timestamp = velodyne_message->timestamp;

	build_sensor_point_cloud(&(velodyne_data->points), velodyne_data->intensity, &(velodyne_data->point_cloud_index), num_points,
			moving_objects_input.num_velodyne_point_clouds);

	carmen_velodyne_variable_scan_update_points(velodyne_message,
			velodyne_params->vertical_resolution,
			&(velodyne_data->points[velodyne_data->point_cloud_index]),
			velodyne_data->intensity[velodyne_data->point_cloud_index],
			velodyne_params->ray_order,
			velodyne_params->vertical_correction,
			velodyne_params->range_max,
			velodyne_message->timestamp);

	if (velodyne_message_id >= 0)
	{
		carmen_pose_3D_t local_pose;

		local_pose.position          = moving_objects_input.car_fused_pose.position;
		local_pose.orientation.yaw   = moving_objects_input.car_fused_pose.orientation.yaw;
		local_pose.orientation.pitch = local_pose.orientation.roll = 0.0;

		r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global, local_pose.orientation);
		current_point_cloud_index = velodyne_data->point_cloud_index;
		last_num_points = detect_points_above_ground(velodyne_params, velodyne_data, r_matrix_car_to_global, &local_pose,
				robot_velocity, 0.0, 0.0, current_point_cloud_index, phi, moving_objects_input, point_clouds);

		if (velodyne_message_id > 1000000)
			velodyne_message_id = 0;
	}
	velodyne_message_id++;
	velodyne_data->last_timestamp = velodyne_message->timestamp;

	return last_num_points;
}


void
detect_and_follow_moving_objects_variable_scan(std::list<object_point_cloud_data_t> &list_point_clouds,
		carmen_velodyne_variable_scan_message *velodyne_message, sensor_parameters_t *velodyne_params,
		sensor_data_t *velodyne_data, carmen_vector_3D_t *robot_velocity, double phi,
		moving_objects_input_data_t moving_objects_input, carmen_vector_3D_t *carmen_vector_3d_point_cloud,
		carmen_map_p & occupancy_grid_map)
{
	int size_of_point_cloud = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::PointIndices> cluster_indices;

	if (object_models.empty())
	{
		set_object_models(object_models);
	}

	/*** GET POINTS FROM LASER SCAN ***/
	size_of_point_cloud = build_point_cloud_using_variable_velodyne_message(velodyne_message, velodyne_params, velodyne_data,
			robot_velocity, phi, moving_objects_input, carmen_vector_3d_point_cloud);

	/*** CONVERT TO PCL POINT CLOUD FORMAT SUBTRACTING GLOBAL POSE ***/
	convert_carmen_vector_3d_to_pcl_point_subtracting_global_pose(carmen_vector_3d_point_cloud, size_of_point_cloud,
			pcl_cloud_ptr, moving_objects_input);

	/*** SEGMENT POINT CLOUDS - RETURNS CLUSTER INDICES ***/
	cluster_indices = find_objects_in_point_clouds(pcl_cloud_ptr);

	/*** ASSOCIATE AND CONFIGURE POINT CLOUDS ***/
	association_list_point_clouds(list_point_clouds, pcl_cloud_ptr, moving_objects_input.car_global_pose, cluster_indices,
			velodyne_message->timestamp, occupancy_grid_map, moving_objects_input);

	/*** PARTICLE FILTER FOR DATMO ***/
	particle_filter_moving_objects_tracking(list_point_clouds);

	/* Set the global variable list_previous_point_clouds for next iteration */
	set_association_list_point_clouds(list_point_clouds);
}
