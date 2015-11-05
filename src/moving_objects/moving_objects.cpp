/*********************************************************
	---  Moving Objects Module ---
**********************************************************/

#include <math.h>
#include <carmen/carmen.h>
#include <prob_map.h>
#include <prob_measurement_model.h>
#include <carmen/stereo_util.h>
#include <carmen/stereo_velodyne.h>
#include <tf.h>
#include <vector>
#include <list>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/pca.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/feature.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>
#include <pcl/common/centroid.h>
#include <pcl/features/feature.h>

#include <Eigen/Core>

#include <tf.h>
#include <pcl/io/ply_io.h>

#include <time.h>

#include "moving_objects.h"
#include "monte_carlo_moving_objects_tracking.h"
#include "classifier.h"


//#include <flann/flann.h>
//#include <flann/io/hdf5.h>
//#include <boost/filesystem.hpp>

using std::vector;
using namespace std;

#define MAXIMUM_HEIGHT_OF_OBSTACLE 2.0

double x_origin = 0.0;
double y_origin = 0.0;

std::vector<pcl::PointIndices> cluster_indices;
std::vector<std::pair<Eigen::Vector4f, int> > previous_centroids;
std::vector<std::pair<Eigen::Vector4f, int> > current_centroids;

std::list<object_point_cloud_data_t> list_current_point_clouds;
std::list<object_point_cloud_data_t> list_point_clouds;

std::list<color_palette_and_association_data_t> color_palette_and_association;

int size_num_points = 0;
int first_associate_object_point_clouds = 1;

////////////////////////////
//particle filter parameters
////////////////////////////
int num_of_particles = 500;

///////////////////////////////////////////////////////////////////////////////////////////////


void
get_list_color_palette_and_association()
{
	int i,j;
	int variation = 15; // proportional to 255(r,g,b)
	int total_color = 255; //510;//3060; // proportional to 255(r,g,b)
	Eigen::Vector3f color_palette;
	int num_color = 0;
	int num;
	color_palette_and_association_data_t aux_color_palette_and_association;

	srand(time(NULL));
	num = (255/variation) * 3;

	for (i = 0; i < (total_color/num); i++)
	{

		color_palette[0] = 255;
		color_palette[1] = 0;
		color_palette[2] = 0;
		for(j = 0; j < (255/variation); j++)
		{
			aux_color_palette_and_association.color_palette[0] = color_palette[0];
			aux_color_palette_and_association.color_palette[1] = color_palette[1];
			aux_color_palette_and_association.color_palette[2] = color_palette[2];
			aux_color_palette_and_association.num_association = 0;
			aux_color_palette_and_association.num_color = (total_color - num_color);
			color_palette[0] -= variation;
			color_palette[1] = (rand() % 255);
			color_palette[2] = (rand() % 255);
			color_palette_and_association.push_back(aux_color_palette_and_association);
			num_color++;
		}

		color_palette[0] = 0;
		color_palette[1] = 255;
		color_palette[2] = 0;
		for(j = 0; j < (255/variation); j++)
		{
			aux_color_palette_and_association.color_palette[0] = color_palette[0];
			aux_color_palette_and_association.color_palette[1] = color_palette[1];
			aux_color_palette_and_association.color_palette[2] = color_palette[2];
			aux_color_palette_and_association.num_association = 0;
			aux_color_palette_and_association.num_color = (total_color - num_color);
			color_palette[0] = (rand() % 255);
			color_palette[1] -= variation;
			color_palette[2] = (rand() % 255);
			color_palette_and_association.push_back(aux_color_palette_and_association);
			num_color++;
		}

		color_palette[0] = 0;
		color_palette[1] = 0;
		color_palette[2] = 255;
		for(j = 0; j < (255/variation); j++)
		{
			aux_color_palette_and_association.color_palette[0] = color_palette[0];
			aux_color_palette_and_association.color_palette[1] = color_palette[1];
			aux_color_palette_and_association.color_palette[2] = color_palette[2];
			aux_color_palette_and_association.num_association = 0;
			aux_color_palette_and_association.num_color = (total_color - num_color);
			color_palette[0] = (rand() % 255);
			color_palette[1] = (rand() % 255);
			color_palette[2] -= variation;
			color_palette_and_association.push_back(aux_color_palette_and_association);
			num_color++;
		}
	}
}


std::list<color_palette_and_association_data_t>
get_color_palette_and_association()
{
	return color_palette_and_association;
}


std::list<object_point_cloud_data_t>
get_list_point_clouds()
{
	return list_point_clouds;
}


unsigned int
pcl_euclidean_cluster_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	cluster_indices.clear();

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> cluster_indices2;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.80); 	// Set the spatial cluster tolerance as a measure in the L2 Euclidean space.
	ec.setMinClusterSize (15);		// Set the minimum number of points that a cluster needs to contain in order to be considered valid.
	ec.setMaxClusterSize (20000);	// Set the maximum number of points that a cluster needs to contain in order to be considered valid.
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);

	if(cluster_indices.size() > 0)
	{
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			{
				cloud_cluster->points.push_back (cloud->points[*pit]);
			}

			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
		}
	}
	else
	{
		printf(" Cluster indices <= 0 \n");
	}
	return(cloud->points.size());
}


//unsigned int
//pcl_plane_model_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//{
//	unsigned int length_of_inliers = 0;
//
//	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//	// Create the segmentation object
//	pcl::SACSegmentation<pcl::PointXYZ> seg;
//	// Optional
//	seg.setOptimizeCoefficients (true);
//	// Mandatory
//	seg.setModelType (pcl::SACMODEL_PLANE);
//	seg.setMethodType (pcl::SAC_RANSAC);
//	seg.setMaxIterations (100);
//	seg.setDistanceThreshold (0.3);
//
//	seg.setInputCloud (cloud);
//	seg.segment (*inliers, *coefficients);
//
//
//	size_t k = 0;
//	for (size_t i = 0; i < cloud->points.size(); i++)// pega os pontos fora do plano do chao
//	{
//		if(i == inliers->indices[k])
//		{
//			if(k < inliers->indices.size())
//				k++;
//		}
//		else
//		{
//			cloud->points[length_of_inliers].x = cloud->points[i].x;
//			cloud->points[length_of_inliers].y = cloud->points[i].y;
//			cloud->points[length_of_inliers].z = cloud->points[i].z;
//			length_of_inliers++;
//		}
//	}
//
////		for (size_t i = 0; i < inliers->indices.size (); ++i){//pega os pontos no plano do chao
////			cloud->points[length_of_inliers].x = cloud->points[inliers->indices[i]].x;
////			cloud->points[length_of_inliers].y = cloud->points[inliers->indices[i]].y;
////			cloud->points[length_of_inliers].z = cloud->points[inliers->indices[i]].z;
////			length_of_inliers++;
////		}
//
//	return length_of_inliers;
//}


void
subtract_global_pose_from_point_cloud(carmen_vector_3D_t *cloud, int cloud_length,carmen_pose_3D_t car_global_pose)
{
	for (int k = 0; k < cloud_length; k++)
	{
		cloud[k].x -= car_global_pose.position.x;
		cloud[k].y -= car_global_pose.position.y;
		cloud[k].z -= car_global_pose.position.z;
	}
}


void
sum_global_pose_from_point_cloud(carmen_vector_3D_t *cloud, int cloud_length,carmen_pose_3D_t car_global_pose)
{
	for (int k = 0; k < cloud_length; k++)
	{
		cloud[k].x += car_global_pose.position.x;
		cloud[k].y += car_global_pose.position.y;
		cloud[k].z += car_global_pose.position.z;
	}
}


void
carmen_add_vector_3D_point_to_pcl_PointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, carmen_vector_3D_t point)
{
	pcl::PointXYZ p3D;

	p3D.x = point.x;
	p3D.y = point.y;
	p3D.z = point.z;

	pointcloud->push_back(p3D);
}


void
carmen_add_pcl_PointCloud_point_to_vector_3D(pcl::PointXYZ pointcloud_point, carmen_vector_3D_t *pointcloud_vector, int pos)
{

	pointcloud_vector[pos].x = pointcloud_point.x;
	pointcloud_vector[pos].y = pointcloud_point.y;
	pointcloud_vector[pos].z = pointcloud_point.z;
}


void
carmen_convert_vector_3D_PointCloud_to_pcl_PointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_pcl,
		carmen_vector_3D_t *pointcloud_vector, int pointcloud_length)
{
	for (int k = 0; k < pointcloud_length; k++)
		carmen_add_vector_3D_point_to_pcl_PointCloud(pointcloud_pcl, pointcloud_vector[k]);
}


void
carmen_convert_pcl_PointCloud_to_vector_3D_PointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_pcl,
		carmen_vector_3D_t *pointcloud_vector, int pointcloud_length)
{
	for (int k = 0; k < pointcloud_length; k++)
		carmen_add_pcl_PointCloud_point_to_vector_3D(pointcloud_pcl->points[k], pointcloud_vector, k);
}


void
get_point_cloud_cluster_i_from_point_cloud(int i, pcl::PointCloud<pcl::PointXYZ>::Ptr aux_pcl_cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_pcl)
{
	for (std::vector<int>::const_iterator pit = cluster_indices[i].indices.begin (); pit != cluster_indices[i].indices.end (); pit++)
	{
		aux_pcl_cloud->push_back(pointcloud_pcl->points[*pit]);
	}
}


double
distance_between_3D_point_and_car_global_pose(Eigen::Vector4f point_3D, carmen_pose_3D_t car_global_pose, carmen_pose_3D_t associated_car_global_pose)
{
	carmen_vector_3D_t point_3D_global_pose;

	point_3D_global_pose.x = point_3D[0] + associated_car_global_pose.position.x;
	point_3D_global_pose.y = point_3D[1] + associated_car_global_pose.position.y;
	point_3D_global_pose.z = point_3D[2] + associated_car_global_pose.position.z;

	double distance = sqrt (pow((point_3D_global_pose.x - car_global_pose.position.x), 2) +
						    pow((point_3D_global_pose.y - car_global_pose.position.y), 2) +
						    pow((point_3D_global_pose.z - car_global_pose.position.z), 2));

	return(distance);
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
			return(num_color);
		}

		if (it->num_association == 0)
		{
			num_color = it->num_color;
			continue;
		}
	}
	if (num_color == -1){
		printf("error: number of point clouds greater than the number of colors \n");
		exit(1);
	}

	return (num_color);
}


//void
//releasing_color(int num_color)
//{
//	for (std::list<color_palette_and_association_data_t>::iterator it = color_palette_and_association.begin();
//			it != color_palette_and_association.end(); it++)
//	{
//		if (num_color == it->num_color)
//		{
//			it->num_association = 0;
//			continue;
//		}
//	}
//}


void
increment_color_palette_and_association(int num_color)
{

	for (std::list<color_palette_and_association_data_t>::iterator it = color_palette_and_association.begin();
			it != color_palette_and_association.end(); it++)
	{
		if (num_color == it->num_color)
		{
			it->num_association += 1;
			continue;
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
			continue;
		}
	}
}


void
exclude_objects_point_clouds_no_necessary_anymore(std::list<object_point_cloud_data_t> *list_point_clouds, carmen_pose_3D_t car_global_pose)
{
	Eigen::Vector4f centroid;
	carmen_pose_3D_t associated_car_global_pose_point_cloud;
	int num_color_associate;


	for (std::list<object_point_cloud_data_t>::iterator it = list_point_clouds->begin(); it != list_point_clouds->end(); )
	{
		centroid = it->centroid;
		associated_car_global_pose_point_cloud.position.x = it->car_global_pose.position.x;
		associated_car_global_pose_point_cloud.position.y = it->car_global_pose.position.y;
		associated_car_global_pose_point_cloud.position.z = it->car_global_pose.position.z;

		double distance = distance_between_3D_point_and_car_global_pose(centroid, car_global_pose, associated_car_global_pose_point_cloud);
		it->distance_object_pose_and_car_global_pose = distance;

		if ((distance >= 25.0))
		{
			num_color_associate = it->num_color_associate;
//			releasing_color(num_color_associate);
			decrement_color_palette_and_association(num_color_associate);
			it = list_point_clouds->erase(it);
		}
		else
		{
			it++;
		}
	}
}


void
include_unassociated_objects_point_clouds(std::list<object_point_cloud_data_t> *list_point_clouds,
		std::list<object_point_cloud_data_t> *list_current_point_clouds)
{

	object_point_cloud_data_t aux_objects_data;
	int num_color_associate;

	for (std::list<object_point_cloud_data_t>::iterator it = list_current_point_clouds->begin(); it != list_current_point_clouds->end(); it++)
	{
		if (it->num_color_associate == -1)
		{
			num_color_associate = search_free_color();

			if (num_color_associate < 0)
			{
				printf("error: search_free_color \n");
				return;
			}

			aux_objects_data.point_cloud = it->point_cloud;
			aux_objects_data.num_color_associate = num_color_associate;
			aux_objects_data.label_associate = it->label_associate;
			aux_objects_data.car_global_pose = it->car_global_pose;
			aux_objects_data.centroid = it->centroid;
			aux_objects_data.orientation = it->orientation;
			aux_objects_data.delta_time_t_and_t_1 = 0.0; //it->delta_time_t_and_t_1;
			aux_objects_data.distance_object_pose_and_car_global_pose = it->distance_object_pose_and_car_global_pose;
			aux_objects_data.geometric_model = it->geometric_model;
			aux_objects_data.geometry = it->geometry;
			aux_objects_data.linear_velocity = it->linear_velocity;
			aux_objects_data.timestamp = it->timestamp;
			list_point_clouds->push_back (aux_objects_data);

			increment_color_palette_and_association(num_color_associate);
		}
	}
}


void
get_list_current_point_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_pcl, std::list<object_point_cloud_data_t> *list_point_clouds,
		carmen_pose_3D_t car_global_pose, double timestamp)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr aux_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	int association = -1; // -1 = not associated, label of association and color;
	Eigen::Vector4f centroid;
	object_point_cloud_data_t aux_objects_data;
	Eigen::Vector4f min_point;
	Eigen::Vector4f max_point;

	for (size_t i = 0; i < cluster_indices.size(); i++)
	{
		get_point_cloud_cluster_i_from_point_cloud(i, aux_pcl_cloud, pointcloud_pcl);
//		pcl::compute3DCentroid(*aux_pcl_cloud, centroid);
		// testando ponto central da bounding box
		pcl::getMinMax3D(*aux_pcl_cloud, min_point, max_point);

		centroid[0] = (max_point[0] + min_point[0]) / 2.0;
		centroid[1] = (max_point[1] + min_point[1]) / 2.0;
		centroid[2] = (max_point[2] + min_point[2]) / 2.0;

		aux_objects_data.point_cloud = *aux_pcl_cloud;
		aux_objects_data.num_color_associate = association;
		aux_objects_data.label_associate = 0;
		aux_objects_data.car_global_pose = car_global_pose;
		aux_objects_data.centroid = centroid;
		aux_objects_data.distance_object_pose_and_car_global_pose = 0.0;
		aux_objects_data.linear_velocity = 0.0;
		aux_objects_data.object_pose.position.x = 0.0;
		aux_objects_data.object_pose.position.y = 0.0;
		aux_objects_data.object_pose.position.z = 0.0;
		aux_objects_data.geometry.length = 0.0;
		aux_objects_data.geometry.width = 0.0;
		aux_objects_data.geometry.height = 0.0;
		aux_objects_data.geometric_model = -1;
		aux_objects_data.orientation = 0.0;
		aux_objects_data.object_density = 0.0;
		aux_objects_data.delta_time_t_and_t_1 = 0.0;

		aux_objects_data.particle_set.resize(num_of_particles);

		aux_objects_data.timestamp = timestamp;
		list_point_clouds->push_back (aux_objects_data);
		aux_pcl_cloud->clear();
	}
}


double
distance_between_two_3D_points(Eigen::Vector4f current_point, carmen_pose_3D_t c_associated_car_global_pose_point_cloud,
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

	double distance = sqrt ((pow(d_x, 2)) + (pow(d_y, 2)) + (pow(d_z, 2)));

	return(distance);
}


double
distance_between_two_2D_points(Eigen::Vector4f current_point, carmen_pose_3D_t c_associated_car_global_pose_point_cloud,
		Eigen::Vector4f previous_point, carmen_pose_3D_t p_associated_car_global_pose_point_cloud)
{
	double d_x, d_y;

	carmen_vector_3D_t current_point_global_pose;
	carmen_vector_3D_t previous_point_global_pose;

	current_point_global_pose.x = current_point[0] + c_associated_car_global_pose_point_cloud.position.x;
	current_point_global_pose.y = current_point[1] + c_associated_car_global_pose_point_cloud.position.y;

	previous_point_global_pose.x = previous_point[0] + p_associated_car_global_pose_point_cloud.position.x;
	previous_point_global_pose.y = previous_point[1] + p_associated_car_global_pose_point_cloud.position.y;

	d_x = current_point_global_pose.x - previous_point_global_pose.x;
	d_y = current_point_global_pose.y - previous_point_global_pose.y;

	double distance = sqrt ((pow(d_x, 2)) + (pow(d_y, 2)));

	return(distance);
}


double
calculate_velocity_of_the_object(double distance, double delta_time)
{
	double velocity;
	if (delta_time <= 0.0)
	{
		velocity = 0.0;
	}
	else
	{
		velocity = (distance / delta_time);
	}

	return (velocity);

}


double
orientation_by_displacement_between_two_points(Eigen::Vector4f current_point, carmen_pose_3D_t c_associated_car_global_pose_point_cloud,
		Eigen::Vector4f previous_point, carmen_pose_3D_t p_associated_car_global_pose_point_cloud)
{
	double delta_x, delta_y;

	carmen_vector_3D_t current_point_global_pose;
	carmen_vector_3D_t previous_point_global_pose;

	current_point_global_pose.x = current_point[0] + c_associated_car_global_pose_point_cloud.position.x;
	current_point_global_pose.y = current_point[1] + c_associated_car_global_pose_point_cloud.position.y;

	previous_point_global_pose.x = previous_point[0] + p_associated_car_global_pose_point_cloud.position.x;
	previous_point_global_pose.y = previous_point[1] + p_associated_car_global_pose_point_cloud.position.y;


	delta_x = current_point_global_pose.x - previous_point_global_pose.x;
	delta_y = current_point_global_pose.y - previous_point_global_pose.y;

	double orientation = atan2(delta_y, delta_x);

	return(orientation);
}


void
updated_label_associate(std::list<object_point_cloud_data_t> *list_point_clouds)
{
	for (std::list<object_point_cloud_data_t>::iterator pit = list_point_clouds->begin(); pit != list_point_clouds->end(); pit++)
		pit->label_associate = 0;
}


void
check_associate_between_point_clouds_from_centroids(std::list<object_point_cloud_data_t> *list_point_clouds,
		std::list<object_point_cloud_data_t> *list_current_point_clouds)
{
	double distance;
	double distance_x_and_y;
	Eigen::Vector4f previous_centroid;
	Eigen::Vector4f current_centroid;
	object_point_cloud_data_t aux_objects_data;
	double threshold_between_points = 2.5;
	double previous_timestamp;
	double previous_timestamp_best = 0.0;
	double current_timestamp;
	double delta_time;
	double orientation;

	for (std::list<object_point_cloud_data_t>::iterator it = list_current_point_clouds->begin(); it != list_current_point_clouds->end(); it++)
	{
		double min_dist = 99999.0;
		int min_dist_reading = -1;
		distance_x_and_y = 99999.0;
		orientation = 0.0;

		current_centroid = it->centroid;
		current_timestamp = it->timestamp;

		for (std::list<object_point_cloud_data_t>::iterator pit = list_point_clouds->begin(); pit != list_point_clouds->end(); pit++)
		{
			previous_centroid = pit->centroid;
			previous_timestamp = pit->timestamp;


			distance = distance_between_two_3D_points(current_centroid, it->car_global_pose, previous_centroid, pit->car_global_pose);

			if (distance < min_dist && pit->label_associate == 0)
//			if (distance < min_dist)
			{
				pit->delta_time_t_and_t_1 = -1.0;
				min_dist = distance;
				min_dist_reading = pit->num_color_associate;
				distance_x_and_y = distance_between_two_2D_points(current_centroid, it->car_global_pose, previous_centroid, pit->car_global_pose);
				orientation = orientation_by_displacement_between_two_points(current_centroid, it->car_global_pose, previous_centroid, pit->car_global_pose);
				previous_timestamp_best = previous_timestamp;

			}
		}

//		if (min_dist < threshold_between_points)
		if (distance_x_and_y < threshold_between_points)
		{
			for (std::list<object_point_cloud_data_t>::iterator pit = list_point_clouds->begin(); pit != list_point_clouds->end(); pit++)
			{
				if (pit->num_color_associate == min_dist_reading)
				{
					delta_time = (current_timestamp - previous_timestamp_best);
					pit->linear_velocity = calculate_velocity_of_the_object(distance_x_and_y, delta_time);
					pit->delta_time_t_and_t_1 = delta_time;
					pit->orientation = orientation;
					pit->car_global_pose = it->car_global_pose;
					pit->centroid = it->centroid;
					pit->point_cloud = it->point_cloud;
					pit->label_associate = 1;
					pit->distance_object_pose_and_car_global_pose = it->distance_object_pose_and_car_global_pose;
					pit->geometric_model = it->geometric_model;
					pit->geometry = it->geometry;
					pit->object_density = it->object_density;
					pit->timestamp = it->timestamp;
					it->num_color_associate = min_dist_reading;
					continue;
				}
			}
		}
		else
		{
			it->num_color_associate = -1;
		}
	}
	updated_label_associate(list_point_clouds);
}


void
filter_curbs_point_cloud(std::list<object_point_cloud_data_t> *list_current_point_clouds)
{
	int num_color_associate;
	double sum;
	double average_height;
	double variance;
	double z;
	double threshold_average_height = 0.5;
	double threshold_variance = 0.02;

	for (std::list<object_point_cloud_data_t>::iterator it = list_current_point_clouds->begin(); it != list_current_point_clouds->end();)
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
			sum += pow ((z - average_height), 2);
		}
		variance = (sum / it->point_cloud.size());

		if ((average_height < threshold_average_height) && (variance < threshold_variance))
		{
			num_color_associate = it->num_color_associate;
			decrement_color_palette_and_association(num_color_associate);
			it = list_current_point_clouds->erase(it);
		}
		else
		{
			it++;
		}
	}
}


void
associate_objects_point_clouds(std::list<object_point_cloud_data_t> *list_point_clouds,
		std::list<object_point_cloud_data_t> *list_current_point_clouds)
{
	if (first_associate_object_point_clouds)
	{
		get_list_color_palette_and_association();
		first_associate_object_point_clouds = 0;
	}
	else
	{
	filter_curbs_point_cloud(list_current_point_clouds);

	check_associate_between_point_clouds_from_centroids(list_point_clouds, list_current_point_clouds);
	}
}


void
associate_object_point_clouds_with_previous_point_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,
		carmen_pose_3D_t car_global_pose, double timestamp)
{
	get_list_current_point_clouds(pcl_cloud, &list_current_point_clouds, car_global_pose, timestamp);
	associate_objects_point_clouds(&list_point_clouds, &list_current_point_clouds);
	include_unassociated_objects_point_clouds(&list_point_clouds, &list_current_point_clouds);
	exclude_objects_point_clouds_no_necessary_anymore(&list_point_clouds, car_global_pose);

	list_current_point_clouds.clear();
}


void
find_object_point_clouds(int last_num_points, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud)
{

	if(pcl_cloud->points.size() > 0)
	{
//		last_num_points = pcl_plane_model_segmentation(pcl_cloud);

		last_num_points = pcl_euclidean_cluster_extraction(pcl_cloud);

		size_num_points = last_num_points;
	}
}


void
get_geometry_and_pose_moving_objects(std::list<object_point_cloud_data_t> *list_point_clouds)
{
	Eigen::Vector4f min_point;
	Eigen::Vector4f max_point;
	double d_x, d_y, d_z;

	for (std::list<object_point_cloud_data_t>::iterator pit = list_point_clouds->begin(); pit != list_point_clouds->end(); pit++)
	{
		pcl::getMinMax3D(pit->point_cloud, min_point, max_point);
		d_x = (max_point[0] + pit->car_global_pose.position.x) - (min_point[0] + pit->car_global_pose.position.x);
		d_y = (max_point[1] + pit->car_global_pose.position.y) - (min_point[1] + pit->car_global_pose.position.y);
		d_z = (max_point[2] + pit->car_global_pose.position.z) - (min_point[2] + pit->car_global_pose.position.z);

		pit->object_pose.position.x = pit->centroid[0];//(max_point[0] + min_point[0]) / 2.0;
		pit->object_pose.position.y = pit->centroid[1];//(max_point[1] + min_point[1]) / 2.0;
		pit->object_pose.position.z = pit->centroid[2];//(max_point[2] + min_point[2]) / 2.0;

		pit->geometry.length = (fabs (d_x));
		pit->geometry.width = (fabs (d_y));
		pit->geometry.height = (fabs (d_z));
	}
}


void
get_orientation_from_eigen_vectors(std::list<object_point_cloud_data_t> *list_point_clouds)
{
	Eigen::Matrix3f covariance_matrix;
	Eigen::Vector4f xyz_centroid;
	Eigen::Vector3f eigen_values;
	Eigen::Matrix3f eigen_vectors;
	double yaw;

	for (std::list<object_point_cloud_data_t>::iterator it = list_point_clouds->begin(); it != list_point_clouds->end(); it++)
	{
//		xyz_centroid = it->centroid;
		pcl::compute3DCentroid(it->point_cloud, xyz_centroid);

//		pcl::computeCovarianceMatrix (it->point_cloud, xyz_centroid, covariance_matrix);
		pcl::computeCovarianceMatrixNormalized(it->point_cloud, xyz_centroid, covariance_matrix);
		pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);

		// calculate angle of orientation
		yaw = 	atan2 ((double) eigen_vectors (1,2), (double) eigen_vectors (0,2));
		it->orientation = yaw;
	}
}


void
get_object_density_from_number_of_points_by_volume(std::list<object_point_cloud_data_t> *list_point_clouds)
{
	double density;
	int num_points;
	double volume;

	for (std::list<object_point_cloud_data_t>::iterator it = list_point_clouds->begin(); it != list_point_clouds->end(); it++)
	{
		num_points = it->point_cloud.size();
		volume = it->geometry.width * it->geometry.length * it->geometry.height;
		density = (num_points / volume);
		it->object_density = density;
	}
}


void
get_object_density_from_number_of_points_by_area(std::list<object_point_cloud_data_t> *list_point_clouds)
{
	double density;
	int num_points;
	double area;

	for (std::list<object_point_cloud_data_t>::iterator it = list_point_clouds->begin(); it != list_point_clouds->end(); it++)
	{
		num_points = it->point_cloud.size();
		area = it->geometry.width * it->geometry.length;
		density = (num_points / area);
		it->object_density = density;
	}
}


void
get_features_objects_from_point_cloud(std::list<object_point_cloud_data_t> *list_point_clouds)
{
	get_geometry_and_pose_moving_objects(list_point_clouds);

	get_orientation_from_eigen_vectors(list_point_clouds);

//	get_object_density_from_number_of_points_by_volume(list_point_clouds);

	get_object_density_from_number_of_points_by_area(list_point_clouds);
}


void
classify_moving_objects(std::list<object_point_cloud_data_t> *list_point_clouds)
{
	std::pair<int,float> classifier_cloud;

	for (std::list<object_point_cloud_data_t>::iterator it = list_point_clouds->begin(); it != list_point_clouds->end(); it++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr aux_pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		for (pcl::PointCloud<pcl::PointXYZ>::const_iterator pit = it->point_cloud.begin (); pit != it->point_cloud.end (); pit++)
		{
			pcl::PointXYZ point;
			point.x = pit->x;
			point.y = pit->y;
			point.z = pit->z;
			aux_pcl_cloud->push_back(point);
		}

//		printf("aux_pcl_cloud->size() %d \n", aux_pcl_cloud->size());

		classifier_cloud = classifier_point_cloud (aux_pcl_cloud);
		if (classifier_cloud.first == 3)// || classifier_cloud.first == 20 || classifier_cloud.first == 22)
		{
			it->geometric_model = 1;

			printf("distance = %f numero = %d\n",classifier_cloud.second,it->num_color_associate);

		}
		else
		{
			it->geometric_model = -1;
		}
//		printf ("classifier_point_cloud.first, %d classifier_point_cloud.second, %lf \n", classifier_cloud.first, classifier_cloud.second);
//		printf ("it->num_color_associate %d\n", it->num_color_associate);

	}
}


particle
select_best_particle(std::vector<particle> *particle_set_t)
{
	double max_weight = 0.0;
	particle best_particle;

	for(std::vector<particle>::iterator it = particle_set_t->begin();
			it != particle_set_t->end(); it++)
	{
		if (it->weight >= max_weight)
		{
			max_weight = it->weight;
			best_particle = *it;
		}
	}
	return(best_particle);
}


particle
compute_average_state(std::vector<particle> *particle_set_t)
{
	particle mean_particle;
	double total_weight = 0.0;
	double mean_x, mean_y, mean_theta_x, mean_theta_y, mean_velocity;

	/* compute mean particle pose */
	mean_x = 0.0;
	mean_y = 0.0;
	mean_theta_x = 0.0;
	mean_theta_y = 0.0;
	mean_velocity = 0.0;
	for(std::vector<particle>::iterator it = particle_set_t->begin();
				it != particle_set_t->end(); it++)
	{
		mean_x += it->pose.x * it->weight;
		mean_y += it->pose.y * it->weight;
		mean_theta_x += cos(it->pose.theta) * it->weight;
		mean_theta_y += sin(it->pose.theta) * it->weight;
		mean_velocity += it->velocity * it->weight;
		total_weight += it->weight;
	}

	mean_particle.pose.x = mean_x / total_weight;
	mean_particle.pose.y = mean_y / total_weight;
	mean_particle.pose.theta = carmen_normalize_theta(atan2(mean_theta_y, mean_theta_x));
	mean_particle.velocity = mean_velocity / total_weight;
//	mean_particle.velocity = mean_velocity;

	return(mean_particle);
}



void
particle_filter_moving_objects_tracking(std::list<object_point_cloud_data_t> *list_point_clouds)
{
	double x, y;
	double delta_time, diagonal_measurement;
	particle particle_reference;
	int m;

	for (std::list<object_point_cloud_data_t>::iterator it = list_point_clouds->begin();
			it != list_point_clouds->end(); it++)
	{
		x = it->object_pose.position.x + it->car_global_pose.position.x;
		y = it->object_pose.position.y + it->car_global_pose.position.y;
		delta_time = it->delta_time_t_and_t_1;


		diagonal_measurement = sqrt ((it->geometry.width * it->geometry.width) + (it->geometry.length * it->geometry.length)
				+ (it->geometry.height * it->geometry.height));

		if (it->particle_set.size() == 0)
		{
			particle particle_t_1;
			for(int i = 0; i < num_of_particles; i++)
			{
				particle_t_1.pose.x = x;
				particle_t_1.pose.y = y;
				particle_t_1.pose.theta = carmen_uniform_random(-M_PI, M_PI);//carmen_normalize_theta(it->orientation + carmen_gaussian_random(0.0, M_PI));
				particle_t_1.velocity = carmen_uniform_random(0.0, 25.0);//0.0; //it->linear_velocity;
				particle_t_1.weight = (1.0 / double (num_of_particles));
				particle_t_1.timestamp = it->timestamp;
				it->particle_set.push_back(particle_t_1);
			}
		}
		else
		{
			if (it->particle_set[0].timestamp != it->timestamp)
			{
				std::vector<particle> particle_set_t_1;
				std::vector<particle> particle_set_t;

				delta_time = it->timestamp - it->particle_set[0].timestamp;

				particle particle_t_1;
				for (std::vector<particle>::iterator pit = it->particle_set.begin();
					pit != it->particle_set.end(); pit++)
				{
					particle_t_1.pose.x = pit->pose.x;
					particle_t_1.pose.y = pit->pose.y;
					particle_t_1.pose.theta = pit->pose.theta;
					particle_t_1.velocity = pit->velocity;
					particle_t_1.weight = (1.0 / double (num_of_particles));
					particle_t_1.timestamp = pit->timestamp;
					particle_set_t_1.push_back(particle_t_1);
				}

				pcl::PointCloud<pcl::PointXYZ> aux_pcl_cloud;
				for (pcl::PointCloud<pcl::PointXYZ>::const_iterator pit = it->point_cloud.begin (); pit != it->point_cloud.end (); pit++)
				{
					pcl::PointXYZ point;
					point.x = pit->x;
					point.y = pit->y;
					point.z = pit->z;
					aux_pcl_cloud.push_back(point);
				}

				particle_set_t = algorithm_monte_carlo(&particle_set_t_1, x, y, delta_time,
						aux_pcl_cloud);

//				particle_reference = select_best_particle(&particle_set_t);

				particle_reference = compute_average_state(&particle_set_t);

				it->object_pose.position.x = particle_reference.pose.x;
				it->object_pose.position.y = particle_reference.pose.y;
				it->orientation = particle_reference.pose.theta;
				it->linear_velocity = particle_reference.velocity;

				m = 0;
				for (std::vector<particle>::iterator pit = it->particle_set.begin();
					pit != it->particle_set.end(); pit++)
				{
					pit->pose.x = particle_set_t[m].pose.x;
					pit->pose.y = particle_set_t[m].pose.y;
					pit->pose.theta = particle_set_t[m].pose.theta;
					pit->velocity = particle_set_t[m].velocity;
					pit->weight = 0.0; //particle_set_t[m].weight;
					pit->timestamp = it->timestamp;
					m++;
				}

				int count_down_points = 0;
				for (pcl::PointCloud<pcl::PointXYZ>::const_iterator pit = it->point_cloud.begin (); pit != it->point_cloud.end (); pit++)
				{
					if (pit->z < 0.90)
						count_down_points += 1;
				}

//				FILE * arq;
//				arq = fopen("teste.txt", "a");


				//filtering
				if ((it->linear_velocity > 3.0) && (it->object_density > 10.0) && (count_down_points > 35)
					&& (diagonal_measurement > 1.0) && (diagonal_measurement < 12.0)) //&& (it->geometric_model == 1))
				{
					it->geometric_model = 1;
					printf("it->num_color_associate %d \n", it->num_color_associate);
//					printf("it->timestamp %lf \n", it->timestamp);
//					fprintf(arq, "%d %lf \n", it->num_color_associate, it->timestamp);
//					fprintf(arq, "%lf \n", it->distance_object_pose_and_car_global_pose);

				}
				else
				{
					it->geometric_model = -1;
				}

//				fclose(arq);

			}
			else
			{
				it->geometric_model = -1;
			}
		}
	}
}


void
vector_3D_PointCloud_to_pcl_PointCloud_with_subtract_global_pose(carmen_vector_3D_t *cloud, int cloud_length,
		carmen_pose_3D_t car_global_pose, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_pcl,
		moving_objects_input_data_t moving_objects_input)
{
	if(moving_objects_input.first_offline_map_message == -1)// Agora a global pos eh valida
	{
		subtract_global_pose_from_point_cloud(cloud, cloud_length, car_global_pose);

		carmen_convert_vector_3D_PointCloud_to_pcl_PointCloud(pointcloud_pcl, cloud, cloud_length);
	}
}


void
pcl_PointCloud_to_vector_3D_PointCloud_with_sum_global_pose(carmen_vector_3D_t *cloud, int cloud_length,
		carmen_pose_3D_t car_global_pose, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_pcl,
		moving_objects_input_data_t moving_objects_input)
{
	if(moving_objects_input.first_offline_map_message == -1)// Agora a global pos eh valida
	{
		carmen_convert_pcl_PointCloud_to_vector_3D_PointCloud(pointcloud_pcl, cloud, cloud_length);

		sum_global_pose_from_point_cloud(cloud, cloud_length, car_global_pose);
	}
}


void
build_sensor_point_cloud(spherical_point_cloud **points, unsigned char **intensity, int *point_cloud_index, int num_points, int max_point_buffer)
{
	(*point_cloud_index)++;
	if ((*point_cloud_index) >= max_point_buffer)
		*point_cloud_index = 0;

	if ((*points)[*point_cloud_index].num_points != num_points)
		intensity[*point_cloud_index] = (unsigned char *)realloc((void *)intensity[*point_cloud_index], num_points * sizeof(unsigned char));

	carmen_alloc_spherical_point_cloud(*points, num_points, *point_cloud_index);
}


int
detect_points_above_ground_in_vertical_beam(int i, const moving_objects_input_data_t &moving_objects_input, sensor_data_t *velodyne_data,
		sensor_parameters_t *velodyne_params, carmen_vector_3D_t *point_clouds, int &last_num_points)
{

	carmen_prob_models_get_occuppancy_log_odds_via_unexpeted_delta_range(velodyne_data, velodyne_params, i,
			moving_objects_input.highest_sensor, moving_objects_input.safe_range_above_sensors, 1);

	for (int k = 0; k < velodyne_params->vertical_resolution; k++)
	{
		// remove ground and height maximum of obstacle
		if (velodyne_data->occupancy_log_odds_of_each_ray_target[k] > velodyne_params->log_odds.log_odds_l0
			&& velodyne_data->obstacle_height[k] <= MAXIMUM_HEIGHT_OF_OBSTACLE)
		{
			point_clouds[last_num_points].x = velodyne_data->ray_position_in_the_floor[k].x;
			point_clouds[last_num_points].y = velodyne_data->ray_position_in_the_floor[k].y;
			point_clouds[last_num_points].z = velodyne_data->obstacle_height[k];
			last_num_points++;
		}
//		if ((velodyne_data->obstacle_height[k] >= 0.5) && (velodyne_data->obstacle_height[k] <= MAXIMUM_HEIGHT_OF_OBSTACLE))
//		{
//			point_clouds[last_num_points].x = velodyne_data->ray_position_in_the_floor[k].x;
//			point_clouds[last_num_points].y = velodyne_data->ray_position_in_the_floor[k].y;
//			point_clouds[last_num_points].z = velodyne_data->obstacle_height[k];
//			last_num_points++;
//		}
//		else if (velodyne_data->occupancy_log_odds_of_each_ray_target[k] > velodyne_params->log_odds.log_odds_l0
//						&& velodyne_data->obstacle_height[k] <= MAXIMUM_HEIGHT_OF_OBSTACLE)
//		{
//			point_clouds[last_num_points].x = velodyne_data->ray_position_in_the_floor[k].x;
//			point_clouds[last_num_points].y = velodyne_data->ray_position_in_the_floor[k].y;
//			point_clouds[last_num_points].z = velodyne_data->obstacle_height[k];
//			last_num_points++;
//		}

	}
	return last_num_points;
}


int
detect_points_above_ground(sensor_parameters_t *velodyne_params, sensor_data_t *velodyne_data, rotation_matrix *r_matrix_car_to_global,
				 carmen_pose_3D_t *robot_pose, carmen_vector_3D_t *robot_velocity, double x_origin, double y_origin, int point_cloud_index,
				 double phi, moving_objects_input_data_t moving_objects_input, carmen_vector_3D_t *point_clouds)
{
	int i, j;
	spherical_point_cloud v_zt = velodyne_data->points[point_cloud_index];
	double dt = 0.0;
	carmen_pose_3D_t robot_interpolated_position;

	// Ray-trace the grid
	int last_num_points = 0;

	for (i = 0, j = 0; i < v_zt.num_points; i = i + velodyne_params->vertical_resolution, j++)
	{
		dt = j * velodyne_params->time_spent_by_each_scan;
		robot_interpolated_position = carmen_ackerman_interpolated_robot_position_at_time(*robot_pose, dt, robot_velocity->x,
				phi, moving_objects_input.car_config.distance_between_front_and_rear_axles);

		r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global, robot_interpolated_position.orientation);

		carmen_prob_models_compute_relevant_map_coordinates(velodyne_data, velodyne_params, i, robot_interpolated_position.position,
				moving_objects_input.sensor_board_1_pose, r_matrix_car_to_global, moving_objects_input.sensor_board_1_to_car_matrix,
				moving_objects_input.robot_wheel_radius, x_origin, y_origin, &moving_objects_input.car_config, 0);

		last_num_points = detect_points_above_ground_in_vertical_beam(i, moving_objects_input, velodyne_data, velodyne_params,
				point_clouds, last_num_points);
	}
	return (last_num_points);
}


int
build_point_cloud_using_velodyne_message(carmen_velodyne_partial_scan_message *velodyne_message, sensor_parameters_t *velodyne_params,
		sensor_data_t *velodyne_data, carmen_vector_3D_t *robot_velocity, double phi, moving_objects_input_data_t moving_objects_input,
		carmen_vector_3D_t *point_clouds)
{
	static rotation_matrix *r_matrix_car_to_global = NULL;
	static int velodyne_message_id;
	int current_point_cloud_index;
	int num_points = velodyne_message->number_of_32_laser_shots * velodyne_params->vertical_resolution;
	int last_num_points = 0;

	if (velodyne_data->last_timestamp == 0.0)
	{
		velodyne_data->last_timestamp = velodyne_message->timestamp;
		velodyne_message_id = -2; // correntemente sao necessarias pelo menos 2 mensagens para se ter uma volta completa de velodyne
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
			velodyne_params->range_max);

	if (velodyne_message_id >= 0)
	{
		carmen_pose_3D_t local_pose;

		local_pose.position = moving_objects_input.car_fused_pose.position;
		local_pose.orientation.yaw = moving_objects_input.car_fused_pose.orientation.yaw;
		local_pose.orientation.pitch = local_pose.orientation.roll = 0.0;

		r_matrix_car_to_global = compute_rotation_matrix(r_matrix_car_to_global, local_pose.orientation);
		current_point_cloud_index =  velodyne_data->point_cloud_index;
		last_num_points = detect_points_above_ground(velodyne_params, velodyne_data, r_matrix_car_to_global, &local_pose,
				robot_velocity, 0.0, 0.0, current_point_cloud_index, phi, moving_objects_input, point_clouds);

		if (velodyne_message_id > 1000000)
			velodyne_message_id = 0;
	}
	velodyne_message_id++;
	velodyne_data->last_timestamp = velodyne_message->timestamp;

	return(last_num_points);
}


void
detect_and_follow_moving_objects(carmen_velodyne_partial_scan_message *velodyne_message, sensor_parameters_t *velodyne_params,
sensor_data_t *velodyne_data, carmen_vector_3D_t *robot_velocity, double phi, moving_objects_input_data_t moving_objects_input,
carmen_vector_3D_t *point_clouds)
{

	double timestamp;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	size_num_points = build_point_cloud_using_velodyne_message(velodyne_message, velodyne_params, velodyne_data, robot_velocity,
			phi, moving_objects_input, point_clouds);

	vector_3D_PointCloud_to_pcl_PointCloud_with_subtract_global_pose(point_clouds, size_num_points, moving_objects_input.car_global_pose,
			pcl_cloud, moving_objects_input);

	find_object_point_clouds(size_num_points, pcl_cloud);

	timestamp = velodyne_message->timestamp;
	associate_object_point_clouds_with_previous_point_clouds(pcl_cloud, moving_objects_input.car_global_pose, timestamp);

//	pcl_PointCloud_to_vector_3D_PointCloud_with_sum_global_pose(point_clouds, size_num_points, moving_objects_input.car_global_pose,
//			pcl_cloud, moving_objects_input);

	get_features_objects_from_point_cloud(&list_point_clouds);

//	classify_moving_objects(&list_point_clouds);

	particle_filter_moving_objects_tracking(&list_point_clouds);

}
