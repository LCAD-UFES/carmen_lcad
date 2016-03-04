/*********************************************************
	---  Moving Objects Module ---
**********************************************************/

#include "moving_objects.h"
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


std::list<color_palette_and_association_data_t> color_palette_and_association;
/* List of point clouds used for association */
std::list<object_point_cloud_data_t> list_previous_point_clouds;
std::vector<object_model_features_t> object_models;
int num_of_models;

int first_associate_object_point_clouds_flag = 1;


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
	int total_color = 510;//255; //3060; // proportional to 255(r,g,b)
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
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
				cloud_cluster->points.push_back(cloud->points[*pit]);

			cloud_cluster->width    = cloud_cluster->points.size();
			cloud_cluster->height   = 1;
			cloud_cluster->is_dense = true;
		}
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
	int num_color_associate;


	for (std::list<object_point_cloud_data_t>::iterator it = list_point_clouds.begin(); it != list_point_clouds.end(); )
	{
		centroid = it->centroid;
		associated_car_global_pose_point_cloud.position.x = it->car_global_pose.position.x;
		associated_car_global_pose_point_cloud.position.y = it->car_global_pose.position.y;
		associated_car_global_pose_point_cloud.position.z = it->car_global_pose.position.z;

		double distance = distance_between_3d_point_and_car_global_pose(centroid, car_global_pose, associated_car_global_pose_point_cloud);
		it->distance_object_pose_and_car_global_pose = distance;
		if (distance >= threshold_max_dist_from_car)
		{
			num_color_associate = it->num_color_associate;
			decrement_color_palette_and_association(num_color_associate);
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

			aux_objects_data.point_cloud = it->point_cloud;
			aux_objects_data.num_color_associate = num_color_associate;
			aux_objects_data.label_associate = it->label_associate;
			aux_objects_data.car_global_pose = it->car_global_pose;
			aux_objects_data.centroid = it->centroid;
			aux_objects_data.orientation = it->orientation;
			aux_objects_data.delta_time_t_and_t_1 = 0.0;
			aux_objects_data.distance_object_pose_and_car_global_pose = it->distance_object_pose_and_car_global_pose;
			aux_objects_data.geometric_model = it->geometric_model;
			aux_objects_data.geometry = it->geometry;
			aux_objects_data.linear_velocity = it->linear_velocity;
			aux_objects_data.timestamp = it->timestamp;
			aux_objects_data.object_pose.position.x = it->centroid[0];
			aux_objects_data.object_pose.position.y = it->centroid[1];
			aux_objects_data.object_pose.position.z = it->centroid[2];

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


std::list<object_point_cloud_data_t>
get_current_list_point_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud_ptr,
		vector<pcl::PointIndices> cluster_indices, carmen_pose_3D_t car_global_pose, double timestamp)
{
	std::list<object_point_cloud_data_t> list_point_clouds;
	pcl::PointCloud<pcl::PointXYZ>::Ptr aux_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f centroid;
	object_point_cloud_data_t aux_objects_data;
	Eigen::Vector4f min_point;
	Eigen::Vector4f max_point;

	double d_x, d_y, d_z;

	for (size_t i = 0; i < cluster_indices.size(); i++)
	{
		set_point_cloud_cluster_i_from_point_cloud(i, aux_pcl_cloud, pcl_point_cloud_ptr, cluster_indices);
		/*** PROPOSAL XY POSE OF POINT CLOUD ***/
//		pcl::compute3DCentroid(*aux_pcl_cloud, centroid);
		/* "ponto central" ou suposto centro de massa da bounding box: */
		pcl::getMinMax3D(*aux_pcl_cloud, min_point, max_point);
		centroid[0] = (max_point[0] + min_point[0]) / 2.0;
		centroid[1] = (max_point[1] + min_point[1]) / 2.0;
		centroid[2] = (max_point[2] + min_point[2]) / 2.0;

//		if (is_point_cloud_the_current_autonomous_vehicle(centroid, 4.437, 2.065, 1.720, 0.909, 0.280, car_global_pose.orientation.yaw))
//			continue;

		d_x = max_point[0] - min_point[0];
		d_y = max_point[1] - min_point[1];
		d_z = max_point[2] - min_point[2];

		aux_objects_data.distance_object_pose_and_car_global_pose = 0.0;
		aux_objects_data.point_cloud            = *aux_pcl_cloud;
		aux_objects_data.num_color_associate    = -1; //-1 = not associated
		aux_objects_data.label_associate        = 0;
		aux_objects_data.car_global_pose        = car_global_pose;
		aux_objects_data.centroid               = centroid;
		aux_objects_data.linear_velocity        = 0.0;
		aux_objects_data.delta_time_t_and_t_1   = 0.0;
		aux_objects_data.object_pose.position.x = centroid[0];
		aux_objects_data.object_pose.position.y = centroid[1];
		aux_objects_data.object_pose.position.z = centroid[2];
		aux_objects_data.geometry.length        = fabs(d_x);
		aux_objects_data.geometry.width         = fabs(d_y);
		aux_objects_data.geometry.height        = fabs(d_z);
		aux_objects_data.geometric_model        = -1;
		aux_objects_data.orientation            = 0.0;
		aux_objects_data.object_density         = 0.0;
		aux_objects_data.timestamp              = timestamp;

		aux_objects_data.particle_set.resize(num_of_particles);

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
	double delta_x, delta_y;

	carmen_vector_3D_t current_point_global_pose;
	carmen_vector_3D_t previous_point_global_pose;

	current_point_global_pose.x = current_point[0] + c_associated_car_global_pose_point_cloud.position.x;
	current_point_global_pose.y = current_point[1] + c_associated_car_global_pose_point_cloud.position.y;

	previous_point_global_pose.x = previous_point[0] + p_associated_car_global_pose_point_cloud.position.x;
	previous_point_global_pose.y = previous_point[1] + p_associated_car_global_pose_point_cloud.position.y;


	delta_x = current_point_global_pose.x - previous_point_global_pose.x;
	delta_y = current_point_global_pose.y - previous_point_global_pose.y;

	double orientation = atan2(delta_y, delta_x); //returns in range [-pi,pi]

	return(orientation);
}


void
associate_point_clouds_by_centroids_distance(std::list<object_point_cloud_data_t> &list_point_clouds,
		std::list<object_point_cloud_data_t> &list_current_point_clouds)
{
	double distance_3d;
	double distance_2d;
	object_point_cloud_data_t aux_objects_data;
	double delta_time;
	double orientation;


	for (std::list<object_point_cloud_data_t>::iterator it = list_current_point_clouds.begin();
			it != list_current_point_clouds.end(); it++)
	{
		double min_dist = 99999.0;
		int min_dist_reading = -1;

		distance_2d = 99999.0;
		orientation = 0.0;

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
					orientation = orientation_by_displacement_between_two_points(it->centroid, it->car_global_pose, pit->centroid, pit->car_global_pose);

					pit->label_associate = 1;
					pit->linear_velocity = calculate_velocity_of_the_object(distance_2d, delta_time);
					pit->delta_time_t_and_t_1 = delta_time;
					pit->orientation = orientation;
					pit->car_global_pose = it->car_global_pose;
					pit->centroid = it->centroid;
					pit->point_cloud = it->point_cloud;
					pit->distance_object_pose_and_car_global_pose = it->distance_object_pose_and_car_global_pose;
					pit->geometric_model = it->geometric_model;
					pit->geometry = it->geometry;
					pit->object_density = it->object_density;
					pit->timestamp = it->timestamp;
					pit->object_pose.position.x = it->centroid[0];
					pit->object_pose.position.y = it->centroid[1];
					pit->object_pose.position.z = it->centroid[2];
					it->num_color_associate = min_dist_reading;
					break;
				}
			}
		}
		else
			it->num_color_associate = -1;
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
			sum += pow ((z - average_height), 2);
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
	double map_x_origin   = occupancy_grid_map->config.x_origin;
	double map_y_origin   = occupancy_grid_map->config.y_origin;
	double map_x_size     = occupancy_grid_map->config.x_size;
	double map_y_size     = occupancy_grid_map->config.y_size;

	for (std::list<object_point_cloud_data_t>::iterator it = list_current_point_clouds.begin(); it != list_current_point_clouds.end();)
	{
		int points_in_occupied_grid = 0;
		bool off_limits = false;
		for (pcl::PointCloud<pcl::PointXYZ>::iterator pit = it->point_cloud.begin(); pit != it->point_cloud.end(); pit++)
		{
			int x = carmen_round((pit->x + it->car_global_pose.position.x - map_x_origin)/map_resolution);
			int y = carmen_round((pit->y + it->car_global_pose.position.y - map_y_origin)/map_resolution);

			/* Check point cloud off limits of gridmap */
			if (x < 0 || y < 0 || x > map_x_size || y > map_y_size)
			{
				off_limits = true;
				break;
			}

			double occupancy_rate = occupancy_grid_map->map[x][y];

			if (occupancy_rate >= threshold_occupancy_rate)
				points_in_occupied_grid++;
		}

		if (points_in_occupied_grid/it->point_cloud.size() >= threshold_points_in_occupied_grid_rate)
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
associate_object_point_clouds(std::list<object_point_cloud_data_t> &list_point_clouds,
		std::list<object_point_cloud_data_t> &list_current_point_clouds, carmen_map_p &occupancy_grid_map,
		moving_objects_input_data_t moving_objects_input)
{
	filter_curbs_point_cloud(list_current_point_clouds);
	filter_static_objects(list_current_point_clouds, occupancy_grid_map, moving_objects_input);
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


std::list<object_point_cloud_data_t>
association_list_point_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr, carmen_pose_3D_t car_global_pose,
		std::vector<pcl::PointIndices> cluster_indices, double timestamp, carmen_map_p & occupancy_grid_map,
		moving_objects_input_data_t moving_objects_input)
{
	std::list<object_point_cloud_data_t> list_current_point_clouds;
	std::list<object_point_cloud_data_t> list_point_clouds;

	list_current_point_clouds = get_current_list_point_clouds(pcl_cloud_ptr, cluster_indices, car_global_pose, timestamp);
	associate_object_point_clouds(list_point_clouds, list_current_point_clouds, occupancy_grid_map, moving_objects_input);
	include_unassociated_objects_point_clouds(list_point_clouds, list_current_point_clouds);
	exclude_unecessary_objects_from_point_clouds(list_point_clouds, car_global_pose);

	return list_point_clouds;
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
		it->geometry.width  = fabs(d_y);
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

//		pcl::computeCovarianceMatrix (it->point_cloud, xyz_centroid, covariance_matrix);
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


particle_datmo
select_best_particle(std::vector<particle_datmo> &particle_set_t)
{
	double max_weight = 0.0;
	particle_datmo best_particle;

	for (std::vector<particle_datmo>::iterator it = particle_set_t.begin(); it != particle_set_t.end(); it++)
	{
		if (it->weight >= max_weight)
		{
			max_weight = it->weight;
			best_particle = *it;
		}
	}
	return(best_particle);
}


object_geometry_t
get_geom_based_on_class(int particle_class) {
	object_geometry_t obj_model;
	if (particle_class < 0 || particle_class > int(object_models.size()))
	{
		obj_model.width  = 0.0;
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
	obj_model.geometry.width  = 0.0;
	obj_model.geometry.length = 0.0;
	obj_model.geometry.height = 0.0;
	obj_model.red   = 0.0;
	obj_model.green = 0.0;
	obj_model.blue  = 0.0;
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


particle_datmo
compute_average_state_and_update_timestamp(std::vector<particle_datmo> &particle_set_t, double timestamp)
{
	particle_datmo mean_particle;
	double total_weight = 0.0;
	double mean_x, mean_y, mean_theta_x, mean_theta_y, mean_velocity;

	/* most frequent object class */
	std::vector<int> vec_class_counter(num_of_models);
	std::vector<int>::iterator most_frequent;

	/* compute mean particle pose */
	mean_x        = 0.0;
	mean_y        = 0.0;
	mean_theta_x  = 0.0;
	mean_theta_y  = 0.0;
	mean_velocity = 0.0;

	for (std::vector<particle_datmo>::iterator it = particle_set_t.begin(); it != particle_set_t.end(); it++)
	{
		mean_x        += it->pose.x * it->weight;
		mean_y        += it->pose.y * it->weight;
		mean_theta_x  += cos(it->pose.theta) * it->weight;
		mean_theta_y  += sin(it->pose.theta) * it->weight;
		mean_velocity += it->velocity * it->weight;
		total_weight  += it->weight;

		vec_class_counter[it->class_id] += 1;
		it->timestamp = timestamp;
	}

	mean_particle.pose.x     = mean_x/total_weight;
	mean_particle.pose.y     = mean_y/total_weight;
	mean_particle.pose.theta = carmen_normalize_theta(atan2(mean_theta_y, mean_theta_x));
	mean_particle.velocity   = mean_velocity/total_weight;

	most_frequent = max_element(vec_class_counter.begin(), vec_class_counter.end());
	mean_particle.class_id = most_frequent - vec_class_counter.begin();
	mean_particle.model_features = get_obj_model_features(mean_particle.class_id);

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
init_particle_set(std::vector<particle_datmo> &particle_set, double x, double y, double timestamp)
{
	for (int i = 0; i < num_of_particles; i++)
	{
		particle_datmo particle_t_1;
		particle_t_1.pose.x = x;
		particle_t_1.pose.y = y;
		particle_t_1.pose.theta = carmen_uniform_random(-M_PI, M_PI);//carmen_normalize_theta(it->orientation + carmen_gaussian_random(0.0, M_PI));
		particle_t_1.velocity = carmen_uniform_random(0.0, 25.0);//it->linear_velocity;//
		particle_t_1.weight = (1.0 / double(num_of_particles));
		particle_t_1.class_id = get_random_model_id(num_of_models);
		particle_t_1.model_features = get_obj_model_features(particle_t_1.class_id);
		particle_t_1.timestamp = timestamp;
		particle_set.push_back(particle_t_1);
	}
}


void
update_object_pose_and_features(object_point_cloud_data_t &object_point_cloud, particle_datmo particle_ref,
		std::vector<particle_datmo> particle_set)
{
	object_point_cloud.object_pose.position.x = particle_ref.pose.x;
	object_point_cloud.object_pose.position.y = particle_ref.pose.y;
	object_point_cloud.orientation = particle_ref.pose.theta;
	object_point_cloud.linear_velocity = particle_ref.velocity;
	object_point_cloud.geometric_model = particle_ref.class_id;
	object_point_cloud.model_features = particle_ref.model_features;
	object_point_cloud.particle_set = particle_set;
}


void
remove_obj_model_features(object_point_cloud_data_t &object_point_cloud)
{
	object_point_cloud.geometric_model = -1;
	clear_obj_model_features(object_point_cloud.model_features);
}


void
particle_filter_moving_objects_tracking(std::list<object_point_cloud_data_t> &list_point_clouds)
{
	double x, y;
	double delta_time;
	particle_datmo particle_reference;

	for (std::list<object_point_cloud_data_t>::iterator it = list_point_clouds.begin(); it != list_point_clouds.end(); it++)
	{
		/* object global pose */
		x = it->object_pose.position.x + it->car_global_pose.position.x;
		y = it->object_pose.position.y + it->car_global_pose.position.y;

		if (it->particle_set.size() == 0)
		{
			/* INITIALIZE PARTICLES */
			init_particle_set(it->particle_set, x, y, it->timestamp);
		}
		else
		{
			if (it->particle_set[0].timestamp != it->timestamp)
			{
				std::vector<particle_datmo> particle_set_t_1 = it->particle_set;
				std::vector<particle_datmo> particle_set_t;

				delta_time = it->timestamp - it->particle_set[0].timestamp;

				/* KNOWN ISSUE: Car global position included due to lost of precision problem with PCL point types */
				particle_set_t = algorithm_monte_carlo(particle_set_t_1, x, y, delta_time, it->point_cloud,
						it->geometry, it->car_global_pose.position);

				particle_reference = compute_average_state_and_update_timestamp(particle_set_t, it->timestamp);

				update_object_pose_and_features(*it, particle_reference, particle_set_t);

				if (it->linear_velocity < threshold_min_velocity)
				{
					remove_obj_model_features(*it);
					continue;
				}

				// Count points below 0.9m
				int count_down_points = 0;
				for (pcl::PointCloud<pcl::PointXYZ>::const_iterator pit = it->point_cloud.begin(); pit != it->point_cloud.end(); pit++)
					if (pit->z < 0.90)
						count_down_points += 1;

				// PointCloud diagonal measurement and density
				double diagonal_measurement = get_object_3d_diagonal_measurement(*it);
				double density = get_object_density_by_area(*it);

				if (density < 10.0 || count_down_points < 35 || (diagonal_measurement < 1.0 || diagonal_measurement > 12.0))
				{
					remove_obj_model_features(*it);
					continue;
				}
			}
			else
			{
				remove_obj_model_features(*it);
			}
		}
	}
}


void
convert_carmen_vector_3d_to_pcl_point_subtracting_global_pose(carmen_vector_3D_t *cloud, int point_cloud_size,
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud_ptr, moving_objects_input_data_t moving_objects_input)
{
	if (moving_objects_input.first_offline_map_message == -1)// Agora a global pos eh valida
	{
		for (int k = 0; k < point_cloud_size; k++)
		{
			pcl::PointXYZ pcl_point_3D;

			pcl_point_3D.x = cloud[k].x - moving_objects_input.car_global_pose.position.x;
			pcl_point_3D.y = cloud[k].y - moving_objects_input.car_global_pose.position.y;
			pcl_point_3D.z = cloud[k].z - moving_objects_input.car_global_pose.position.z;

			pcl_point_cloud_ptr->push_back(pcl_point_3D);
		}
	}
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
			moving_objects_input.highest_sensor, moving_objects_input.safe_range_above_sensors, 1);

	for (int k = 0; k < velodyne_params->vertical_resolution; k++)
	{
		if ((velodyne_data->obstacle_height[k] >= 0.5) && (velodyne_data->obstacle_height[k] <= MAXIMUM_HEIGHT_OF_OBSTACLE))
		{
			point_clouds[last_num_points].x = velodyne_data->ray_position_in_the_floor[k].x;
			point_clouds[last_num_points].y = velodyne_data->ray_position_in_the_floor[k].y;
			point_clouds[last_num_points].z = velodyne_data->obstacle_height[k];
			last_num_points++;
		}
		else if (velodyne_data->occupancy_log_odds_of_each_ray_target[k] > velodyne_params->log_odds.log_odds_l0
						&& velodyne_data->obstacle_height[k] <= MAXIMUM_HEIGHT_OF_OBSTACLE)
		{
			point_clouds[last_num_points].x = velodyne_data->ray_position_in_the_floor[k].x;
			point_clouds[last_num_points].y = velodyne_data->ray_position_in_the_floor[k].y;
			point_clouds[last_num_points].z = velodyne_data->obstacle_height[k];
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
set_model(object_model_features_t &obj_model, int model_id, double width, double length, double height, double red, double green, double blue)
{
	obj_model.model_id = model_id;
	obj_model.geometry.width = width;
	obj_model.geometry.length = length;
	obj_model.geometry.height = height;
	obj_model.red = red;
	obj_model.green = green;
	obj_model.blue = blue;
}


void
set_object_models(std::vector<object_model_features_t> &obj_models)
{
	/* Currently included objects:
	 * 0) sedan;
	 * 1) pedestrian;
	 * 2) bike/motorbike;
	 * 3) small truck;
	 * 4) signs/posts (wrongly classified) */
	object_model_features_t obj_class;

	/* 0) sedan */
	set_model(obj_class, 0, 2.0, 4.5, 1.48, 1.0, 0.0, 0.8);
	obj_models.push_back(obj_class);

	/* 1) pedestrian */
	set_model(obj_class, 1, 0.9, 0.9, 1.7, 0.0, 1.0, 0.0);
	obj_models.push_back(obj_class);

	/* 2) bike/motorbike */
	set_model(obj_class, 2, 0.8, 2.3, 1.6, 0.0, 1.0, 1.0);
	obj_models.push_back(obj_class);

	/* 3) small truck */
	set_model(obj_class, 3, 2.5, 5.0, 2.3, 0.5, 0.5, 1.0);
	obj_models.push_back(obj_class);

	/* 4) signs/posts (wrongly classified) */
	set_model(obj_class, 4, 0.5, 0.5, 2.0, 0.8, 0.0, 1.0);
	obj_models.push_back(obj_class);

	num_of_models = int(obj_models.size());
}

std::list<object_point_cloud_data_t>
detect_and_follow_moving_objects(carmen_velodyne_partial_scan_message *velodyne_message, sensor_parameters_t *velodyne_params,
		sensor_data_t *velodyne_data, carmen_vector_3D_t *robot_velocity, double phi,
		moving_objects_input_data_t moving_objects_input, carmen_vector_3D_t *carmen_vector_3d_point_cloud,
		carmen_map_p & occupancy_grid_map)
{
	int size_of_point_cloud = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::PointIndices> cluster_indices;
	std::list<object_point_cloud_data_t> list_point_clouds;

	if (object_models.empty())
	{
		set_object_models(object_models);
	}

	/*** GET POINTS FROM LASER SCAN ***/
	size_of_point_cloud = build_point_cloud_using_velodyne_message(velodyne_message, velodyne_params, velodyne_data,
			robot_velocity, phi, moving_objects_input, carmen_vector_3d_point_cloud);

	/*** CONVERT TO PCL POINT CLOUD FORMAT SUBTRACTING GLOBAL POSE ***/
	convert_carmen_vector_3d_to_pcl_point_subtracting_global_pose(carmen_vector_3d_point_cloud, size_of_point_cloud,
			pcl_cloud_ptr, moving_objects_input);

	/*** SEGMENT POINT CLOUDS - RETURNS CLUSTER INDICES ***/
	cluster_indices = find_objects_in_point_clouds(pcl_cloud_ptr);

	/*** ASSOCIATE AND CONFIGURE POINT CLOUDS ***/
	list_point_clouds = association_list_point_clouds(pcl_cloud_ptr, moving_objects_input.car_global_pose, cluster_indices,
			velodyne_message->timestamp, occupancy_grid_map, moving_objects_input);

	/*** PARTICLE FILTER FOR DATMO ***/
	particle_filter_moving_objects_tracking(list_point_clouds);

	/* Set the global variable list_previous_point_clouds for next iteration */
	set_association_list_point_clouds(list_point_clouds);

	return list_point_clouds;
}
