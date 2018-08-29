#include "neural_object_detector2.hpp"


dbscan::Cluster
generate_cluster(std::vector<carmen_vector_3D_t> points)
{
	dbscan::Cluster cluster;
	for (unsigned int i = 0; i < points.size(); i++)
	{
		dbscan::Point point;
		point.x = points[i].x;
		point.y = points[i].y;
		cluster.push_back(point);
	}
	return (cluster);
}


dbscan::Cluster
get_biggest_cluster(dbscan::Clusters clusters)
{
	unsigned int max_size, max_index;
	max_size = 0;
	max_index = 0;
	for (unsigned int i = 0; i < clusters.size(); i++)
	{
		if (clusters[i].size() > max_size)
		{
			max_size = clusters[i].size();
			max_index = i;
		}
	}
	return (clusters[max_index]);
}


std::vector<carmen_vector_3D_t>
get_carmen_points(dbscan::Cluster cluster)
{
	std::vector<carmen_vector_3D_t> points;
	for (unsigned int i = 0; i < cluster.size(); i++)
	{
		carmen_vector_3D_t p;
		p.x = cluster[i].x;
		p.y = cluster[i].y;
		p.z = 0.0;
		points.push_back(p);
	}
	return (points);
}


carmen_moving_object_type
find_cluster_type_by_obj_id(const std::vector<std::string> &object_names, int obj_id)
{
    std::string obj_name;

    if(object_names.size() > obj_id)
        obj_name = object_names[obj_id];

    if(obj_name.compare("car") == 0)
        return carmen_moving_object_type::car;

    if(obj_name.compare("bus") == 0)
        return carmen_moving_object_type::bus;

    if(obj_name.compare("person") == 0)
        return carmen_moving_object_type::pedestrian;

    return carmen_moving_object_type::other;

}

carmen_vector_3D_t
translate_point(carmen_vector_3D_t point, carmen_vector_3D_t offset)
{
	point.x += offset.x;
	point.y += offset.y;
	point.z += offset.z;
	return (point);
}


carmen_vector_3D_t
rotate_point(carmen_vector_3D_t point, double theta)
{
	carmen_vector_3D_t p;
	p.x = point.x * cos(theta) - point.y * sin(theta);
	p.y = point.x * sin(theta) + point.y * cos(theta);
	p.z = point.z;
	return (p);
}


void
filter_points_in_clusters(std::vector<std::vector<carmen_vector_3D_t> > *cluster_list)
{
	for (unsigned int i = 0; i < cluster_list->size(); i++)
	{
		if ((*cluster_list)[i].size() > 0)
		{
			dbscan::Cluster cluster = generate_cluster((*cluster_list)[i]);  // Create vector in dbscan point type

			dbscan::Clusters clusters = dbscan::DBSCAN(0.5, 5, cluster);     // Compute clusters using dbscan


//			vector<vector<carmen_vector_3D_t>> converted = dbscan_compute_clusters(0.5, 5, (*cluster_list)[i]);
//			printf("L %d R %d \n", (int)clusters.size(), (int)converted.size());
//
//			for (int i = 0; i < clusters.size() && i < converted.size(); i++)
//			{
//				for (int j = 0; j < clusters[i].size() && i < converted[i].size(); j++)
//				{
//					printf("%f\n", clusters[i][j].x - converted[i][j].x);
//				}
//			}


			if (clusters.size() > 0)
			{
				cluster = get_biggest_cluster(clusters);
				(*cluster_list)[i] = get_carmen_points(cluster);
			}
		}
	}
}

