/*
 * neural_car_detector.cpp
 *
 *  Created on: 28 de jul de 2017
 *      Author: luan
 */

#include "neural_car_detector.hpp"


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
