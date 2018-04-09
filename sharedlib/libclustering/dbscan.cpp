#include <carmen/carmen.h>
#include "dbscan.h"

using std::size_t;

namespace dbscan
{

typedef std::vector<int> Indexes;


// The squared distance between points a and b.
inline double
distance2(const carmen_point_t &a, const carmen_point_t &b)
{
	double dx = a.x - b.x;
	double dy = a.y - b.y;

	return (dx * dx + dy * dy);
}


Indexes
query(double d2, int i, const Cluster &points, std::vector<bool> clustered)
{
	Indexes neighbors;
	const carmen_point_t &point = points[i];
	for (size_t j = 0; j < points.size(); j++)
	{
		if ((distance2(point, points[j]) < d2) && !clustered[j])
			neighbors.push_back(j);
	}

	return (neighbors);
}


Clusters
dbscan(double d2, size_t density, const Cluster &points)
{
	Clusters clusters;
	std::vector<bool> clustered(points.size(), false);
	for (size_t i = 0; i < points.size(); ++i)
	{
		// Ignore already clustered points.
		if (clustered[i])
			continue;

		// Ignore points without enough neighbors.
		Indexes neighbors = query(d2, i, points, clustered);
		if (neighbors.size() < density)
			continue;

		// Create a new cluster with the i-th point as its first element.
		clusters.push_back(Cluster());
		Cluster &cluster = clusters.back();
		cluster.push_back(points[i]);
		clustered[i] = true;

		// Add the point's neighbors (and possibly their neighbors) to the cluster.
		for (size_t j = 0; j < neighbors.size(); ++j)
		{
			int k = neighbors[j];
			if (clustered[k])
				continue;

			cluster.push_back(points[k]);
			clustered[k] = true;

			Indexes farther = query(d2, k, points, clustered);
			if (farther.size() >= density)
				neighbors.insert(neighbors.end(), farther.begin(), farther.end());
		}
	}

	return (clusters);
}

} // namespace dbscan
