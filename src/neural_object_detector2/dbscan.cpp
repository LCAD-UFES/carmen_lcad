#include <carmen/carmen.h>
#include "dbscan.h"

using std::size_t;

namespace dbscan
{

typedef std::vector<int> Indexes;

// The squared distance between points a and b.
inline double distance2(const Point &a, const Point &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return dx * dx + dy * dy;
}

Indexes query(double d2, int i, const Cluster &points) {
    Indexes neighbors;
    const Point &point = points[i];
    for (int j = 0, n = points.size(); j < n; ++j) {
        if (distance2(point, points[j]) < d2) {
            neighbors.push_back(j);
        }
    }

    return neighbors;
}


Clusters DBSCAN(double d2, size_t density, const Cluster &points) {
    Clusters clusters;
    int n = points.size();
    std::vector<bool> clustered(n, false);
    for (int i = 0; i < n; ++i) {
        // Ignore already clustered points.
        if (clustered[i]) {
            continue;
        }

        // Ignore points without enough neighbors.
        Indexes neighbors = query(d2, i, points);
        if (neighbors.size() < density) {
            continue;
        }

        // Create a new cluster with the i-th point as its first element.
        clusters.push_back(Cluster());
        Cluster &cluster = clusters.back();
        cluster.push_back(points[i]);
        clustered[i] = true;

        // Add the point's neighbors (and possibly their neighbors) to the cluster.
        for (size_t j = 0; j < neighbors.size(); ++j) {
            int k = neighbors[j];
            if (clustered[k]) {
                continue;
            }

            cluster.push_back(points[k]);
            clustered[k] = true;

            Indexes farther = query(d2, k, points);
            if (farther.size() >= density) {
                neighbors.insert(neighbors.end(), farther.begin(), farther.end());
            }
        }
    }

    return clusters;
}

} // end of namespace dbscan


double distance(carmen_vector_3D_t a, carmen_vector_3D_t b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return dx * dx + dy * dy;
}

vector<int>
query2(double max_distance, int point_index, vector<carmen_vector_3D_t> &points)
{
	vector<int> neighbors;
	carmen_vector_3D_t point = points[point_index];

    for (int j = 0, size = points.size(); j < size; ++j)
    {
        if (distance(point, points[j]) < max_distance)  // TODO Trocar por uma fincao do global
            neighbors.push_back(j);
    }

    return neighbors;
}


vector<vector<carmen_vector_3D_t>>
dbscan_compute_clusters(double max_distance, int density, vector<carmen_vector_3D_t> &points)
{
	vector<vector<carmen_vector_3D_t>> clusters;
    int num_points = points.size();
    vector<bool> clustered(num_points, false);

    for (int i = 0; i < num_points; ++i)
    {
        // Ignore already clustered points.
        if (clustered[i])
            continue;

        // Ignore points without enough neighbors.
        vector<int> number_of_neighbors = query2(max_distance, i, points);
        if (number_of_neighbors.size() < density)
            continue;

        clusters.push_back(vector<carmen_vector_3D_t>());
        vector<carmen_vector_3D_t> &cluster = clusters.back();
        cluster.push_back(points[i]);

        clustered[i] = true;

        // Add the point's neighbors (and possibly their neighbors) to the cluster.
        for (size_t j = 0; j < number_of_neighbors.size(); ++j)
        {
            int k = number_of_neighbors[j];

            if (clustered[k])
                continue;

            cluster.push_back(points[k]);
            clustered[k] = true;

            vector<int> farther = query2(max_distance, k, points);
            if (farther.size() >= density)
            	number_of_neighbors.insert(number_of_neighbors.end(), farther.begin(), farther.end());
        }
    }
    return clusters;
}
