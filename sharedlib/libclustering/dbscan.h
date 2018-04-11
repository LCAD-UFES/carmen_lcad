#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>

namespace dbscan
{

typedef std::vector<carmen_point_t> Cluster;

typedef std::vector<Cluster> Clusters;

/**
 * @brief Split a collection of points into clusters by distance.
 *
 * If a point has at least `density` neighbors with a squared distance no
 * larger than `d2`, then it will either be included on an existing cluster or
 * become the seed for a new one, depending on what is more appropriate. If it
 * doesn't make the density requirement, it may still be added to a cluster if
 * it's close enough to a point already assigned to a cluster (i.e. if its
 * squared distance to it is less than `d2`).
 */
Clusters dbscan(double d2, std::size_t density, const Cluster &points);

} // namespace dbscan

#endif
