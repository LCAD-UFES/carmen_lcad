#ifndef DBSCAN_H
#define DBSCAN_H

#include "types.h"

typedef std::vector<g2d::Points> Clusters;

/**
 * @brief Implementation of the Density-Based Spatial Clustering of Applications with Noise (DBSCAN).
 *
 * The DBSCAN algorithm clusterizes a point set based on how densely they are packed over an area.
 *
 * @see https://en.wikipedia.org/wiki/DBSCAN
 *
 * @param points Sequence of 2D points to clusterize.
 * @param e Maximum distance between two points for them to be considered part of the same cluster.
 * @param b Minimum local density (i.e. number of neighbors) for a point to be considered part of a cluster.
 *
 * @return Sequence of point sequences, each one representing a disjoint cluster.
 */
Clusters DBSCAN(const g2d::Points &points, g2d::Field e, size_t b);

/**
 * @brief Implementation of Andrew's monotone chain 2D convex hull algorithm.
 *
 * Andrew's monotone chain 2D convex hull algorithm finds the minimum-area convex polygon
 * that encloses a set of points. This implementation takes a cluster sequence as input,
 * returning a sequence of polygons as result.
 *
 * @see https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain
 */
Clusters monotoneChain(const Clusters &clusters);

#endif
