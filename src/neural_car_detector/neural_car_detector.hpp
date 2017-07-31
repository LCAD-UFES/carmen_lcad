/*
 * neural_car_detector.hpp
 *
 *  Created on: 28 de jul de 2017
 *      Author: luan
 */

#ifndef SRC_NEURAL_CAR_DETECTOR_NEURAL_CAR_DETECTOR_HPP_
#define SRC_NEURAL_CAR_DETECTOR_NEURAL_CAR_DETECTOR_HPP_

#include <carmen/carmen.h>
#include <vector>

#include "dbscan.h"


dbscan::Cluster
generate_cluster(std::vector<carmen_vector_3D_t> points);

dbscan::Cluster
get_biggest_cluster(dbscan::Clusters clusters);

std::vector<carmen_vector_3D_t>
get_carmen_points(dbscan::Cluster cluster);

carmen_vector_3D_t
rotate_point(carmen_vector_3D_t point, double theta);

carmen_vector_3D_t
translate_point(carmen_vector_3D_t point, carmen_vector_3D_t offset);

void
filter_points_in_clusters(std::vector<std::vector<carmen_vector_3D_t> > *cluster_list);

#endif /* SRC_NEURAL_CAR_DETECTOR_NEURAL_CAR_DETECTOR_HPP_ */
