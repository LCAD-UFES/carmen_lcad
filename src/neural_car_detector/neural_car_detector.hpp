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

#endif /* SRC_NEURAL_CAR_DETECTOR_NEURAL_CAR_DETECTOR_HPP_ */
