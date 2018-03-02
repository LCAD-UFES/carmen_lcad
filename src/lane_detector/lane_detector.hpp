/*
 * lane_detector.hpp
 *
 *  Created on: 28 de fev de 2018
 *      Author: marcelo
 */

#ifndef SRC_LANE_DETECTOR_NEURAL_CAR_DETECTOR_HPP_
#define SRC_LANE_DETECTOR_NEURAL_CAR_DETECTOR_HPP_

#include <carmen/carmen.h>
#include <vector>
#include <string>

#include "dbscan.h"

enum class carmen_moving_object_type
{
    car = 0,
    bus,
    pedestrian,
    other
} ;

typedef struct
{
    std::vector<carmen_vector_3D_t> points; /*< pontos que compõe o cluster*/
    double orientation;
    double linear_velocity;
    int track_id; /*< usado para fazer o tracking */
    double last_detection_timestamp;
    carmen_moving_object_type cluster_type;
} carmen_tracked_cluster_t, *carmen_tracked_cluster_p;

dbscan::Cluster
generate_cluster(std::vector<carmen_vector_3D_t> points);

dbscan::Cluster
get_biggest_cluster(dbscan::Clusters clusters);

std::vector<carmen_vector_3D_t>
get_carmen_points(dbscan::Cluster cluster);

carmen_moving_object_type
find_cluster_type_by_obj_id(const std::vector<std::string> &object_names, int obj_id);

carmen_vector_3D_t
rotate_point(carmen_vector_3D_t point, double theta);

carmen_vector_3D_t
translate_point(carmen_vector_3D_t point, carmen_vector_3D_t offset);

void
filter_points_in_clusters(std::vector<std::vector<carmen_vector_3D_t> > *cluster_list);

#endif /* SRC_LANE_DETECTOR_NEURAL_CAR_DETECTOR_HPP_ */
