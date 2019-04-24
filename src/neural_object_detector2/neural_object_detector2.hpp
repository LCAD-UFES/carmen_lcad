#ifndef SRC_NEURAL_OBJECT_DETECTOR_NEURAL_OBJECT_DETECTOR_HPP_
#define SRC_NEURAL_OBJECT_DETECTOR_NEURAL_OBJECT_DETECTOR_HPP_

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/behavior_selector_interface.h>
#include <carmen/rddf_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_camera_calibration.h>
#include <carmen/camera_boxes_to_world.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/laser_ldmrs_utils.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <vector>
#include <string>

#include "dbscan.h"
#include "SampleFilter_neural_object_detector2.h"


#include <opencv2/highgui/highgui.hpp>

#include <cstdlib>
#include <fstream>
//#include "Darknet.hpp"
#include <carmen/carmen_darknet_interface.hpp>

#define SHOW_DETECTIONS

using namespace std;
using namespace cv;


enum class carmen_moving_object_type
{
    car = 0,
    bus,
    pedestrian,
    other
};


typedef struct
{
    std::vector<carmen_vector_3D_t> points; // pontos que compõe o cluster
    double orientation;
    double linear_velocity;
    int track_id;
    double last_detection_timestamp;
    carmen_moving_object_type cluster_type;
} carmen_tracked_cluster_t, *carmen_tracked_cluster_p;


typedef struct
{
	double translate_factor_x;
	double translate_factor_y;
	double scale_factor_x;
	double scale_factor_y;
}t_transform_factor;


typedef struct
{
	double translate_factor_x;
	double translate_factor_y;
}t_translate_factor;


dbscan::Cluster
generate_cluster(vector<carmen_vector_3D_t> points);


dbscan::Cluster
get_biggest_cluster(dbscan::Clusters clusters);


std::vector<carmen_vector_3D_t>
get_carmen_points(dbscan::Cluster cluster);


carmen_moving_object_type
find_cluster_type_by_obj_id(const vector<string> &object_names, int obj_id);


carmen_vector_3D_t
rotate_point(carmen_vector_3D_t point, double theta);


carmen_vector_3D_t
translate_point(carmen_vector_3D_t point, carmen_vector_3D_t offset);


void
filter_points_in_clusters(vector<vector<carmen_vector_3D_t> > *cluster_list);


#endif /* SRC_NEURAL_OBJECT_DETECTOR_NEURAL_OBJECT_DETECTOR_HPP_ */
