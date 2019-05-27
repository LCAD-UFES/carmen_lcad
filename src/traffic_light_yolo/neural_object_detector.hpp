#ifndef SRC_NEURAL_OBJECT_DETECTOR_NEURAL_OBJECT_DETECTOR_HPP_
#define SRC_NEURAL_OBJECT_DETECTOR_NEURAL_OBJECT_DETECTOR_HPP_

#include <carmen/carmen.h>
#include <carmen/carmen_darknet_interface.hpp>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_camera_calibration.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/traffic_light_interface.h>
#include <carmen/traffic_light_messages.h>
#include <carmen/rddf_messages.h>
#include <carmen/laser_ldmrs_utils.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>
#include <fstream>

using namespace std;
using namespace cv;
using namespace cv::ml;


carmen_vector_3D_t
rotate_point(carmen_vector_3D_t point, double theta);


carmen_vector_3D_t
translate_point(carmen_vector_3D_t point, carmen_vector_3D_t offset);


#endif /* SRC_NEURAL_OBJECT_DETECTOR_NEURAL_OBJECT_DETECTOR_HPP_ */
