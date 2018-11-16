#ifndef SRC_NEURAL_OBJECT_DETECTOR_NEURAL_OBJECT_DETECTOR_HPP_
#define SRC_NEURAL_OBJECT_DETECTOR_NEURAL_OBJECT_DETECTOR_HPP_

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/velodyne_interface.h>
//#include <carmen/velodyne_camera_calibration.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/moving_objects_interface.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>
#include <fstream>

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

carmen_vector_3D_t
rotate_point(carmen_vector_3D_t point, double theta);


carmen_vector_3D_t
translate_point(carmen_vector_3D_t point, carmen_vector_3D_t offset);

#endif /* SRC_NEURAL_OBJECT_DETECTOR_NEURAL_OBJECT_DETECTOR_HPP_ */
