#ifndef SRC_NEURAL_OBJECT_DETECTOR_NEURAL_OBJECT_DETECTOR_HPP_
#define SRC_NEURAL_OBJECT_DETECTOR_NEURAL_OBJECT_DETECTOR_HPP_

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_camera_calibration.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/moving_objects_interface.h>
#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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

struct bbox {
	unsigned int x, y, w, h;	// (x,y) - top-left corner, (w, h) - width & height of bounded box
	float prob;					// confidence - probability that the object was found correctly
	unsigned int obj_id;		// class of object - from range [0, classes-1]
	unsigned int track_id;		// tracking id for video (0 - untracked, 1 - inf - tracked object)
};

carmen_vector_3D_t
rotate_point(carmen_vector_3D_t point, double theta);


carmen_vector_3D_t
translate_point(carmen_vector_3D_t point, carmen_vector_3D_t offset);

#endif /* SRC_NEURAL_OBJECT_DETECTOR_NEURAL_OBJECT_DETECTOR_HPP_ */
