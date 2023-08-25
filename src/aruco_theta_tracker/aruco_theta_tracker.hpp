#define BOOST_SIGNALS_NO_DEPRECATION_WARNING

#include <vector>
#include <carmen/carmen.h>
#include <aruco/aruco.h>
#include <carmen/camera_drivers_interface.h>
#include <carmen/bumblebee_basic_interface.h>

#include "PoseTracker.hpp"


typedef struct
{
    int camera_id;
    int show_output;
    int detector_id;
    char* camera_name;
    int image_index_to_show;
    std::vector <PoseTracker *> detectors;

    double _fx, _fy, _cu, _cv;
    double _k1, _k2, _p1, _p2, _k3;
} aruco_theta_tracker_parameters;


void 
aruco_theta_tracker_detect(camera_message *msg, aruco_theta_tracker_parameters params, cv::Mat &tvec, cv::Mat &rvec, int &n_markers_detected, double &offset);


void 
aruco_theta_tracker_detect(carmen_bumblebee_basic_stereoimage_message *msg, aruco_theta_tracker_parameters params, cv::Mat &tvec, cv::Mat &rvec, int &n_markers_detected, double &offset);


int 
aruco_theta_tracker_read_parameters(int argc, char **argv, aruco_theta_tracker_parameters &params);