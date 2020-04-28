#ifndef ROUTE_PLANNER_UTILS_H_
#define ROUTE_PLANNER_UTILS_H_

#include <carmen/carmen.h>
#include <carmen/grid_mapping.h>
#include <carmen/rddf_interface.h>
#include <iostream>
#include <list>
#include <string>
#include <queue>
#include <list>
#include <utility>
#include <string.h>
#include <dirent.h>
#include <string>
#include <vector>
#include <GL/glut.h>
//#include "../rddf/rddf_util.h"
#include <carmen/rddf_messages.h>
#include <carmen/rddf_util.h>
#include <carmen/road_network_generator_utils.h>
#include <carmen/carmen_gps.h>
#include <carmen/gps_xyz_interface.h>
#include <Python.h>

#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION == 3
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/flann/miniflann.hpp>
//#include <opencv2/tracking.hpp>
#else
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

using namespace std;
using namespace cv;

void call_osmnx_python_func (Gdc_Coord_3d origin_gdc, Gdc_Coord_3d destination_gdc);
carmen_point_t set_destination(vector<carmen_annotation_t> annotations);
void get_annotation_from_rddf(char *allrddf, vector<carmen_annotation_t> &annotations);

#ifdef __cplusplus
}
#endif

#endif /* ROUTE_PLANNER_H_ */
