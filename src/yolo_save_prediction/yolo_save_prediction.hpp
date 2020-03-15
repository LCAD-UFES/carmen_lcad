//#include <carmen/carmen.h>
//#include <carmen/carmen_darknet_interface.hpp>
//#include <carmen/bumblebee_basic_interface.h>
//#include <carmen/velodyne_interface.h>
//#include <carmen/velodyne_camera_calibration.h>
//#include <carmen/moving_objects_messages.h>
//#include <carmen/moving_objects_interface.h>
//#include <carmen/traffic_light_interface.h>
//#include <carmen/traffic_light_messages.h>
//#include <carmen/rddf_messages.h>
//#include <carmen/laser_ldmrs_utils.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>
#include <fstream>

using namespace std;
using namespace cv;

#include <vector>


struct bbox_t
{
	float prob,x, y, w, h;					// confidence - probability that the object was found correctly
	unsigned int obj_id;		// class of object - from range [0, classes-1]
};


struct image_t
{
	int h;						// height
	int w;						// width
	int c;						// number of chanels (3 - for RGB)
	float *data;				// pointer to the image data
};


char **
get_classes_names(char *classes_names_file);


void*
initialize_YOLO(char *cfg_file_name, char *weights_file_name);

std::vector<bbox_t>
run_YOLO(unsigned char *data, int w, int h, void *net_config, char **classes_names, float threshold=0.5, float hier_thresh=0.5);
