
#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_camera_calibration.h>
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
#include "libefficientdet.h"

using namespace std;
using namespace cv;


int camera;
int camera_side;

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////



void
image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	if (image_msg == NULL)
		return;

	//double fps;
	//static double start_time = 0.0;
	unsigned char *img;

	if (camera_side == 0)
		img = image_msg->raw_left;
	else
		img = image_msg->raw_right;

    double timestamp = image_msg->timestamp;

    Size size(image_msg->width,image_msg->height);
	cv::Mat image_cv = cv::Mat(cv::Size(image_msg->width, image_msg->height), CV_8UC3, img);
    cv::Mat imgResized;
    resize(image_cv, imgResized, size);

	unsigned char *resized_img = imgResized.data;

	vector<bbox_t> predictions = run_EfficientDet(resized_img, image_msg->width, image_msg->height, timestamp);

	//publish_moving_objects_message(image_msg->timestamp, &msg);

}


void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
        carmen_ipc_disconnect();
        cvDestroyAllWindows();

        printf("EfficientDet: Disconnected.\n");
        exit(0);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////


void
subscribe_messages()
{
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);
}

void
read_parameters(int argc, char **argv)
{
	if ((argc != 3))
		carmen_die("%s: Wrong number of parameters. camera_to_efficientdet requires 2 parameter and received %d. \n Usage: %s <camera_number> <camera_side(0-left; 1-right)\n>", argv[0], argc - 1, argv[0]);

	camera = atoi(argv[1]);             // Define the camera to be used
    camera_side = atoi(argv[2]);        // 0 For left image 1 for right image

    int num_items;
    int width, height;

    char bumblebee_string[256];
    char camera_string[256];

    sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera); // Geather the camera ID
    sprintf(camera_string, "%s%d", "camera", camera);

    carmen_param_t param_list[] =
    {
        {bumblebee_string, (char *)"width", CARMEN_PARAM_INT, &width, 0, NULL},
		{bumblebee_string, (char *)"height", CARMEN_PARAM_INT, &height, 0, NULL},
    };

    num_items = sizeof(param_list) / sizeof(param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);
    initialize_Efficientdet(width, height);
}

int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	read_parameters(argc, argv);

	subscribe_messages();

	signal(SIGINT, shutdown_module);

	setlocale(LC_ALL, "C");

	carmen_ipc_dispatch();

	return 0;
}