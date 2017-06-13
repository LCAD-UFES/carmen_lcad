/*
 * show_boxes.cpp
 *
 *  Created on: 13 de jun de 2017
 *      Author: luan
 */

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_camera_calibration.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int camera;
int camera_side;


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
image_handler(carmen_bumblebee_basic_stereoimage_message* image_msg)
{
	static cv::Mat *src_image = NULL;
	static cv::Mat *rgb_image = NULL;

	if (src_image == NULL)
	{
		src_image = new cv::Mat(cv::Size(image_msg->width, image_msg->height), CV_8UC3);
		rgb_image = new cv::Mat(cv::Size(image_msg->width, image_msg->height), CV_8UC3);
	}

	if (camera_side == 0)
	{
		memcpy(src_image->data, image_msg->raw_left, image_msg->image_size * sizeof(char));
	}
	else
	{
		memcpy(src_image->data, image_msg->raw_right, image_msg->image_size * sizeof(char));
	}

//	carmen_velodyne_camera_calibration_lasers_points_in_camera_with_obstacle_and_display(
//						velodyne_message, image_msg->width, image_msg->height);

	cv::cvtColor(*src_image, *rgb_image, cv::COLOR_RGB2BGR);

	cv::imshow("Display window", *rgb_image);
	cv::waitKey(1);
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		cvDestroyAllWindows();

		printf("show_boxes: disconnected.\n");
		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////


void
subscribe_messages()
{
    carmen_bumblebee_basic_subscribe_stereoimage(camera,
    		NULL, (carmen_handler_t) image_handler,
			CARMEN_SUBSCRIBE_LATEST);
}


int
main(int argc, char **argv)
{

	if (argc != 3)
	{
		fprintf(stderr, "%s: Wrong number of parameters. tracker_opentld requires 2 parameter and received %d. \n Usage: %s <camera_number> <camera_side(0-left; 1-right)\n>", argv[0], argc - 1, argv[0]);
		exit(1);
	}

	cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
	setlocale(LC_ALL, "C");

	camera = atoi(argv[1]);
	camera_side = atoi(argv[2]);

	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);

	subscribe_messages();
	carmen_ipc_dispatch();

	return 0;
}
