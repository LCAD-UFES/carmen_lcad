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

const unsigned int maxPositions = 50;
carmen_velodyne_partial_scan_message *velodyne_message_arrange;
std::vector<carmen_velodyne_partial_scan_message> velodyne_vector;


/*
 This function find the closest velodyne message with the camera message
 */

carmen_velodyne_partial_scan_message
find_velodyne_most_sync_with_cam(double bumblebee_timestamp)
{
	carmen_velodyne_partial_scan_message velodyne;
	double minTimestampDiff = DBL_MAX;
	int minTimestampIndex = -1;
	for (unsigned int i = 0; i < velodyne_vector.size(); i++)
	{
		if(fabs(velodyne_vector[i].timestamp - bumblebee_timestamp) < minTimestampDiff)
		{
			minTimestampIndex = i;
			minTimestampDiff = fabs(velodyne_vector[i].timestamp - bumblebee_timestamp);
		}
	}
	velodyne = velodyne_vector[minTimestampIndex];
	return (velodyne);
}


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

	carmen_velodyne_partial_scan_message velodyne_sync_with_cam;
	std::vector<carmen_velodyne_points_in_cam_with_obstacle_t> points_lasers_in_cam_with_obstacle;

	velodyne_sync_with_cam = find_velodyne_most_sync_with_cam(image_msg->timestamp);
	points_lasers_in_cam_with_obstacle =  carmen_velodyne_camera_calibration_lasers_points_in_camera_with_obstacle_and_display(
			&velodyne_sync_with_cam, image_msg->width, image_msg->height);

	cv::cvtColor(*src_image, *rgb_image, cv::COLOR_RGB2BGR);

	for (unsigned int i = 0; i < points_lasers_in_cam_with_obstacle.size(); i++)
	{
		if (points_lasers_in_cam_with_obstacle[i].hit_in_obstacle == true)
		{
			cv::circle(*rgb_image, cv::Point(points_lasers_in_cam_with_obstacle[i].velodyne_points_in_cam.ipx,
					points_lasers_in_cam_with_obstacle[i].velodyne_points_in_cam.ipy), 2, cv::Scalar(0, 255, 0), -1);
		}
	}

	cv::Mat resized_image(cv::Size(640, 480), CV_8UC3);
	cv::resize(*rgb_image, resized_image, resized_image.size());

	cv::imshow("Display window", resized_image);
	cv::waitKey(1);
}


void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	velodyne_message_arrange = velodyne_message;
	carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(velodyne_message_arrange);

	carmen_velodyne_partial_scan_message velodyne_copy;

	velodyne_copy.host = velodyne_message_arrange->host;
	velodyne_copy.number_of_32_laser_shots = velodyne_message_arrange->number_of_32_laser_shots;
	velodyne_copy.partial_scan = (carmen_velodyne_32_laser_shot*)malloc(sizeof(carmen_velodyne_32_laser_shot) * velodyne_message_arrange->number_of_32_laser_shots);
	memcpy(velodyne_copy.partial_scan, velodyne_message_arrange->partial_scan, sizeof(carmen_velodyne_32_laser_shot) * velodyne_message_arrange->number_of_32_laser_shots);
	velodyne_copy.timestamp = velodyne_message_arrange->timestamp;

	velodyne_vector.push_back(velodyne_copy);

	if (velodyne_vector.size() > maxPositions)
	{
		free(velodyne_vector.begin()->partial_scan);
		velodyne_vector.erase(velodyne_vector.begin());
	}
	//	show_velodyne(velodyne_message);
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

	carmen_velodyne_subscribe_partial_scan_message(NULL,
			(carmen_handler_t)velodyne_partial_scan_message_handler,
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
