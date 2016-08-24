//
// Created by thom on 10/08/16.
//

#include <cstdio>
#include <iostream>
#include <string>

#include <carmen/carmen.h>
#include <opencv2/opencv.hpp>
#include "zed_camera_sensor_interface.h"

using namespace cv;

void processStereoImage(carmen_zed_camera_sensor_stereoimage_message *msg) {

    Mat frameLeft(msg->height, msg->width, CV_8UC4, msg->raw_left, 4 * msg->width);
    Mat frameRight(msg->height, msg->width, CV_8UC4, msg->raw_right, 4 * msg->width);

    imshow("LEFT", frameLeft);
    imshow("RIGHT", frameRight);
    if(waitKey(1) == 27)
        exit(0);
}

void processDepthMap(carmen_zed_camera_sensor_depthmap_message *msg) {

    Mat frame(msg->height, msg->width, CV_8UC4, msg->raw_image, 4 * msg->width);

    imshow("DEPTH", frame);
    if(waitKey(1) == 27)
        exit(0);
}

void
shutdown_module(int signo)
{
    if (signo == SIGINT)
    {
        carmen_ipc_disconnect();
        printf("as1 camera view was disconnected.\n");
        exit(0);
    }
}


int main(int argc, char* argv[]) {

    carmen_ipc_initialize(argc, argv);
    signal(SIGINT, shutdown_module);

    carmen_zed_camera_sensor_subscribe_stereoimage(NULL, (carmen_handler_t) processStereoImage, CARMEN_SUBSCRIBE_LATEST);
    carmen_zed_camera_sensor_subscribe_depthmap(NULL, (carmen_handler_t) processDepthMap, CARMEN_SUBSCRIBE_LATEST);
    carmen_ipc_dispatch();
}