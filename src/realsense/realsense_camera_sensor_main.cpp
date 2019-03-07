// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.
// https://github.com/IntelRealSense/librealsense/blob/master/examples/capture/rs-capture.cpp

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
// #include "opencv2/opencv.hpp"

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API


#define BUMBLEBEE_ID 7

int stop_required = 0;
int rs_height = 0;
int rs_width = 0;

void
carmen_bumblebee_publish_stereoimage_message(unsigned char *rawLeft, unsigned char *rawRight, int width, int height, int channels)
{
	carmen_bumblebee_basic_stereoimage_message stereo_msg;

	stereo_msg.timestamp = carmen_get_time();
    stereo_msg.host = carmen_get_host();
    stereo_msg.image_size = width * height * channels;
    stereo_msg.width = width;
    stereo_msg.isRectified = 1;
    stereo_msg.height = height;
    stereo_msg.raw_left = rawLeft;
    stereo_msg.raw_right = rawRight;

    carmen_bumblebee_basic_publish_message(BUMBLEBEE_ID, &stereo_msg);
}


// bool 
// can_render(const rs2::frame& f) const
// {
//     auto format = f.get_profile().format();
//     switch (format)
//     {
//     case RS2_FORMAT_RGB8:
//     case RS2_FORMAT_RGBA8:
//     case RS2_FORMAT_Y8:
//     case RS2_FORMAT_MOTION_XYZ32F:
//         return true;
//     default:
//         return false;
//     }
// }


void shutdown_module(int signo)
{
	if(signo == SIGINT)
	{
		stop_required = 1;

		printf("rs_camera_sensor: disconnected.\n");
		exit(0);
	}
}


static int read_parameters(int argc, char **argv)
{
	int num_items;
	char bb_name[64];

	sprintf(bb_name, "bumblebee_basic%d", BUMBLEBEE_ID);

	carmen_param_t param_list[] =
	{
		// {(char*) "commandline", (char*) "height", CARMEN_PARAM_INT, &rs_height, 0, NULL},
		// {(char*) "commandline", (char*) "width", CARMEN_PARAM_INT, &rs_width, 0, NULL},
		{bb_name, (char*) "height", CARMEN_PARAM_INT, &rs_height, 0, NULL},
		{bb_name, (char*) "width", CARMEN_PARAM_INT, &rs_width, 0, NULL},
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}

int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_module);
	read_parameters(argc, argv);

	carmen_bumblebee_basic_define_messages(BUMBLEBEE_ID);
	/////////////////////////////////////////////////////////////////////////////////////////
	// https://github.com/IntelRealSense/librealsense/blob/master/examples/capture/rs-capture.cpp
	/////////////////////////////////////////////////////////////////////////////////////////

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 0, rs_width, rs_height);
    // cfg.enable_stream(RS2_STREAM_DEPTH, 0,640, 480);
    // cfg.enable_stream(RS2_STREAM_INFRARED, 0);
    // cfg.enable_stream(RS2_STREAM_INFRARED, 1);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Start streaming with default recommended configuration
    // The default video configuration contains Depth and Color streams
    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
    pipe.start(cfg);

	while (!stop_required)
	{

        rs2::frameset frames = pipe.wait_for_frames();

        // rs2::frame depth_frame = frames.first(RS2_STREAM_DEPTH);
        // if (depth_frame)
        //     depth_frame.get_data(); // Pointer to depth pixels

        rs2::frame frame_color = frames.first(RS2_STREAM_COLOR);
        if (frame_color)
        {
            unsigned char* rgb_frame_data = (unsigned char*) frame_color.get_data();
            carmen_bumblebee_publish_stereoimage_message(rgb_frame_data,rgb_frame_data, rs_width, rs_height, 3);
        }
        
	}

	

	return 0;
}

