// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.
// https://github.com/IntelRealSense/librealsense/blob/master/examples/capture/rs-capture.cpp

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/stereo_velodyne.h>
#include <carmen/stereo_velodyne_interface.h>
#include "opencv2/opencv.hpp"

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

// basic file operations
#include <iostream>
#include <fstream>
// using namespace std;
#define BUMBLEBEE_ID 7

int stop_required = 0;
int rs_height = 720;
int rs_width = 1280;
int rs_flip = 0;

void
carmen_bumblebee_publish_stereoimage_message(unsigned char *rawLeft, unsigned char *rawRight, int width, int height, int channels, double timestamp)
{
	carmen_bumblebee_basic_stereoimage_message stereo_msg;

	stereo_msg.timestamp = timestamp;
    stereo_msg.host = carmen_get_host();
    stereo_msg.image_size = width * height * channels;
    stereo_msg.width = width;
    stereo_msg.isRectified = 1;
    stereo_msg.height = height;
    stereo_msg.raw_left = rawLeft;
    stereo_msg.raw_right = rawRight;

    carmen_bumblebee_basic_publish_message(BUMBLEBEE_ID, &stereo_msg);
}

void
carmen_laser_ldmrs_publish_depth_as_laser_message( rs2::points point_cloud, double width, double height, double timestamp)
{

    double vertical_pixels_angle = 57.0/width;
    double horizontal_pixels_angle = 86.0/height;

    carmen_laser_ldmrs_message laser_msg;

    laser_msg.start_angle = -28.5;
    laser_msg.end_angle = 28.5;
    laser_msg.scan_points = 480.0*640.0;
    laser_msg.angle_ticks_per_rotation = 640.0;
    laser_msg.scan_start_time = timestamp;
    laser_msg.host = carmen_get_host();
	laser_msg.timestamp = timestamp;


    carmen_laser_publish_ldmrs(&laser_msg);
}

void
save_extrinsics(rs2::pipeline_profile& selection)
{
	auto ir1_stream = selection.get_stream(RS2_STREAM_COLOR, 0);
	auto ir2_stream = selection.get_stream(RS2_STREAM_DEPTH, 0);
	rs2_extrinsics e = ir1_stream.get_extrinsics_to(ir2_stream);
	std::ofstream myfile;
	myfile.open ("extrinsicsColortoDepth.txt");
	myfile << "rotation:";
	myfile << std::endl;
	for(int x=0;x<9;x++) 
	{
		if (x%3==0 && x>0)
			myfile << std::endl;

		myfile << e.rotation[x] << " ";
	}
	myfile << std::endl;
	myfile << "translation:";
	myfile << std::endl;
	for(int z=0;z<3;z++) 
	{
		myfile << e.translation[z] << " ";
	}
	myfile.close();
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

	carmen_param_allow_unfound_variables(1);
	carmen_param_t param_list[] =
	{
		// {(char*) "commandline", (char*) "height", CARMEN_PARAM_INT, &rs_height, 0, NULL},
		// {(char*) "commandline", (char*) "width", CARMEN_PARAM_INT, &rs_width, 0, NULL},
		{bb_name, (char*) "height", CARMEN_PARAM_INT, &rs_height, 0, NULL},
		{bb_name, (char*) "width", CARMEN_PARAM_INT, &rs_width, 0, NULL},
		{(char*) "commandline", (char*) "flip", CARMEN_PARAM_ONOFF, &rs_flip, 0, NULL},
		//{bb_name, (char*) "flip", CARMEN_PARAM_ONOFF, &rs_flip, 0, NULL},
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	carmen_param_allow_unfound_variables(0);
	return 0;
}


unsigned char *
rotate_raw_image(int image_width, int image_height, unsigned char *raw_image)
{
	unsigned char *flipped_image = (unsigned char *) malloc (image_width * image_height * 3 * sizeof(unsigned char));  // Only works for 3 channels image

	image_height   = image_height * image_width * 3;
	image_width   *= 3;

	for (int line = 0, index = 0; line < image_height; line += image_width)
	{
		for (int column = 0; column < image_width; column += 3)
		{
				flipped_image[index]     = raw_image[(image_height - line - image_width) + (image_width - 3 - column)];
				flipped_image[index + 1] = raw_image[(image_height - line - image_width) + (image_width - 3 - column) + 1];
				flipped_image[index + 2] = raw_image[(image_height - line - image_width) + (image_width - 3 - column) + 2];

				index += 3;
		}
	}
	return (flipped_image);
}

carmen_velodyne_shot *
alloc_velodyne_shot_scan_vector(int horizontal_resolution_l, int vertical_resolution_l)
{
	int i;

	carmen_velodyne_shot *vector = (carmen_velodyne_shot *) malloc(horizontal_resolution_l * sizeof(carmen_velodyne_shot));
	carmen_test_alloc(vector);

	for (i = 0; i < horizontal_resolution_l; i++)
	{
		vector[i].distance = (unsigned short *) calloc(vertical_resolution_l, sizeof(unsigned short));
		carmen_test_alloc(vector[i].distance);
		vector[i].intensity = (unsigned char *) calloc(vertical_resolution_l, sizeof(unsigned char));
		carmen_test_alloc(vector[i].distance);
		vector[i].shot_size = vertical_resolution_l;
	}

	return (vector);
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
//    cfg.enable_stream(RS2_STREAM_COLOR, 0, rs_width, rs_height);
//    cfg.enable_stream(RS2_STREAM_DEPTH, 0,rs_width, rs_height, RS2_FORMAT_DISPARITY16);

    carmen_stereo_velodyne_define_messages(BUMBLEBEE_ID);

	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;


    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Start streaming with default recommended configuration
    // The default video configuration contains Depth and Color streams
    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
    rs2::pipeline_profile selection = pipe.start(cfg);

	rs2::colorizer c;
	save_extrinsics(selection);
	unsigned char* depth_color_frame_data;
	unsigned short *depth_frame_data;
	unsigned char* rgb_frame_data;

	float depth_frame_float[rs_width*rs_height];
	carmen_velodyne_shot *scan = alloc_velodyne_shot_scan_vector(rs_width, rs_height);
	carmen_velodyne_variable_scan_message velodyne_partial_scan;
	velodyne_partial_scan.host = carmen_get_host();
	velodyne_partial_scan.number_of_shots = rs_width;
	velodyne_partial_scan.partial_scan = scan;

	stereo_util instance = get_stereo_instance(BUMBLEBEE_ID, rs_width, rs_height);

	rs2::hole_filling_filter hole_filter;

//	Configuration
	rs2::depth_sensor depth_sensor = selection.get_device().first<rs2::depth_sensor>();
	depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f);
	depth_sensor.set_option(RS2_OPTION_LASER_POWER, depth_sensor.get_option_range(RS2_OPTION_LASER_POWER).max);
	//depth_sensor.set_option(RS2_OPTION_ACCURACY, depth_sensor.get_option_range(RS2_OPTION_ACCURACY).max);

//
	rs2_intrinsics int_param = selection.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();

	printf("fx = %f\tfy = %f\tppx = %f\tppy = %f\n", int_param.fx/int_param.width , int_param.fy/int_param.height, int_param.ppx/int_param.width, int_param.ppy/int_param.height);

	while (!stop_required)
	{
        rs2::frameset frames = pipe.wait_for_frames();

		//frames = align_to_depth.process(frames);
		// frames = align_to_color.process(frames);

        rs2::depth_frame depth_frame = frames.get_depth_frame();
        rs2::video_frame frame_color = frames.get_color_frame();

        depth_frame = hole_filter.process(depth_frame);

        if (frame_color)
            rgb_frame_data = (unsigned char*) frame_color.get_data();

        if (depth_frame)
        {
            depth_frame_data = (unsigned short *) depth_frame.get_data(); // Pointer to depth pixels
            depth_color_frame_data = (unsigned char*) c.colorize(depth_frame).get_data(); // Pointer to depth pixels
        }

        double timestamp = carmen_get_time();

//        if (rs_flip)
//        {
//        	rgb_frame_data = rotate_raw_image(rs_width,rs_height,rgb_frame_data);
//        	depth_frame_data = rotate_raw_image(rs_width,rs_height,depth_frame_data);
//
//        	carmen_bumblebee_publish_stereoimage_message( (unsigned char*) rgb_frame_data, (unsigned char*) depth_frame_data , rs_width, rs_height, 3, timestamp);
//
//        	free(rgb_frame_data);
//        	free(depth_frame_data);
//        }
//        else

        cv::Mat depthMat(rs_height, rs_width, CV_16UC1, depth_frame_data);

        cv::medianBlur(depthMat, depthMat, 5);

        convert_stereo_depth_to_velodyne_beams(instance, (unsigned short*)depthMat.data, rs_height, rs_width, scan, 200000 , 0, rs_height, 0, rs_width, rgb_frame_data);

        velodyne_partial_scan.timestamp = timestamp;

		carmen_bumblebee_publish_stereoimage_message(rgb_frame_data, depth_color_frame_data, rs_width, rs_height, 3, timestamp);
		carmen_stereo_velodyne_publish_message(BUMBLEBEE_ID, &velodyne_partial_scan);
	}
	return 0;
}

