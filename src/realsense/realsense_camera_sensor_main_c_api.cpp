// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.
// https://github.com/IntelRealSense/librealsense/blob/master/examples/C/color/rs-color.c

// ESSE ARQUIVO FUNCIONA, POREM UTILIZA A API EM C DA CAMERA, QUE EH MAIS COMPLICADA

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
// #include "opencv2/opencv.hpp"

#include <librealsense2/rs.h>
#include <librealsense2/h/rs_pipeline.h>
#include <librealsense2/h/rs_frame.h>

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#define BUMBLEBEE_ID 7

#define STREAM          RS2_STREAM_COLOR  // rs2_stream is a types of data provided by RealSense device           //
#define FORMAT          RS2_FORMAT_RGB8   // rs2_format is identifies how binary data is encoded within a frame   //
#define WIDTH           640               // Defines the number of columns for each frame                         //
#define HEIGHT          480               // Defines the number of lines for each frame                           //
#define FPS             30                // Defines the rate of frames per second                                //
#define STREAM_INDEX 0 // Defines the stream index, used for multiple streams of the same type //

int stop_required = 0;
int rs_height = 0;
int rs_width = 0;


/* Function calls to librealsense may raise errors of type rs_error*/
void check_error(rs2_error* e)
{
    if (e)
    {
        printf("rs_error was raised when calling %s(%s):\n", rs2_get_failed_function(e), rs2_get_failed_args(e));
        printf("    %s\n", rs2_get_error_message(e));
        exit(EXIT_FAILURE);
    }
}

void print_device_info(rs2_device* dev)
{
    rs2_error* e = 0;
    printf("\nUsing device 0, an %s\n", rs2_get_device_info(dev, RS2_CAMERA_INFO_NAME, &e));
    check_error(e);
    printf("    Serial number: %s\n", rs2_get_device_info(dev, RS2_CAMERA_INFO_SERIAL_NUMBER, &e));
    check_error(e);
    printf("    Firmware version: %s\n\n", rs2_get_device_info(dev, RS2_CAMERA_INFO_FIRMWARE_VERSION, &e));
    check_error(e);
}


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
	// https://github.com/IntelRealSense/librealsense/blob/master/examples/C/color/rs-color.c
	/////////////////////////////////////////////////////////////////////////////////////////
	rs2_error* e = 0;

    // Create a context object. This object owns the handles to all connected realsense devices.
    // The returned object should be released with rs2_delete_context(...)
    rs2_context* ctx = rs2_create_context(RS2_API_VERSION, &e);
    check_error(e);

    /* Get a list of all the connected devices. */
    // The returned object should be released with rs2_delete_device_list(...)
    rs2_device_list* device_list = rs2_query_devices(ctx, &e);
    check_error(e);

    int dev_count = rs2_get_device_count(device_list, &e);
    check_error(e);
    printf("There are %d connected RealSense devices.\n", dev_count);
    if (0 == dev_count)
        return EXIT_FAILURE;

    // Get the first connected device
    // The returned object should be released with rs2_delete_device(...)
    rs2_device* dev = rs2_create_device(device_list, 0, &e);
    check_error(e);

    print_device_info(dev);

    // Create a pipeline to configure, start and stop camera streaming
    // The returned object should be released with rs2_delete_pipeline(...)
    rs2_pipeline* pipeline =  rs2_create_pipeline(ctx, &e);
    check_error(e);

    // Create a config instance, used to specify hardware configuration
    // The retunred object should be released with rs2_delete_config(...)
    rs2_config* config = rs2_create_config(&e);
    check_error(e);

    // Request a specific configuration
    rs2_config_enable_stream(config, STREAM, STREAM_INDEX, rs_width, rs_height, FORMAT, FPS, &e);
    check_error(e);

    // Start the pipeline streaming
    // The retunred object should be released with rs2_delete_pipeline_profile(...)
	rs2_pipeline_profile* pipeline_profile = rs2_pipeline_start_with_config(pipeline, config, &e);

	while (!stop_required)
	{
        // This call waits until a new composite_frame is available
        // composite_frame holds a set of frames. It is used to prevent frame drops
        // The retunred object should be released with rs2_release_frame(...)
        rs2_frame* frames = rs2_pipeline_wait_for_frames(pipeline, 5000, &e);
        check_error(e);

        // Returns the number of frames embedded within the composite frame
        int num_of_frames = rs2_embedded_frames_count(frames, &e);
        check_error(e);

        int i;
        for (i = 0; i < num_of_frames; ++i)
        {
			printf("Num frames: %d\n", num_of_frames);
            // The retunred object should be released with rs2_release_frame(...)
            rs2_frame* frame = rs2_extract_frame(frames, i, &e);
            check_error(e);

            unsigned char* rgb_frame_data = (unsigned char*)(rs2_get_frame_data(frame, &e));
            check_error(e);
			
			carmen_bumblebee_publish_stereoimage_message(rgb_frame_data,rgb_frame_data,rs_width, rs_height, 3);



            // unsigned long long frame_number = rs2_get_frame_number(frame, &e);
            // check_error(e);

            // rs2_time_t frame_timestamp = rs2_get_frame_timestamp(frame, &e);
            // check_error(e);

            // // Specifies the clock in relation to which the frame timestamp was measured
            // rs2_timestamp_domain frame_timestamp_domain = rs2_get_frame_timestamp_domain(frame, &e);
            // check_error(e);
            // const char* frame_timestamp_domain_str = rs2_timestamp_domain_to_string(frame_timestamp_domain);

            // rs2_metadata_type frame_metadata_time_of_arrival = rs2_get_frame_metadata(frame, RS2_FRAME_METADATA_TIME_OF_ARRIVAL, &e);
            // check_error(e);

            // printf("RGB frame arrived.\n");
            // printf("First 10 bytes: ");
            // int i;
            // for(i=0; i < 10; ++i)
            //     printf("%02x ", rgb_frame_data[i]);

            // printf("\nFrame No: %llu\n", frame_number);
            // printf("Timestamp: %f\n", frame_timestamp);
            // printf("Timestamp domain: %s\n", frame_timestamp_domain_str);
            // printf("Time of arrival: %lld\n\n", frame_metadata_time_of_arrival);


            rs2_release_frame(frame);
        }

		rs2_release_frame(frames);
	}

	// Stop the pipeline streaming
	rs2_pipeline_stop(pipeline, &e);
	check_error(e);

	// Release resources
	rs2_delete_pipeline_profile(pipeline_profile);
	rs2_delete_config(config);
	rs2_delete_pipeline(pipeline);
	rs2_delete_device(dev);
	rs2_delete_device_list(device_list);
	rs2_delete_context(ctx);

	return 0;
}

