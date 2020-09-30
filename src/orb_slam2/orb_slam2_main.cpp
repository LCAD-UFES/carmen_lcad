
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <string>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <carmen/bumblebee_basic_interface.h>
#include <System.h> // ORB_SLAM2

using namespace std;

ORB_SLAM2::System *SLAM = NULL;

char *output_file = NULL;
int shutdown_signal_received = 0;


void
save_camera_path_and_shutdown(int sign)
{
	if (sign == SIGINT && !shutdown_signal_received)
	{
		// prevent entering this part of the code several time.
		shutdown_signal_received = 1;
		
		SLAM->Shutdown();
		// save the output as a 3x4 transformation matrix
		SLAM->SaveTrajectoryKITTI(output_file);
		printf("Output file '%s' saved.", output_file);

		exit(0);
	}
}


void
carmen_bumblebee_basic_stereoimage_message_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	cv::Mat left(stereo_image->height, stereo_image->width, CV_8UC3, stereo_image->raw_left);
	cv::Mat right(stereo_image->height, stereo_image->width, CV_8UC3, stereo_image->raw_right);
	
	SLAM->TrackStereo(left, right, stereo_image->timestamp);	
}


int
main(int argc, char **argv)
{
	if (argc < 3)
		exit(printf("Error: Use %s <camera_id> <output_file>\n", argv[0]));
	
	int camera_id = atoi(argv[1]);
	output_file = argv[2];

	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, save_camera_path_and_shutdown);

	// TODO: write this file using parameters from carmen-ford-escape.ini
	const char *bb3_config = "bb3_config_640x480.yaml";
	char vocabulary_file[128];
	strcpy(vocabulary_file, getenv("HOME"));
	strcat(vocabulary_file, "/packages_carmen/ORB_SLAM2/Vocabulary/ORBvoc.txt");

	SLAM = new ORB_SLAM2::System(vocabulary_file, bb3_config, ORB_SLAM2::System::STEREO, true);

	carmen_bumblebee_basic_subscribe_stereoimage(
		camera_id, NULL, 
		(carmen_handler_t) carmen_bumblebee_basic_stereoimage_message_handler, 
		CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();

	return 0;
}
