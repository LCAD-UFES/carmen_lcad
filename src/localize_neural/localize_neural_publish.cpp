/*
 * publish.cpp
 *
 *  Created on: Jun 13, 2017
 *      Author: avelino
 */

#include <carmen/carmen.h>
#include <carmen/base_ackerman_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <string>
using namespace std;

vector<pair<carmen_pose_3D_t, double> > poses_array;
vector<pair<string, double> > imagenames_array;

void
load_GT_poses(char *filename)
{
	char imagename[256];
	double timestamp, dummy;
	carmen_pose_3D_t pose;
	FILE *log_file = fopen(filename, "r");

	while(!feof(log_file))
	{
		fscanf(log_file, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
			imagename, &pose.position.x, &pose.position.y, &pose.position.z,
			&dummy, //w
			&dummy, //p
			&dummy, //q
			&dummy, //r
			&pose.orientation.roll, &pose.orientation.pitch, &pose.orientation.yaw,
			&timestamp);

		poses_array.push_back(pair<carmen_pose_3D_t, double>(pose, timestamp));
		imagenames_array.push_back(pair<string, double>(string(imagename), timestamp));
	}

	fclose(log_file);
	printf("Load complete.\n");
}

int
find_more_synchronized_pose(double timestamp)
{
	uint i;
	int index_min_time_diff;
	double min_time_diff, time_diff;

	min_time_diff = DBL_MAX;
	index_min_time_diff = -1;

	for (i = 0; i < poses_array.size(); i++)
	{
		time_diff = timestamp - poses_array[i].second;

		if ((time_diff < min_time_diff) && (time_diff > 0))
		{
			min_time_diff = time_diff;
			index_min_time_diff = i;
		}
	}

	return index_min_time_diff;
}

void
assembly_and_publish_globalpos_message(carmen_pose_3D_t pose, double timestamp, carmen_fused_odometry_message *msg)
{
	carmen_localize_ackerman_globalpos_message message;
	memset(&message, 0, sizeof(message));

	message.phi = msg->phi;
	message.v = msg->velocity.x;
	message.converged = 1;
	message.pose = pose;
	message.velocity = msg->velocity;

	message.globalpos.x = pose.position.x;
	message.globalpos.y = pose.position.y;
	message.globalpos.theta = pose.orientation.yaw;

	message.host = carmen_get_host();
	message.timestamp = timestamp;

	carmen_localize_ackerman_publish_globalpos_message(&message);
}

void
copy_image (carmen_localize_neural_imagepos_message *message, string imagename)
{
	string imagepath = string("/dados/ufes/") + imagename;
	IplImage *img = cvLoadImage (imagepath.c_str(), CV_LOAD_IMAGE_ANYCOLOR);

	message->height = img->height;
	message->width = img->width;
	message->size = 3 * img->height * img->width;

	if (message->image_data == NULL)
		message->image_data = (char *) calloc (message->size, sizeof(char));

	for(int i = 0; i < img->height; i++)
	{
		for(int j = 0; j < img->width; j++)
		{
			int p_msg = 3 * (i * img->width + j);
			int p_img = (i * img->widthStep) + (3 * j);

			// opencv image is BGR and I want my image in rgb format ...
			message->image_data [p_msg + 2] = img->imageData[p_img + 0];
			message->image_data [p_msg + 1] = img->imageData[p_img + 1];
			message->image_data [p_msg + 0] = img->imageData[p_img + 2];
		}
	}
}

void
assembly_and_publish_imagepos_message(carmen_pose_3D_t pose, double timestamp, string imagename)
{
	static carmen_localize_neural_imagepos_message message;
	static bool first_time = true;
	if (first_time)
	{
		first_time = false;
		memset(&message, 0, sizeof(message));
	}

	copy_image(&message, imagename);

	message.pose = pose;

	message.host = carmen_get_host();
	message.timestamp = timestamp;

	carmen_localize_neural_publish_imagepos_message(&message);
}

void
fused_odometry_handler(carmen_fused_odometry_message *message)
{
	int index = find_more_synchronized_pose(message->timestamp);

	if (index < 0)
		return;

	carmen_pose_3D_t pose = poses_array[index].first;

	string imagename = imagenames_array[index].first;

	assembly_and_publish_globalpos_message(pose, message->timestamp, message);
	assembly_and_publish_imagepos_message(pose, message->timestamp, imagename);
}

static void
define_messages(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_NEURAL_IMAGEPOS_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_NEURAL_IMAGEPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_NEURAL_IMAGEPOS_NAME);
}

void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("publish: disconnected.\n");

		exit(0);
	}
}

int
main(int argc, char **argv)
{
	if (argc < 2)
		exit(printf("Use %s <deltapos-20140418.txt>\n", argv[0]));

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	define_messages();

	signal(SIGINT, shutdown_module);

	load_GT_poses(argv[1]);

	carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t) fused_odometry_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();
	return(0);
}
