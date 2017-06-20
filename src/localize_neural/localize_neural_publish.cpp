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

vector<pair<carmen_pose_3D_t, double> > key_poses_array;
vector<pair<string, double> > key_frames_array;
vector<pair<carmen_pose_3D_t, double> > current_poses_array;
vector<pair<string, double> > current_frames_array;

void
load_image_poses(char *filename)
{
	string header;
	char frame[256];
	double timestamp, dummy;
	carmen_pose_3D_t pose;
	FILE *log_file = fopen(filename, "r");

	fscanf(log_file, "%*[^\n]\n");
	while(!feof(log_file))
	{
		//image x y z w p q r roll pitch yaw timestamp
		fscanf(log_file, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
			frame, &pose.position.x, &pose.position.y, &pose.position.z,
			&dummy, //w
			&dummy, //p
			&dummy, //q
			&dummy, //r
			&pose.orientation.roll, &pose.orientation.pitch, &pose.orientation.yaw,
			&timestamp);

		current_poses_array.push_back(pair<carmen_pose_3D_t, double>(pose, timestamp));
		current_frames_array.push_back(pair<string, double>(string(frame), timestamp));
	}

	fclose(log_file);
	printf("Load complete.\n");
}

void
transform_deltapos(carmen_pose_3D_t &finalpos, const carmen_pose_3D_t &keypos, const carmen_point_t &deltapos)
{
	double sin_theta = sin(deltapos.theta);
	double cos_theta = cos(deltapos.theta);
	//FIXME: it is not correct
	finalpos.position.x = keypos.position.x + deltapos.x * cos_theta - deltapos.y * sin_theta + keypos.position.x;
	finalpos.position.y = keypos.position.y + deltapos.x * sin_theta + deltapos.y * cos_theta + keypos.position.y;
	finalpos.orientation.yaw = deltapos.theta + keypos.orientation.yaw;
}

void
transform_odometry(carmen_pose_3D_t &finalpos, const carmen_pose_3D_t &keypos, double delta_trans, double delta_rot1, double delta_rot2)
{
	finalpos.position.x = delta_trans * cos(delta_rot1 + keypos.orientation.yaw) + keypos.position.x;
	finalpos.position.y = delta_trans * sin(delta_rot2 + keypos.orientation.yaw) + keypos.position.y;
	finalpos.orientation.yaw = delta_rot1 + delta_rot2 + keypos.orientation.yaw;
}

void
load_delta_poses(char *filename)
{
	char key_frame[256];
	char current_frame[256];
	double timestamp;
	double delta_trans, delta_rot1, delta_rot2;
	//carmen_point_t delta_pose = {0.0,0.0,0.0};
	carmen_pose_3D_t key_pose = {{0.0,0.0,0.0},{0.0,0.0,0.0}};
	FILE *log_file = fopen(filename, "r");

	fscanf(log_file, "%*[^\n]\n");
	while(!feof(log_file))
	{
		//base_image base_x base_y base_z base_yaw delta_x delta_y delta_yaw curr_image curr_timestamp
		fscanf(log_file, "%s %lf %lf %lf %lf %lf %lf %lf %s %lf\n",
			key_frame, &key_pose.position.x, &key_pose.position.y,
			&key_pose.position.z, &key_pose.orientation.yaw,
			&delta_trans, &delta_rot1, &delta_rot2,
			//&delta_pose.x, &delta_pose.y, &delta_pose.theta,
			current_frame,
			&timestamp);

		carmen_pose_3D_t current_pose = key_pose; //copy z
		transform_odometry(current_pose, key_pose, delta_trans, delta_rot1, delta_rot2);
		//transform_deltapos(current_pose, key_pose, delta_pose);
		key_poses_array.push_back(pair<carmen_pose_3D_t, double>(key_pose, timestamp));
		key_frames_array.push_back(pair<string, double>(string(key_frame), timestamp));
		current_poses_array.push_back(pair<carmen_pose_3D_t, double>(current_pose, timestamp));
		current_frames_array.push_back(pair<string, double>(string(current_frame), timestamp));
	}

	fclose(log_file);
	printf("Load complete.\n");
}

int
find_more_synchronized_pose(const vector<pair<carmen_pose_3D_t, double> > &poses, double timestamp)
{
	uint i;
	int index_min_time_diff;
	double min_time_diff, time_diff;

	min_time_diff = DBL_MAX;
	index_min_time_diff = -1;

	for (i = 0; i < poses.size(); i++)
	{
		time_diff = timestamp - poses[i].second;

		if ((time_diff < min_time_diff) && (time_diff > 0))
		{
			min_time_diff = time_diff;
			index_min_time_diff = i;
		}
	}

	return index_min_time_diff;
}

vector<int>
find_all_synchronized_pose(const vector<pair<carmen_pose_3D_t, double> > &poses, double timestamp)
{
	uint i;
	vector<int> index_min_time_diff;
	double min_time_diff, time_diff;

	min_time_diff = DBL_MAX;

	for (i = 0; i < poses.size(); i++)
	{
		time_diff = timestamp - poses[i].second;

		if ((time_diff <= min_time_diff) && (time_diff > 0))
		{
			min_time_diff = time_diff;
			index_min_time_diff.push_back(i);
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
assembly_and_publish_imagepos_keyframe_message(carmen_pose_3D_t pose, double timestamp, string imagename)
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

	carmen_localize_neural_publish_imagepos_keyframe_message(&message);
}

void
assembly_and_publish_imagepos_curframe_message(carmen_pose_3D_t pose, double timestamp, string imagename)
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

	carmen_localize_neural_publish_imagepos_curframe_message(&message);
}

void
fused_odometry_handler(carmen_fused_odometry_message *message)
{
	int index = find_more_synchronized_pose(current_poses_array, message->timestamp);

	if (index < 0)
		return;

	carmen_pose_3D_t pose = current_poses_array[index].first;

	string frame = current_frames_array[index].first;

	assembly_and_publish_imagepos_curframe_message(pose, message->timestamp, frame);
}

void
fused_odometry_handler2(carmen_fused_odometry_message *message)
{
	int index = find_more_synchronized_pose(key_poses_array, message->timestamp);

	if (index < 0)
		return;

	carmen_pose_3D_t key_pose = key_poses_array[index].first;
	carmen_pose_3D_t current_pose = current_poses_array[index].first;
	string key_frame = key_frames_array[index].first;
	string current_frame = current_frames_array[index].first;

	assembly_and_publish_imagepos_curframe_message(current_pose, message->timestamp, current_frame);
	assembly_and_publish_imagepos_keyframe_message(key_pose, message->timestamp, key_frame);
}

void
fused_odometry_handler2_all(carmen_fused_odometry_message *message)
{
	vector<int> indices = find_all_synchronized_pose(key_poses_array, message->timestamp);

	for (unsigned int i=0; i < indices.size(); i++)
	{
		int index = indices[i];
		carmen_pose_3D_t key_pose = key_poses_array[index].first;
		carmen_pose_3D_t current_pose = current_poses_array[index].first;
		string key_frame = key_frames_array[index].first;
		string current_frame = current_frames_array[index].first;

		assembly_and_publish_imagepos_curframe_message(current_pose, message->timestamp, current_frame);
		assembly_and_publish_imagepos_keyframe_message(key_pose, message->timestamp, key_frame);
	}
}

static void
define_messages(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_ACKERMAN_GLOBALPOS_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_NEURAL_IMAGEPOS_KEYFRAME_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_NEURAL_IMAGEPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_NEURAL_IMAGEPOS_KEYFRAME_NAME);

	err = IPC_defineMsg(CARMEN_LOCALIZE_NEURAL_IMAGEPOS_CURFRAME_NAME, IPC_VARIABLE_LENGTH, CARMEN_LOCALIZE_NEURAL_IMAGEPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_LOCALIZE_NEURAL_IMAGEPOS_CURFRAME_NAME);
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
	bool is_delta_poses = true;

	if (argc < 2)
		exit(printf("Use %s <deltapos-20140418.txt>  0/1\n", argv[0]));

	if (argc > 2)
		is_delta_poses = (atoi(argv[2]) != 0);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	define_messages();

	signal(SIGINT, shutdown_module);

	if (is_delta_poses)
	{
		load_delta_poses(argv[1]);
		carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t) fused_odometry_handler2, CARMEN_SUBSCRIBE_LATEST);
	}
	else
	{
		load_image_poses(argv[1]);
		carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t) fused_odometry_handler, CARMEN_SUBSCRIBE_LATEST);
	}

	carmen_ipc_dispatch();
	return(0);
}
