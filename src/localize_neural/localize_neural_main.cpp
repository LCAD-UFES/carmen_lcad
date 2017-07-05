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
#include <tf.h>
#include "localize_neural_inference.h"

using namespace std;

int camera = 0;

tf::Transformer transformer;

double camera_timestamp = 0.0;
carmen_pose_3D_t global_pose;
carmen_pose_3D_t car_pose_g;
carmen_pose_3D_t camera_pose_g;
carmen_pose_3D_t board_pose_g;

vector<pair<carmen_pose_3D_t, double> > key_poses_array;
vector<pair<string, double> > key_frames_array;
vector<pair<carmen_point_t, double> > current_delta_poses_array;
vector<pair<string, double> > current_frames_array;

carmen_fused_odometry_message fused_odometry_message;
carmen_localize_neural_imagepos_message curframe_message;
carmen_localize_neural_imagepos_message keyframe_message;

//#define SYNC_WITH_VELODYNE = 1

void
transform_deltapos(carmen_pose_3D_t &finalpos, const carmen_pose_3D_t &keypos, const carmen_point_t &deltapos)
{
	double sin_theta = sin(keypos.orientation.yaw);
	double cos_theta = cos(keypos.orientation.yaw);
	finalpos.position.x = keypos.position.x + deltapos.x * cos_theta - deltapos.y * sin_theta;
	finalpos.position.y = keypos.position.y + deltapos.y * cos_theta + deltapos.x * sin_theta;
	finalpos.orientation.yaw = keypos.orientation.yaw + deltapos.theta;
}


void
transform_odometry(carmen_pose_3D_t &finalpos, const carmen_pose_3D_t &keypos, double delta_trans, double delta_rot1, double delta_rot2)
{
	finalpos.position.x = delta_trans * cos(delta_rot1 + keypos.orientation.yaw) + keypos.position.x;
	finalpos.position.y = delta_trans * sin(delta_rot2 + keypos.orientation.yaw) + keypos.position.y;
	finalpos.orientation.yaw = delta_rot1 + delta_rot2 + keypos.orientation.yaw;
}


tf::StampedTransform
get_transforms_from_camera_to_car(double camera_x, double camera_y, double camera_theta)
{
	tf::StampedTransform car_to_world_transform;
	tf::Transform camera_to_world_transform;

	camera_to_world_transform.setOrigin(tf::Vector3(camera_x, camera_y, 0.0));
	camera_to_world_transform.setRotation(tf::Quaternion(camera_theta, 0.0, 0.0));

	tf::StampedTransform camera_to_world_stamped_transform(camera_to_world_transform, tf::Time(0), "/world", "/camera");
	transformer.setTransform(camera_to_world_stamped_transform, "camera_to_world_stamped_transform");
	transformer.lookupTransform("/world", "/car", tf::Time(0), car_to_world_transform);

	return car_to_world_transform;
}


void
load_delta_poses(char *filename)
{
	char key_frame[256];
	char current_frame[256];
	double current_timestamp;
	carmen_point_t delta_pose = {0.0,0.0,0.0};
	carmen_pose_3D_t key_pose = {{0.0,0.0,0.0},{0.0,0.0,0.0}};
	FILE *log_file = fopen(filename, "r");

	fscanf(log_file, "%*[^\n]\n"); //skip header
	while(!feof(log_file))
	{
		//base_image base_x base_y base_z base_yaw delta_x delta_y delta_yaw curr_image curr_timestamp
		fscanf(log_file, "%s %lf %lf %lf %lf %lf %lf %lf %s %lf\n",
			key_frame, &key_pose.position.x, &key_pose.position.y,
			&key_pose.position.z, &key_pose.orientation.yaw,
			&delta_pose.x, &delta_pose.y, &delta_pose.theta,
			current_frame,
			&current_timestamp);

		key_poses_array.push_back(pair<carmen_pose_3D_t, double>(key_pose, current_timestamp));
		key_frames_array.push_back(pair<string, double>(string(key_frame), current_timestamp));
		current_delta_poses_array.push_back(pair<carmen_point_t, double>(delta_pose, current_timestamp));
		current_frames_array.push_back(pair<string, double>(string(current_frame), current_timestamp));
	}

	fclose(log_file);
	printf("Load complete.\n");
}


int
find_more_synchronized_pose(const vector<pair<carmen_point_t, double> > &poses, double timestamp)
{
	int nearest_index = -1;
	double shortest_interval = MAXDOUBLE;

	for (uint i = 0; i < poses.size(); i++)
	{
		double delta_t = timestamp - poses[i].second;
		if ((delta_t >= 0) && (delta_t <= shortest_interval))
		{
			shortest_interval = delta_t;
			nearest_index = i;
		}
	}

	return nearest_index;
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
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


static void
publish_globalpos_message(carmen_pose_3D_t pose, double timestamp)
{
	carmen_localize_ackerman_globalpos_message message;
	memset(&message, 0, sizeof(message));

	message.phi = fused_odometry_message.phi;
	message.v = fused_odometry_message.velocity.x;
	message.converged = 1;
	message.pose = pose;
	message.velocity = fused_odometry_message.velocity;

	message.globalpos.x = pose.position.x;
	message.globalpos.y = pose.position.y;
	message.globalpos.theta = pose.orientation.yaw;

	message.host = carmen_get_host();
	message.timestamp = timestamp;

	carmen_localize_ackerman_publish_globalpos_message(&message);
}


void
publish_imagepos_keyframe_message(carmen_pose_3D_t pose, double timestamp)
{
	static bool first_time = true;
	if (first_time)
	{
		first_time = false;
		memset(&keyframe_message, 0, sizeof(keyframe_message));
	}

	keyframe_message.pose = pose;

	keyframe_message.host = carmen_get_host();
	keyframe_message.timestamp = timestamp;

	carmen_localize_neural_publish_imagepos_keyframe_message(&keyframe_message);
}


void
publish_imagepos_curframe_message(carmen_pose_3D_t pose, double timestamp)
{
	static bool first_time = true;
	if (first_time)
	{
		first_time = false;
		memset(&curframe_message, 0, sizeof(curframe_message));
	}

	curframe_message.pose = pose;

	curframe_message.host = carmen_get_host();
	curframe_message.timestamp = timestamp;

	carmen_localize_neural_publish_imagepos_curframe_message(&curframe_message);
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_bumblebee_basic_stereoimage_message_handler(carmen_bumblebee_basic_stereoimage_message *message)
{
	camera_timestamp = message->timestamp;

	int index = find_more_synchronized_pose(current_delta_poses_array, message->timestamp);

	if (index < 0)
		return;

	string key_frame = key_frames_array[index].first;
	string current_frame = current_frames_array[index].first;

	copy_image(&keyframe_message, key_frame);
	copy_image(&curframe_message, current_frame);

	carmen_pose_3D_t key_pose = key_poses_array[index].first;
	carmen_pose_3D_t current_pose = key_pose; //copy z

	//carmen_point_t delta_pose = current_delta_poses_array[index].first;
	carmen_point_t delta_pose =	run_cnn_inference(keyframe_message, curframe_message);

	transform_deltapos(current_pose, key_pose, delta_pose);

	tf::StampedTransform car_to_world = get_transforms_from_camera_to_car(current_pose.position.x, current_pose.position.y, current_pose.orientation.yaw);

	tf::Vector3 position = car_to_world.getOrigin();
	tf::Quaternion orientation = car_to_world.getRotation();

	global_pose.position.x = position.getX();
	global_pose.position.y = position.getY();
	global_pose.position.z = position.getZ();

	tf::Matrix3x3(orientation).getRPY(global_pose.orientation.roll, global_pose.orientation.pitch, global_pose.orientation.yaw);

	#ifndef SYNC_WITH_VELODYNE
	publish_globalpos_message(global_pose, message->timestamp);
	#endif
	publish_imagepos_curframe_message(current_pose, message->timestamp);
	publish_imagepos_keyframe_message(key_pose, message->timestamp);
}


void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
	double dt = velodyne_message->timestamp - camera_timestamp;
	double v = fused_odometry_message.velocity.x;
	double phi = fused_odometry_message.phi;
	carmen_pose_3D_t velodyne_pose = global_pose;
	if ((dt > 0.0) && (camera_timestamp > 0.0))
	{
		velodyne_pose = carmen_ackerman_interpolated_robot_position_at_time(global_pose, dt, v, phi, 2.625);
	}
	else
	{
		carmen_die("dt<0\n");
	}
	publish_globalpos_message(velodyne_pose, velodyne_message->timestamp);
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		finalize_tensorflow();
		printf("publish: disconnected.\n");

		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
initialize_transformations()
{
	tf::Transform world_to_camera_pose;
	tf::Transform camera_to_board_pose;
	tf::Transform board_to_car_pose;

	tf::Time::init();

	// initial camera pose with respect to the world
	world_to_camera_pose.setOrigin(tf::Vector3(-camera_pose_g.position.x, camera_pose_g.position.y, camera_pose_g.position.z));
	world_to_camera_pose.setRotation(tf::Quaternion(camera_pose_g.orientation.yaw, camera_pose_g.orientation.pitch, camera_pose_g.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform board_to_camera_transform(world_to_camera_pose, tf::Time(0), "/world", "/camera");
	transformer.setTransform(board_to_camera_transform, "board_to_camera_transform");

	// board pose with respect to the camera
	camera_to_board_pose.setOrigin(tf::Vector3(-board_pose_g.position.x, board_pose_g.position.y, board_pose_g.position.z));
	camera_to_board_pose.setRotation(tf::Quaternion(board_pose_g.orientation.yaw, board_pose_g.orientation.pitch, board_pose_g.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform car_to_board_transform(camera_to_board_pose, tf::Time(0), "/camera", "/board");
	transformer.setTransform(car_to_board_transform, "car_to_board_transform");

	// car pose with respect to the board
	board_to_car_pose.setOrigin(tf::Vector3(-car_pose_g.position.x, car_pose_g.position.y, car_pose_g.position.z));
	board_to_car_pose.setRotation(tf::Quaternion(car_pose_g.orientation.yaw, car_pose_g.orientation.pitch, car_pose_g.orientation.roll));
	tf::StampedTransform world_to_car_transform(board_to_car_pose, tf::Time(0), "/board", "/car");
	transformer.setTransform(world_to_car_transform, "world_to_car_transform");

}


void
read_camera_parameters(int argc, char **argv)
{
	int num_items;
	char camera_string[256];

	if (argc < 3)
	{
		carmen_die("\nUsage: %s <log> <camera_id> \n", argv[0]);
	}

	camera = atoi(argv[2]);

	sprintf(camera_string, "%s%d", "camera", camera);

	carmen_param_t param_list[] =
	{
		{(char *) camera_string, (char *) "x",		CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.x), 0, NULL},
		{(char *) camera_string, (char *) "y",		CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.y), 0, NULL},
		{(char *) camera_string, (char *) "z", 		CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.z), 0, NULL},
		{(char *) camera_string, (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.yaw), 0, NULL},
		{(char *) camera_string, (char *) "pitch", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.pitch), 0, NULL},
		{(char *) camera_string, (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.roll), 0, NULL},

		{(char *) "car", 			 (char *) "x", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.x), 0, NULL},
		{(char *) "car", 			 (char *) "y", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.y), 0, NULL},
		{(char *) "car", 			 (char *) "z", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.z), 0, NULL},
		{(char *) "car", 			 (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.yaw), 0, NULL},
		{(char *) "car", 			 (char *) "pitch", CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.pitch), 0, NULL},
		{(char *) "car", 			 (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.roll), 0, NULL},

		{(char *) "sensor_board_1",  (char *) "x", 	CARMEN_PARAM_DOUBLE, &(board_pose_g.position.x), 0, NULL},
		{(char *) "sensor_board_1",  (char *) "y", 	CARMEN_PARAM_DOUBLE, &(board_pose_g.position.y), 0, NULL},
		{(char *) "sensor_board_1",  (char *) "z", 	CARMEN_PARAM_DOUBLE, &(board_pose_g.position.z), 0, NULL},
		{(char *) "sensor_board_1",  (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(board_pose_g.orientation.yaw), 0, NULL},
		{(char *) "sensor_board_1",  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(board_pose_g.orientation.pitch), 0, NULL},
		{(char *) "sensor_board_1",  (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(board_pose_g.orientation.roll), 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

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
subscribe_to_relevant_messages()
{
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) carmen_bumblebee_basic_stereoimage_message_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_fused_odometry_subscribe_fused_odometry_message(&fused_odometry_message, NULL, CARMEN_SUBSCRIBE_LATEST);
	#ifdef SYNC_WITH_VELODYNE
	carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);
	#endif
}
///////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_camera_parameters(argc, argv);

	define_messages();

	initialize_transformations();

	initialize_tensorflow(argv[3]);

	load_delta_poses(argv[1]);

	signal(SIGINT, shutdown_module);
	subscribe_to_relevant_messages();

	carmen_ipc_dispatch();
	return(0);
}
