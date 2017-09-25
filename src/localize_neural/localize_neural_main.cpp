/*
 * publish.cpp
 *
 *  Created on: Jun 13, 2017
 *      Author: avelino
 */

#include <carmen/carmen.h>
#include <carmen/global_graphics.h>
#include <carmen/base_ackerman_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <string>
#include <tf.h>
#include "localize_neural_torch.h"

using namespace std;

int camera = 0;

tf::Transformer transformer;

carmen_pose_3D_t car_pose_g;
carmen_pose_3D_t camera_pose_g;
carmen_pose_3D_t board_pose_g;

vector<pair<carmen_pose_3D_t, double> > camera_poses_base_array;
vector<pair<string, double> > camera_frames_base_array;
vector<pair<carmen_pose_3D_t, double> > camera_poses_curr_array;
vector<pair<string, double> > camera_frames_curr_array;
vector<carmen_point_t> camera_delta_error_array;

carmen_simulator_ackerman_truepos_message truepos_message;
carmen_localize_ackerman_globalpos_message globalpos_message;
carmen_fused_odometry_message fused_odometry_message;
carmen_localize_neural_imagepos_message camera_message_curr;
carmen_localize_neural_imagepos_message camera_message_base;
carmen_mapper_virtual_laser_message virtual_laser_message;


carmen_pose_3D_t
camera_carmen_transform(const carmen_pose_3D_t & pose)
{
	double roll, pitch, yaw;
	carmen_pose_3D_t finalpos;

	tf::Matrix3x3 camera_matrix(
			0, -1,  0, // camera x-axis wrt world
			0,  0, -1, // camera y-axis wrt world
			1,  0,  0  // camera z-axis wrt world
			);
	tf::Quaternion camera_pose_q;
	camera_matrix.getRotation(camera_pose_q);
	tf::Vector3 camera_pose_3d(0,0,0);

	tf::Transform camera_frame;
	camera_frame.setOrigin(camera_pose_3d);
	camera_frame.setRotation(camera_pose_q);

	tf::Vector3 pose_3d(pose.position.x, pose.position.y, pose.position.z);
	tf::Quaternion pose_q(pose.orientation.yaw, pose.orientation.pitch, pose.orientation.roll);

	tf::Transform pose_wrt_camera;
	pose_wrt_camera.setOrigin(pose_3d);
	pose_wrt_camera.setRotation(pose_q);

	tf::Transform pose_wrt_carmen = camera_frame.inverse() * pose_wrt_camera * camera_frame;

	tf::Vector3 final_pose_3d = pose_wrt_carmen.getOrigin();
	tf::Quaternion final_pose_q = pose_wrt_carmen.getRotation();
	tf::Matrix3x3(final_pose_q).getRPY(roll, pitch, yaw);

	finalpos.position.x = final_pose_3d.getX();
	finalpos.position.y = final_pose_3d.getY();
	finalpos.position.z = final_pose_3d.getZ();
	finalpos.orientation.roll = carmen_normalize_theta(roll);
	finalpos.orientation.pitch = carmen_normalize_theta(pitch);
	finalpos.orientation.yaw = carmen_normalize_theta(yaw);
	return finalpos;
}


carmen_pose_3D_t
direct_transform(const carmen_pose_3D_t &basepos, const carmen_pose_3D_t &deltapos)
{
	double roll, pitch, yaw;
	carmen_pose_3D_t finalpos;

	tf::Vector3 base_pose_3d(basepos.position.x, basepos.position.y, basepos.position.z);
	tf::Quaternion base_pose_q(basepos.orientation.yaw, basepos.orientation.pitch, basepos.orientation.roll);
	tf::Vector3 delta_pose_3d(deltapos.position.x, deltapos.position.y, deltapos.position.z);
	tf::Quaternion delta_pose_q(deltapos.orientation.yaw, deltapos.orientation.pitch, deltapos.orientation.roll);

	tf::Transform base_pose;
	base_pose.setOrigin(base_pose_3d);
	base_pose.setRotation(base_pose_q);

	tf::Transform delta_pose;
	delta_pose.setOrigin(delta_pose_3d);
	delta_pose.setRotation(delta_pose_q);

	tf::Transform curr_pose = base_pose * delta_pose;
	tf::Quaternion curr_pose_q = curr_pose.getRotation();
	tf::Vector3 curr_pose_3d = curr_pose.getOrigin();
	tf::Matrix3x3(curr_pose_q).getRPY(roll, pitch, yaw);

	finalpos.position.x = curr_pose_3d.getX();
	finalpos.position.y = curr_pose_3d.getY();
	finalpos.position.z = curr_pose_3d.getZ();
	finalpos.orientation.roll = carmen_normalize_theta(roll);
	finalpos.orientation.pitch = carmen_normalize_theta(pitch);
	finalpos.orientation.yaw = carmen_normalize_theta(yaw);
	return finalpos;
}


carmen_pose_3D_t
inverse_transform(const carmen_pose_3D_t &basepos, const carmen_pose_3D_t &currpos)
{
	double roll, pitch, yaw;
	carmen_pose_3D_t deltapos;

	tf::Vector3 base_pose_3d(basepos.position.x, basepos.position.y, basepos.position.z);
	tf::Quaternion base_pose_q(basepos.orientation.yaw, basepos.orientation.pitch, basepos.orientation.roll);
	tf::Vector3 curr_pose_3d(currpos.position.x, currpos.position.y, currpos.position.z);
	tf::Quaternion curr_pose_q(currpos.orientation.yaw, currpos.orientation.pitch, currpos.orientation.roll);

	tf::Transform base_pose;
	base_pose.setOrigin(base_pose_3d);
	base_pose.setRotation(base_pose_q);

	tf::Transform curr_pose;
	curr_pose.setOrigin(curr_pose_3d);
	curr_pose.setRotation(curr_pose_q);

	tf::Transform delta_pose = base_pose.inverseTimes(curr_pose);
	tf::Quaternion delta_pose_q_wrt_base = delta_pose.getRotation();
	tf::Vector3 delta_pose_3d_wrt_base = delta_pose.getOrigin();
	tf::Matrix3x3(delta_pose_q_wrt_base).getRPY(roll, pitch, yaw);

	deltapos.position.x = delta_pose_3d_wrt_base.getX();
	deltapos.position.y = delta_pose_3d_wrt_base.getY();
	deltapos.position.z = delta_pose_3d_wrt_base.getZ();
	deltapos.orientation.roll = carmen_normalize_theta(roll);
	deltapos.orientation.pitch = carmen_normalize_theta(pitch);
	deltapos.orientation.yaw = carmen_normalize_theta(yaw);
	return deltapos;
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
load_poses(char *filename, vector<pair<carmen_pose_3D_t, double> > &poses, vector<pair<string, double> > &frames)
{
	char imagename_l[256];
	char imagename_r[256];
	double timestamp, dummy;
	carmen_pose_3D_t pose = {{0.0,0.0,0.0},{0.0,0.0,0.0}};
	FILE *log_file = fopen(filename, "r");

	fscanf(log_file, "%*[^\n]\n"); //skip header
	while(!feof(log_file))
	{
		//image x y z w p q r roll pitch yaw timestamp
		fscanf(log_file, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %s %s\n",
				&pose.position.x,
				&pose.position.y,
				&pose.position.z,
				&dummy, &dummy, &dummy, &dummy,
				&pose.orientation.roll,
				&pose.orientation.pitch,
				&pose.orientation.yaw,
				&timestamp,
				imagename_l,
				imagename_r);

		poses.push_back(pair<carmen_pose_3D_t, double>(pose, timestamp));
		frames.push_back(pair<string, double>(string(imagename_l), timestamp));
	}

	fclose(log_file);
	printf("Load complete.\n");
}


void
compute_statistics()
{
	double sum[3], mean[3], variance[3];
	uint count = camera_delta_error_array.size();

	sum[0] = sum[1] = sum[2] = 0.0;
	for(uint i = 0; i < count; i++)
	{
		sum[0] += camera_delta_error_array[i].x;
		sum[1] += camera_delta_error_array[i].y;
		sum[2] += camera_delta_error_array[i].theta;
	}

	if (count > 0)
	{
		mean[0] = sum[0] / (double) count;
		mean[1] = sum[1] / (double) count;
		mean[2] = sum[2] / (double) count;
	}

	variance[0] = variance[1] = variance[2] = 0.0;
	for(uint i = 0; i < count; i++)
	{
		variance[0] += pow(camera_delta_error_array[i].x		-mean[0],2.0);
		variance[1] += pow(camera_delta_error_array[i].y		-mean[1],2.0);
		variance[2] += pow(camera_delta_error_array[i].theta	-mean[2],2.0);
	}

	if (count > 0)
	{
		variance[0] /= (double) count;
		variance[1] /= (double) count;
		variance[2] /= (double) count;
	}

	FILE *f = fopen("stats.txt", "w");
	fprintf(f, "mean_x mean_y mean_theta sigma_x sigma_y sigma_theta\n");
	fprintf(f, "%.3f %.3f %.3f %.6f %.6f %.6f\n",
			mean[0], mean[1], mean[2],
			sqrt(variance[0]), sqrt(variance[1]), sqrt(variance[2]));
	fclose(f);
}


int
find_more_synchronized_pose(const vector<pair<carmen_pose_3D_t, double> > &poses, double timestamp)
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
	cv::Mat img = cv::imread (imagepath.c_str(), CV_LOAD_IMAGE_COLOR);

	message->height = img.rows;
	message->width = img.cols;
	message->size = 3 * message->height * message->width;

	if (message->image_data == NULL)
		message->image_data = (char *) calloc (message->size, sizeof(char));

	for(int i = 0; i < (message->height * message->width); i++)
	{
		// opencv image is BGR and I want my image in rgb format ...
		message->image_data [3 * i + 2] = (uchar)img.data[3 * i + 0];
		message->image_data [3 * i + 1] = (uchar)img.data[3 * i + 1];
		message->image_data [3 * i + 0] = (uchar)img.data[3 * i + 2];
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_truepos_message(carmen_pose_3D_t pose, double timestamp)
{
	carmen_fused_odometry_message odom = fused_odometry_message;
	truepos_message.truepose.x = pose.position.x;
	truepos_message.truepose.y = pose.position.y;
	truepos_message.truepose.theta = pose.orientation.yaw;
	truepos_message.odometrypose.x = odom.pose.position.x;
	truepos_message.odometrypose.y = odom.pose.position.y;
	truepos_message.odometrypose.theta = odom.pose.orientation.yaw;
	truepos_message.v = odom.velocity.x;
	truepos_message.phi = odom.phi;

	truepos_message.host = carmen_get_host();
	truepos_message.timestamp = timestamp;

	IPC_RETURN_TYPE err = IPC_publishData(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME, &truepos_message);
	carmen_test_ipc(err, "Could not publish simualator_truepos_message",
			CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME);
}


static void
publish_globalpos_message(carmen_pose_3D_t pose, double timestamp)
{
	carmen_fused_odometry_message odom = fused_odometry_message;
	globalpos_message.phi = odom.phi;
	globalpos_message.v = odom.velocity.x;
	globalpos_message.converged = 1;
	globalpos_message.pose = pose;
	globalpos_message.velocity = odom.velocity;

	globalpos_message.globalpos.x = pose.position.x;
	globalpos_message.globalpos.y = pose.position.y;
	globalpos_message.globalpos.theta = pose.orientation.yaw;

	globalpos_message.host = carmen_get_host();
	globalpos_message.timestamp = timestamp;

	carmen_localize_ackerman_publish_globalpos_message(&globalpos_message);
}


void
publish_imagepos_keyframe_message(carmen_pose_3D_t pose, double timestamp)
{
	camera_message_base.pose = pose;

	camera_message_base.host = carmen_get_host();
	camera_message_base.timestamp = timestamp;

	carmen_localize_neural_publish_imagepos_keyframe_message(&camera_message_base);
}


void
publish_imagepos_curframe_message(carmen_pose_3D_t pose, double timestamp)
{
	camera_message_curr.pose = pose;

	camera_message_curr.host = carmen_get_host();
	camera_message_curr.timestamp = timestamp;

	carmen_localize_neural_publish_imagepos_curframe_message(&camera_message_curr);
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
	double camera_timestamp;
	carmen_pose_3D_t true_pose;
	carmen_pose_3D_t global_pose;
	carmen_pose_3D_t camera_pose_base;
	carmen_pose_3D_t camera_pose_curr;
	carmen_pose_3D_t camera_pose_true;

	int index_base = find_more_synchronized_pose(camera_poses_base_array, message->timestamp);
	if (index_base < 0)
		return;

	int index_curr = find_more_synchronized_pose(camera_poses_curr_array, message->timestamp);
	if (index_curr < 0)
		return;

	if (camera_frames_curr_array[index_curr].second > message->timestamp)
		camera_timestamp = camera_frames_curr_array[index_curr].second;
	else
		camera_timestamp = message->timestamp;

	string key_frame = camera_frames_base_array[index_base].first;
	string cur_frame = camera_frames_curr_array[index_curr].first;

	copy_image(&camera_message_base, key_frame);
	copy_image(&camera_message_curr, cur_frame);

	camera_pose_base = camera_poses_base_array[index_base].first;
	camera_pose_true = camera_poses_curr_array[index_curr].first;

	carmen_pose_3D_t delta_pose_true = inverse_transform(camera_pose_base, camera_pose_true);
	if (delta_pose_true.position.x < 0.0)
		return;

	carmen_pose_3D_t delta_pose_curr = forward_network(camera_message_curr, camera_message_base);
	delta_pose_curr = camera_carmen_transform(delta_pose_curr);

	camera_pose_curr = direct_transform(camera_pose_base, delta_pose_curr);
	camera_pose_base.position.z = 1.7;
	camera_pose_curr.position.z = 1.7;

	carmen_point_t delta_pose_error = {
			(delta_pose_curr.position.x-delta_pose_true.position.x),
			(delta_pose_curr.position.y-delta_pose_true.position.y),
			(delta_pose_curr.orientation.yaw-delta_pose_true.orientation.yaw)
	};
	camera_delta_error_array.push_back(delta_pose_error);
	carmen_warn("(%.2f %.2f %.3f)\n", delta_pose_error.x, delta_pose_error.y, delta_pose_error.theta);

	tf::StampedTransform car_transform_curr = get_transforms_from_camera_to_car(camera_pose_curr.position.x, camera_pose_curr.position.y, camera_pose_curr.orientation.yaw);
	tf::StampedTransform car_transform_true = get_transforms_from_camera_to_car(camera_pose_true.position.x, camera_pose_true.position.y, camera_pose_true.orientation.yaw);

	tf::Vector3 position = car_transform_curr.getOrigin();
	tf::Quaternion orientation = car_transform_curr.getRotation();

	tf::Vector3 position_true = car_transform_true.getOrigin();
	tf::Quaternion orientation_true = car_transform_true.getRotation();

	global_pose.position.x = position.getX();
	global_pose.position.y = position.getY();
	global_pose.position.z = position.getZ();

	true_pose.position.x = position_true.getX();
	true_pose.position.y = position_true.getY();
	true_pose.position.z = position_true.getZ();

	tf::Matrix3x3(orientation).getRPY(global_pose.orientation.roll, global_pose.orientation.pitch, global_pose.orientation.yaw);
	tf::Matrix3x3(orientation_true).getRPY(true_pose.orientation.roll, true_pose.orientation.pitch, true_pose.orientation.yaw);

	virtual_laser_message.positions[0].x = global_pose.position.x;
	virtual_laser_message.positions[0].y = global_pose.position.y;
	virtual_laser_message.positions[1].x = true_pose.position.x;
	virtual_laser_message.positions[1].y = true_pose.position.y;

	//publish_truepos_message(true_pose, camera_timestamp);
	publish_globalpos_message(global_pose, camera_timestamp);
	publish_imagepos_curframe_message(camera_pose_curr, camera_timestamp);
	publish_imagepos_keyframe_message(camera_pose_base, camera_timestamp);
	carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, camera_timestamp);
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		finalize_network();
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
initialize_messages()
{
	memset(&truepos_message, 0, sizeof(truepos_message));
	memset(&globalpos_message, 0, sizeof(globalpos_message));
	memset(&camera_message_curr, 0, sizeof(camera_message_curr));
	memset(&camera_message_base, 0, sizeof(camera_message_base));
	memset(&virtual_laser_message, 0, sizeof(carmen_mapper_virtual_laser_message));
	virtual_laser_message.num_positions = 2;
	virtual_laser_message.positions = (carmen_position_t *) calloc(virtual_laser_message.num_positions, sizeof(carmen_position_t));
	virtual_laser_message.colors = (char *) calloc(virtual_laser_message.num_positions, sizeof(char));
	virtual_laser_message.host = carmen_get_host();
	virtual_laser_message.colors[0] = CARMEN_RED;
	virtual_laser_message.colors[1] = CARMEN_BLUE;
}


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

	if (argc < 2)
	{
		carmen_die("\nUsage: %s<camera_id> \n", argv[0]);
	}

	camera = atoi(argv[1]);

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

	err = IPC_defineMsg(CARMEN_MAPPER_VIRTUAL_LASER_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_MAPPER_VIRTUAL_LASER_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_MAPPER_VIRTUAL_LASER_MESSAGE_NAME);

	err = IPC_defineMsg(CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME, IPC_VARIABLE_LENGTH, CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_SIMULATOR_ACKERMAN_TRUEPOS_NAME);

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
}
///////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_camera_parameters(argc, argv);

	define_messages();

	initialize_messages();

	initialize_transformations();

	initialize_network(argv[4]);

	load_poses(argv[2], camera_poses_base_array, camera_frames_base_array);
	load_poses(argv[3], camera_poses_curr_array, camera_frames_curr_array);

	signal(SIGINT, shutdown_module);

	subscribe_to_relevant_messages();

	carmen_ipc_addPeriodicTimer(30.0, (TIMER_HANDLER_TYPE) compute_statistics, NULL);

	carmen_ipc_dispatch();

	return(0);
}
