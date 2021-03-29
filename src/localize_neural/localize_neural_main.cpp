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
//#include "localize_neural_wnn.h"
//#include "localize_neural_util.h"

using namespace std;

static int camera = 0;
static int cheating = 0;
static char* cnn_model;
static char* cnn_data;
static char* wnn_model;
static char* wnn_data;

tf::Transformer transformer;

carmen_pose_3D_t car_pose_g;
carmen_pose_3D_t camera_pose_g;
carmen_pose_3D_t board_pose_g;

vector<pair<carmen_pose_3D_t, double> > camera_delta_poses_array;
vector<pair<string, string> > camera_delta_frames_array;
vector<pair<carmen_pose_3D_t, double> > camera_poses_base_array;
vector<pair<string, double> > camera_frames_base_array;
vector<pair<carmen_pose_3D_t, double> > camera_poses_curr_array;
vector<pair<string, double> > camera_frames_curr_array;
vector<carmen_point_t> camera_delta_error_array;
vector<pair<carmen_pose_3D_t, int> > camera_poses_base_array2;
vector<pair<string, int> > camera_frames_base_array2;

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
load_base_poses(const char *filename)
{
	int imagelabel;
	char imagename[256];
	carmen_pose_3D_t base_pose = {{0.0,0.0,0.0},{0.0,0.0,0.0}};
	double timestamp;

	FILE *log_file = fopen(filename, "r");
	if (!log_file)
	{
		printf("File not found!\n %s\n", filename);
		exit(0);
	}

	fscanf(log_file, "%*[^\n]\n"); //skip header
	while(!feof(log_file))
	{
		//image label x y z rx ry rz timestamp
		fscanf(log_file, "%s %d %lf %lf %lf %lf %lf %lf %lf\n",
				imagename,
				&imagelabel,
				&base_pose.position.x, &base_pose.position.y, &base_pose.position.z,
				&base_pose.orientation.roll, &base_pose.orientation.pitch, &base_pose.orientation.yaw,
				&timestamp
		);
		if (file_exists(imagename))
		{
			camera_frames_base_array2.push_back(pair<string, int>(imagename, imagelabel));
			camera_poses_base_array2.push_back(pair<carmen_pose_3D_t, int>(base_pose, imagelabel));
		}
	}
	fclose(log_file);
}


void
load_delta_poses(const char *filename)
{
	char imagename_base_depth[256], imagename_base_left[256], imagename_base_right[256];
	char imagename_live_depth[256], imagename_live_left[256], imagename_live_right[256];
	double timestamp, dummy;
	carmen_pose_3D_t delta_pose = {{0.0,0.0,0.0},{0.0,0.0,0.0}};
	carmen_pose_3D_t base_pose = {{0.0,0.0,0.0},{0.0,0.0,0.0}};
	carmen_pose_3D_t live_pose = {{0.0,0.0,0.0},{0.0,0.0,0.0}};
	FILE *log_file = fopen(filename, "r");

	fscanf(log_file, "%*[^\n]\n"); //skip header
	while(!feof(log_file))
	{
		//dx dy dz dr dp dy base_depth base_image live_image timestamp
		fscanf(log_file, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %s %s %s %s %s %s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
				&delta_pose.position.x,
				&delta_pose.position.y,
				&delta_pose.position.z,
				&delta_pose.orientation.roll,
				&delta_pose.orientation.pitch,
				&delta_pose.orientation.yaw,
				&base_pose.position.x,
				&base_pose.position.y,
				&base_pose.position.z,
				&base_pose.orientation.roll,
				&base_pose.orientation.pitch,
				&base_pose.orientation.yaw,
				&live_pose.position.x,
				&live_pose.position.y,
				&live_pose.position.z,
				&live_pose.orientation.roll,
				&live_pose.orientation.pitch,
				&live_pose.orientation.yaw,
				imagename_base_depth, imagename_base_left, imagename_base_right,
				imagename_live_depth, imagename_live_left, imagename_live_right,
				&dummy, &dummy, &dummy, &dummy, &dummy,
				&dummy, &dummy, &dummy, &dummy, &dummy,
				&timestamp
		);
		if (file_exists(imagename_base_left))
		{
			camera_delta_frames_array.push_back(pair<string, string>(string(imagename_base_left), string(imagename_live_left)));
			camera_delta_poses_array.push_back(pair<carmen_pose_3D_t, double>(delta_pose, timestamp));
			camera_poses_base_array.push_back(pair<carmen_pose_3D_t, double>(base_pose, timestamp));
			camera_poses_curr_array.push_back(pair<carmen_pose_3D_t, double>(live_pose, timestamp));
		}
	}

	fclose(log_file);
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


float
euclidean_distance(float *pt1, float *pt2)
{
	return sqrt(pow(pt1[0]-pt2[0],2) + pow(pt1[1]-pt2[1],2));
}


int
find_more_similar_base_pose(const carmen_bumblebee_basic_stereoimage_message* message)
{
	int base_pose_index = -1;
	int base_pose_label = test_wnn_image(message);
	for (uint i = 0; i < camera_poses_base_array2.size(); i++)
	{
		if (camera_poses_base_array2[i].second == base_pose_label)
		{
			base_pose_index = i;
		}
	}

	return base_pose_index;
}


int
find_more_synchronized_pose(const vector<pair<carmen_pose_3D_t, double> > &poses, double timestamp)
{
	int nearest_index = -1;
	double shortest_interval = MAXDOUBLE;

	for (uint i = 0; i < poses.size(); i++)
	{
		//comment out code below to get just frames ahead
		double delta_t = timestamp - poses[i].second;
		if ((delta_t >= 0) && (delta_t <= shortest_interval))
			/*
		double delta_t = fabs(timestamp - poses[i].second);
		if (delta_t <= shortest_interval)
			 */
		{
			shortest_interval = delta_t;
			nearest_index = i;
		}
	}

	return nearest_index;
}


void
read_image_file (carmen_localize_neural_imagepos_message *message, string imagename)
{
	cv::Mat img = cv::imread (imagename.c_str(), CV_LOAD_IMAGE_COLOR);

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


void
read_image_message (carmen_localize_neural_imagepos_message *message, const carmen_bumblebee_basic_stereoimage_message *source)
{
	message->height = source->height;
	message->width = source->width;
	message->size = source->image_size;

	if (message->image_data == NULL)
		message->image_data = (char *) calloc (message->size, sizeof(char));

	memcpy(message->image_data, source->raw_left, message->size * sizeof(unsigned char));
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


void
publish_imagepos_messages(carmen_pose_3D_t camera_pose_true, carmen_pose_3D_t camera_pose_base, carmen_pose_3D_t camera_pose_curr, double camera_timestamp)
{
	carmen_pose_3D_t true_pose;
	carmen_pose_3D_t global_pose;

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
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_bumblebee_basic_stereoimage_message_handler_wnn(carmen_bumblebee_basic_stereoimage_message *message)
{
	//	carmen_pose_3D_t camera_pose_zero = {{0.0,0.0,0.0},{0.0,0.0,0.0}};
	carmen_pose_3D_t camera_pose_base, camera_pose_curr, camera_pose_true;
	carmen_pose_3D_t delta_pose_true, delta_pose_pred;

	double camera_timestamp = message->timestamp;

	int index_base = find_more_similar_base_pose(message);
	if (index_base < 0)
	{
		//carmen_warn("Not enough frames!");
		return;
	}

	int index_delta = find_more_synchronized_pose(camera_delta_poses_array, camera_timestamp);
	if (index_delta < 0)
	{
		//carmen_warn("Frame lost!\n");
		return;
	}

	camera_pose_true = camera_poses_curr_array[index_delta].first;

	read_image_message(&camera_message_curr, message);

	camera_pose_base = camera_poses_base_array2[index_base].first;

	string key_frame = camera_frames_base_array2[index_base].first;

	read_image_file(&camera_message_base, key_frame);

	delta_pose_true = inverse_transform(camera_pose_true, camera_pose_base);

	if (!cheating)
	{
		delta_pose_pred = forward_network(camera_message_curr, camera_message_base);
		delta_pose_pred = camera_carmen_transform(delta_pose_pred);
	}
	else
		delta_pose_pred = delta_pose_true;

	//network is trained with the base pose w.r.t. the live
	//delta_pose_pred = inverse_transform(delta_pose_pred, camera_pose_zero);
	//delta_pose_true = inverse_transform(delta_pose_true, camera_pose_zero);

	camera_pose_curr = direct_transform(camera_pose_base, delta_pose_pred);
	camera_pose_base.position.z = 1.7;
	camera_pose_curr.position.z = 1.7;

	carmen_point_t delta_pose_error = {
			(delta_pose_pred.position.x-delta_pose_true.position.x),
			(delta_pose_pred.position.y-delta_pose_true.position.y),
			(delta_pose_pred.orientation.yaw-delta_pose_true.orientation.yaw)
	};
	camera_delta_error_array.push_back(delta_pose_error);
	//carmen_warn("(%.2f %.2f %.3f)\n", delta_pose_error.x, delta_pose_error.y, delta_pose_error.theta);

	publish_imagepos_messages(camera_pose_true, camera_pose_base, camera_pose_curr, camera_timestamp);
}


void
carmen_bumblebee_basic_stereoimage_message_handler_cnn(carmen_bumblebee_basic_stereoimage_message *message)
{
	//	carmen_pose_3D_t camera_pose_zero = {{0.0,0.0,0.0},{0.0,0.0,0.0}};
	carmen_pose_3D_t camera_pose_base;
	carmen_pose_3D_t camera_pose_curr;
	carmen_pose_3D_t camera_pose_true;
	carmen_pose_3D_t delta_pose_pred;
	carmen_pose_3D_t delta_pose_true;

	double camera_timestamp = message->timestamp;
	/*
	int index_base = find_more_similar_base_pose(message);
	if (index_base < 0)
	{
		carmen_warn("Not enough frames!");
		return;
	}
	 */
	int index_delta = find_more_synchronized_pose(camera_delta_poses_array, camera_timestamp);
	if (index_delta < 0)
	{
		//carmen_warn("Frame lost!\n");
		return;
	}

	string key_frame = camera_delta_frames_array[index_delta].first;
	string cur_frame = camera_delta_frames_array[index_delta].second;

	read_image_file(&camera_message_base, key_frame);
	read_image_file(&camera_message_curr, cur_frame);
	//read_image_message(&camera_message_curr, message);

	camera_pose_base = camera_poses_base_array[index_delta].first;
	camera_pose_true = camera_poses_curr_array[index_delta].first;

	//just in case you want to check transforms
	//carmen_pose_3D_t delta_pose_true = inverse_transform(camera_pose_true, camera_pose_base);
	delta_pose_true = camera_delta_poses_array[index_delta].first;
	delta_pose_true = camera_carmen_transform(delta_pose_true);

	if (cheating)
	{
		delta_pose_pred = delta_pose_true;
	}
	else
	{
		delta_pose_pred = forward_network(camera_message_curr, camera_message_base);
		delta_pose_pred = camera_carmen_transform(delta_pose_pred);
	}
	//network is trained with the base pose w.r.t. the live
	//delta_pose_pred = inverse_transform(delta_pose_pred, camera_pose_zero);
	//delta_pose_true = inverse_transform(delta_pose_true, camera_pose_zero);

	camera_pose_curr = direct_transform(camera_pose_base, delta_pose_pred);
	camera_pose_base.position.z = 1.7;
	camera_pose_curr.position.z = 1.7;

	carmen_point_t delta_pose_error = {
			(delta_pose_pred.position.x-delta_pose_true.position.x),
			(delta_pose_pred.position.y-delta_pose_true.position.y),
			(delta_pose_pred.orientation.yaw-delta_pose_true.orientation.yaw)
	};
	camera_delta_error_array.push_back(delta_pose_error);
	//carmen_warn("(%.2f %.2f %.3f)\n", delta_pose_error.x, delta_pose_error.y, delta_pose_error.theta);

	publish_imagepos_messages(camera_pose_true, camera_pose_base, camera_pose_curr, camera_timestamp);
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		finalize_network();
		finalize_wnn();
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
	char camera_string[256];

	carmen_param_t param_cmd_list[] =
	{
			{(char *) "commandline", (char *) "camera_id", CARMEN_PARAM_INT, &camera, 0, NULL},
			{(char *) "commandline", (char *) "cnn_model", CARMEN_PARAM_STRING, &cnn_model, 0, NULL},
			{(char *) "commandline", (char *) "cnn_data", CARMEN_PARAM_STRING, &cnn_data, 0, NULL},
			{(char *) "commandline", (char *) "wnn_model", CARMEN_PARAM_STRING, &wnn_model, 0, NULL},
			{(char *) "commandline", (char *) "wnn_data", CARMEN_PARAM_STRING, &wnn_data, 0, NULL},
			{(char *) "commandline", (char *) "cheating", CARMEN_PARAM_INT, &cheating, 0, NULL}
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, param_cmd_list, sizeof(param_cmd_list)/sizeof(param_cmd_list[0]));

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

	carmen_param_allow_unfound_variables(0);
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list)/sizeof(param_list[0]));

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
	carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) carmen_bumblebee_basic_stereoimage_message_handler_cnn, CARMEN_SUBSCRIBE_LATEST);
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

	if (!cheating)
	{
		initialize_network(cnn_model);

		initialize_wnn(); train_wnn(wnn_data);
	}

	load_base_poses(wnn_data);

	load_delta_poses(cnn_data);

	signal(SIGINT, shutdown_module);

	subscribe_to_relevant_messages();

	carmen_ipc_addPeriodicTimer(30.0, (TIMER_HANDLER_TYPE) compute_statistics, NULL);

	carmen_warn("Ready!");

	carmen_ipc_dispatch();

	return(0);
}
