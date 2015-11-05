 /*********************************************************
	---   Skeleton Module Application ---
**********************************************************/

#include <carmen/carmen.h>
#include <carmen/slam6d_interface.h>
#include <carmen/kinect_interface.h>
#include <vector>

#include <kinfu_wrapper.h>
#include "slam6d_opengl_thread.h"
#include <Eigen/Core>
#include <opencv/cv.h>
#include <opencv/highgui.h>

static double slam6d_volume_size_g;
static int kinect_width_g, kinect_height_g;
static double kinect_depth_focal_length_x_g, kinect_depth_focal_length_y_g,
							kinect_rgb_focal_length_x_g, kinect_rgb_focal_length_y_g,
							kinect_rgb_center_x_g, kinect_rgb_center_y_g,
							kinect_baseline_g, kinect_max_depth_g;

double position_g[3];
double orientation_g[3];
double rotation_g[9];

int is_cuda_enable_g;

std::vector<carmen_slam6d_kinect_fused_t> keyframe_chain_g;
carmen_pose_3D_t last_keyframe_pose_g;

int count_init_kinect_interpolation_g;
int kinect_video_message_size_g;
int count_kinect_video_messages_g;

int handler_pass = 0;
bool has_new_frame = false;

carmen_slam6d_pointcloud_message* slam6d_pointcloud_g = NULL;
carmen_slam6d_kinect_fused_t* kinect_interpolated_message_g = NULL;
carmen_slam6d_kinect_fused_t* kinect_interpolated_message_prev_g = NULL;
carmen_kinect_video_message* kinect_video_message_list_g = NULL;

Thread opengl;
unsigned short int m_gamma[2048];

void
initialize_last_keyframe_pose()
{
	last_keyframe_pose_g.position.x = 0.0;
	last_keyframe_pose_g.position.y = 0.0;
	last_keyframe_pose_g.position.z = 0.0;
	last_keyframe_pose_g.orientation.yaw = 0.0;
	last_keyframe_pose_g.orientation.pitch = 0.0;
	last_keyframe_pose_g.orientation.roll = 0.0;

	keyframe_chain_g.clear();
}

void
initialize_slam6d_pointcloud_message()
{
	if(slam6d_pointcloud_g == NULL)
		slam6d_pointcloud_g = (carmen_slam6d_pointcloud_message *) malloc (sizeof(carmen_slam6d_pointcloud_message));
}

void
initialize_kinect_interpolated_message()
{
	if(kinect_interpolated_message_g == NULL)
		kinect_interpolated_message_g = (carmen_slam6d_kinect_fused_t *) malloc (sizeof(carmen_slam6d_kinect_fused_t));

	kinect_interpolated_message_g->depth = (unsigned short *) malloc ((kinect_height_g * kinect_width_g) * sizeof(unsigned short));
	kinect_interpolated_message_g->image = (unsigned char *) malloc ((kinect_height_g * kinect_width_g * 3) * sizeof(unsigned char));

	if(kinect_interpolated_message_prev_g == NULL)
		kinect_interpolated_message_prev_g = (carmen_slam6d_kinect_fused_t *) malloc (sizeof(carmen_slam6d_kinect_fused_t));

	kinect_interpolated_message_prev_g->depth = (unsigned short *) malloc ((kinect_height_g * kinect_width_g) * sizeof(unsigned short));
	kinect_interpolated_message_prev_g->image = (unsigned char *) malloc ((kinect_height_g * kinect_width_g * 3) * sizeof(unsigned char));
}

void
initialize_kinect_video_message_list(int size_of_list)
{
	kinect_video_message_size_g = size_of_list;

	if(kinect_video_message_list_g == NULL)
		kinect_video_message_list_g = (carmen_kinect_video_message *) malloc (kinect_video_message_size_g * sizeof(carmen_kinect_video_message));

	for(int i=0; i < kinect_video_message_size_g; i++)
		kinect_video_message_list_g[i].video = (unsigned char*) malloc ((kinect_height_g * kinect_width_g * 3) * sizeof(unsigned char));
}


void
init_gamma()
{
  unsigned int i;
  for(i = 0 ; i < 2048 ; i++) {
    float v = i/2048.0;
    v = pow(v, 3)* 6;
    m_gamma[i] = v*6*256;
  }
}

void
initialize_slam6d()
{
	//char * sift_params[] ={ (char*) "-fo", (char*) "-1", (char*) "-v", (char*) "1"};

	count_init_kinect_interpolation_g = 0;
	count_kinect_video_messages_g = 0;

	//init_gamma();
	initialize_kinect_video_message_list(30);
	initialize_kinect_interpolated_message();
	initialize_slam6d_pointcloud_message();
	initialize_last_keyframe_pose();

	//initialize_siftgpu(sift_params, true);
	initialize_kinfu(kinect_width_g, kinect_height_g, kinect_depth_focal_length_x_g, kinect_depth_focal_length_y_g, kinect_baseline_g, kinect_max_depth_g, slam6d_volume_size_g, 0);
}

void
add_message_to_kinect_video_message_list(carmen_kinect_video_message* message)
{
	kinect_video_message_list_g[count_kinect_video_messages_g].id = message->id;
	kinect_video_message_list_g[count_kinect_video_messages_g].width = message->width;
	kinect_video_message_list_g[count_kinect_video_messages_g].height = message->height;
	kinect_video_message_list_g[count_kinect_video_messages_g].size = message->size;

	for(int i = 0; i < message->size; i++)
		kinect_video_message_list_g[count_kinect_video_messages_g].video[i] = message->video[i];

	kinect_video_message_list_g[count_kinect_video_messages_g].timestamp = message->timestamp;
	kinect_video_message_list_g[count_kinect_video_messages_g].host = message->host;

	if(count_kinect_video_messages_g == kinect_video_message_size_g - 1)
		count_kinect_video_messages_g = 0;

	count_kinect_video_messages_g++;
}

carmen_slam6d_kinect_fused_t *
interpolate_kinect_video_and_depth_messages(carmen_kinect_depth_message* message)
{
	int video_index = 0;
	double curr_timestamp_difference;
	double min_timestamp_difference = 99999999.0;

	for(int i = 0; i < kinect_video_message_size_g; i++)
	{
		curr_timestamp_difference = fabs(kinect_video_message_list_g[i].timestamp - message->timestamp);

		if(curr_timestamp_difference < min_timestamp_difference)
		{
			min_timestamp_difference = curr_timestamp_difference;
			video_index = i;
		}
	}

	if(kinect_interpolated_message_g != NULL)
	{
		kinect_interpolated_message_g->is_keyframe = false;
		kinect_interpolated_message_g->is_loop_closure = false;

		for(int i=0; i < kinect_width_g * kinect_height_g * 3; i++)
			kinect_interpolated_message_g->image[i] = kinect_video_message_list_g[video_index].video[i];

		for(int i=0; i < kinect_width_g * kinect_height_g; i++)
			kinect_interpolated_message_g->depth[i] = (unsigned short) (message->depth[i] * 1000); //meters to milimeters

		kinect_interpolated_message_g->image_timestamp = kinect_video_message_list_g[video_index].timestamp;
		kinect_interpolated_message_g->depth_timestamp = message->timestamp;
	}

	return kinect_interpolated_message_g;
}

void
copy_global_kinect_current_interpolated_message_to_previous_message()
{
	kinect_interpolated_message_prev_g->position[0] = kinect_interpolated_message_g->position[0];
	kinect_interpolated_message_prev_g->position[1] = kinect_interpolated_message_g->position[1];
	kinect_interpolated_message_prev_g->position[2] = kinect_interpolated_message_g->position[2];

	kinect_interpolated_message_prev_g->rotation[0] = kinect_interpolated_message_g->rotation[0];
	kinect_interpolated_message_prev_g->rotation[1] = kinect_interpolated_message_g->rotation[1];
	kinect_interpolated_message_prev_g->rotation[2] = kinect_interpolated_message_g->rotation[2];
	kinect_interpolated_message_prev_g->rotation[3] = kinect_interpolated_message_g->rotation[3];
	kinect_interpolated_message_prev_g->rotation[4] = kinect_interpolated_message_g->rotation[4];
	kinect_interpolated_message_prev_g->rotation[5] = kinect_interpolated_message_g->rotation[5];
	kinect_interpolated_message_prev_g->rotation[6] = kinect_interpolated_message_g->rotation[6];
	kinect_interpolated_message_prev_g->rotation[7] = kinect_interpolated_message_g->rotation[7];
	kinect_interpolated_message_prev_g->rotation[8] = kinect_interpolated_message_g->rotation[8];


	kinect_interpolated_message_prev_g->is_keyframe = kinect_interpolated_message_g->is_keyframe;
	kinect_interpolated_message_prev_g->is_loop_closure = kinect_interpolated_message_g->is_loop_closure;

	for(int i=0; i < kinect_width_g * kinect_height_g * 3; i++)
		kinect_interpolated_message_prev_g->image[i] = kinect_interpolated_message_g->image[i];

	for(int i=0; i < kinect_width_g * kinect_height_g; i++)
		kinect_interpolated_message_prev_g->depth[i] = kinect_interpolated_message_g->depth[i]; //meters to milimeters

	kinect_interpolated_message_prev_g->image_timestamp = kinect_interpolated_message_g->image_timestamp;
	kinect_interpolated_message_prev_g->depth_timestamp = kinect_interpolated_message_g->depth_timestamp;
}

double frame_position_difference(double* frame_position, carmen_vector_3D_t last_frame_position)
{
	return sqrt((frame_position[0] - last_frame_position.x)*(frame_position[0] - last_frame_position.x) +
				(frame_position[1] - last_frame_position.y)*(frame_position[1] - last_frame_position.y) +
				(frame_position[2] - last_frame_position.z)*(frame_position[2] - last_frame_position.z));
}

double frame_orientation_difference(double* frame_orientation, carmen_orientation_3D_t last_frame_orientation, int ref)
{
	double difference = 0.0;

	switch(ref)
	{
		//pitch diff
		case 0:
			difference = fabs(frame_orientation[0]-last_frame_orientation.pitch);
			break;

		//roll diff
		case 1:
			difference = fabs(frame_orientation[1]-last_frame_orientation.roll);
			break;

		//yaw diff
		case 2:
			difference = fabs(frame_orientation[2]-last_frame_orientation.yaw);
			break;
		default:
			break;
	}

	return difference;
}

bool
check_kinect_interpolated_message_g_is_keyframe(double* frame_position, double* frame_orientation, double off_set, double angle)
{
	if(count_init_kinect_interpolation_g == 1)
	{
		kinect_interpolated_message_g->is_keyframe = true;

		last_keyframe_pose_g.position.x = frame_position[0];
		last_keyframe_pose_g.position.y = frame_position[1];
		last_keyframe_pose_g.position.z = frame_position[2];
		last_keyframe_pose_g.orientation.pitch = frame_orientation[0];
		last_keyframe_pose_g.orientation.roll = frame_orientation[1];
		last_keyframe_pose_g.orientation.yaw = frame_orientation[2];

		return true;
	}
	else
	{
		if(frame_position_difference(frame_position, last_keyframe_pose_g.position) > off_set)
		{
			kinect_interpolated_message_g->is_keyframe = true;

			last_keyframe_pose_g.position.x = frame_position[0];
			last_keyframe_pose_g.position.y = frame_position[1];
			last_keyframe_pose_g.position.z = frame_position[2];
			last_keyframe_pose_g.orientation.pitch = frame_orientation[0];
			last_keyframe_pose_g.orientation.roll = frame_orientation[1];
			last_keyframe_pose_g.orientation.yaw = frame_orientation[2];

			return true;
		}
		else
			kinect_interpolated_message_g->is_keyframe = false;

		if(frame_orientation_difference(frame_orientation, last_keyframe_pose_g.orientation, 0) > angle ||
		   frame_orientation_difference(frame_orientation, last_keyframe_pose_g.orientation, 1) > angle ||
		   frame_orientation_difference(frame_orientation, last_keyframe_pose_g.orientation, 2) > angle)
		{
			kinect_interpolated_message_g->is_keyframe = true;

			last_keyframe_pose_g.position.x = frame_position[0];
			last_keyframe_pose_g.position.y = frame_position[1];
			last_keyframe_pose_g.position.z = frame_position[2];
			last_keyframe_pose_g.orientation.pitch = frame_orientation[0];
			last_keyframe_pose_g.orientation.roll = frame_orientation[1];
			last_keyframe_pose_g.orientation.yaw = frame_orientation[2];

			return true;
		}
		else
		{
			kinect_interpolated_message_g->is_keyframe = false;
		}
	}

	return false;
}

void copy_raw_image_to_opencv_ipl_image(unsigned char* image, IplImage* opencv_image)
{
	for(int i = 0; i < opencv_image->width * opencv_image->height; i++)
	{
		opencv_image->imageData[3 * i] = 		 (char) image[3 * i + 2];
		opencv_image->imageData[3 * i + 1] = (char) image[3 * i + 1];
		opencv_image->imageData[3 * i + 2] = (char) image[3 * i];
	}
}

void copy_raw_depth_to_opencv_ipl_image(unsigned short* depth, IplImage* opencv_image)
{
	unsigned short value_depth;

	for(int i = 0; i < opencv_image->width * opencv_image->height; i++)
	{
		value_depth = convert_kinect_depth_meters_to_raw(depth[i]/1000.0);

		int pval = m_gamma[value_depth];
		int lb = pval & 0xff;
		switch (pval>>8) {
		case 0:
			opencv_image->imageData[i] = (char) ((3 * 255 - 2 * lb) / 3);
			break;
		case 1:
			opencv_image->imageData[i] = (char) ((255 + lb) / 3);
			break;
		case 2:
			opencv_image->imageData[i] = (char) ((2 * 255 - lb) / 3);
			break;
		case 3:
			opencv_image->imageData[i] = (char) ((255 + lb) / 3);
			break;
		case 4:
			opencv_image->imageData[i] = (char) ((2 * 255 - lb) / 3);
			break;
		case 5:
			opencv_image->imageData[i] = (char) ((255 - lb) / 3);
			break;
		default:
			opencv_image->imageData[i] = (char) 0;
			break;
		}
	}
}

void copy_ipl_image_to_raw_image(IplImage* opencv_image, unsigned char* image)
{
	for(int i = 0; i < opencv_image->width * opencv_image->height; i++)
	{
		image[3 * i] = 		 (unsigned char) opencv_image->imageData[3 * i + 2];
		image[3 * i + 1] = (unsigned char) opencv_image->imageData[3 * i + 1];
		image[3 * i + 2] = (unsigned char) opencv_image->imageData[3 * i];
	}
}

void copy_ipl_image_to_raw_depth( IplImage* opencv_image, unsigned short* depth)
{
	for(int i = 0; i < opencv_image->width * opencv_image->height; i++)
		depth[i] = (unsigned short) opencv_image->imageData[i];
}

void
add_keyframe_message_to_graph_chain(carmen_slam6d_kinect_fused_t* keyframe)
{
	carmen_slam6d_kinect_fused_t local_keyframe;
	local_keyframe.depth = (unsigned short*) malloc (kinect_height_g * kinect_width_g * sizeof(unsigned short));
	local_keyframe.image = (unsigned char*) malloc (3 * kinect_height_g * kinect_width_g * sizeof(unsigned char));

	memcpy(local_keyframe.depth, keyframe->depth, kinect_height_g * kinect_width_g * sizeof(unsigned short));
	memcpy(local_keyframe.image, keyframe->image, 3 * kinect_height_g * kinect_width_g * sizeof(unsigned char));
//	IplImage* distorted_image = cvCreateImage(cvSize(kinect_width_g, kinect_height_g), IPL_DEPTH_8U, 3);
//	IplImage* distorted_depth = cvCreateImage(cvSize(kinect_width_g, kinect_height_g), IPL_DEPTH_16U, 1);
//	IplImage* undistorted_image = cvCreateImage(cvSize(kinect_width_g, kinect_height_g), IPL_DEPTH_8U, 3);
//	IplImage* undistorted_depth = cvCreateImage(cvSize(kinect_width_g, kinect_height_g), IPL_DEPTH_16U, 1);
//
//	double cm_data[9] = {530.72778403, 0.0, 313.01038359,
//										0.0, 530.18957572, 263.5122859,
//										0.0, 0.0, 1.0 };
//
//	double coeff_data[5] = {0.36228793, -2.89053824, -0.000211240, -0.004548858, 12.900147450};
//
//	CvMat camera_matrix = cvMat(3, 3, CV_1F, cm_data);
//	CvMat distortion_coeff = cvMat(1, 5, CV_1F, coeff_data);
//
//	copy_raw_image_to_opencv_ipl_image(keyframe->image, distorted_image);
//	//copy_raw_depth_to_opencv_ipl_image(keyframe->depth, distorted_depth);
//
//	cvShowImage("image", distorted_image);
//	cvWaitKey(-1);
//
//	cvUndistort2(distorted_image, undistorted_image, &camera_matrix, &distortion_coeff);
//	//cvUndistort2(distorted_depth, undistorted_depth, &camera_matrix, &distortion_coeff);
//
//	cvShowImage("image2", undistorted_image);
//	cvWaitKey(-1);
//
//	copy_ipl_image_to_raw_image(undistorted_image, local_keyframe.image);
//	//copy_ipl_image_to_raw_depth(undistorted_depth, local_keyframe.depth);

	local_keyframe.is_keyframe = keyframe->is_keyframe;
	local_keyframe.is_loop_closure = keyframe->is_loop_closure;

	local_keyframe.position[0] = keyframe->position[0];
	local_keyframe.position[1] = keyframe->position[1];
	local_keyframe.position[2] = keyframe->position[2];

	local_keyframe.rotation[0] = keyframe->rotation[0];
	local_keyframe.rotation[1] = keyframe->rotation[1];
	local_keyframe.rotation[2] = keyframe->rotation[2];
	local_keyframe.rotation[3] = keyframe->rotation[3];
	local_keyframe.rotation[4] = keyframe->rotation[4];
	local_keyframe.rotation[5] = keyframe->rotation[5];
	local_keyframe.rotation[6] = keyframe->rotation[6];
	local_keyframe.rotation[7] = keyframe->rotation[7];
	local_keyframe.rotation[8] = keyframe->rotation[8];

	local_keyframe.image_timestamp = keyframe->image_timestamp;
	local_keyframe.depth_timestamp = keyframe->depth_timestamp;

	keyframe_chain_g.push_back(local_keyframe);
}

void
add_position_information_to_kinect_interpolated_message_g(double* position_g, double* rotation_g)
{
	kinect_interpolated_message_g->position[0] = position_g[0];
	kinect_interpolated_message_g->position[1] = position_g[1];
	kinect_interpolated_message_g->position[2] = position_g[2];

	kinect_interpolated_message_g->rotation[0] = rotation_g[0];
	kinect_interpolated_message_g->rotation[1] = rotation_g[1];
	kinect_interpolated_message_g->rotation[2] = rotation_g[2];
	kinect_interpolated_message_g->rotation[3] = rotation_g[3];
	kinect_interpolated_message_g->rotation[4] = rotation_g[4];
	kinect_interpolated_message_g->rotation[5] = rotation_g[5];
	kinect_interpolated_message_g->rotation[6] = rotation_g[6];
	kinect_interpolated_message_g->rotation[7] = rotation_g[7];
	kinect_interpolated_message_g->rotation[8] = rotation_g[8];
}

/*********************************************************
		   --- Publishers ---
**********************************************************/

carmen_slam6d_pointcloud_message *
assembly_slam6d_pointcloud_message(carmen_slam6d_kinect_fused_t* kinect_interpolated_message)
{
	if(slam6d_pointcloud_g != NULL)
	{
		slam6d_pointcloud_g->width = kinect_width_g;
		slam6d_pointcloud_g->height = kinect_height_g;
		slam6d_pointcloud_g->depth_size = kinect_width_g * kinect_height_g;
		slam6d_pointcloud_g->image_size = kinect_width_g * kinect_height_g * 3;
		slam6d_pointcloud_g->depth = kinect_interpolated_message->depth;
		slam6d_pointcloud_g->image = kinect_interpolated_message->image;
		slam6d_pointcloud_g->timestamp = kinect_interpolated_message->image_timestamp;

		if(kinect_interpolated_message->is_keyframe)
			slam6d_pointcloud_g->is_keyframe = 1;
		else
			slam6d_pointcloud_g->is_keyframe = 0;

		for(int i=0; i < 3; i++)
			slam6d_pointcloud_g->position[i] = kinect_interpolated_message->position[i];

		for(int i=0; i < 9; i++)
			slam6d_pointcloud_g->rotation[i] = kinect_interpolated_message->rotation[i];

		slam6d_pointcloud_g->host = carmen_get_host();
	}

	return slam6d_pointcloud_g;
}

void
publish_slam6d_pointcloud_message(carmen_slam6d_pointcloud_message* message)
{
	IPC_RETURN_TYPE err;

	err = IPC_publishData(CARMEN_SLAM6D_POINTCLOUD_MESSAGE_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_SLAM6D_POINTCLOUD_MESSAGE_FMT);
}

/*********************************************************
		   --- Handlers ---
**********************************************************/

void
kinect_image_message_handler(carmen_kinect_video_message *message)
{
	add_message_to_kinect_video_message_list(message);
}

void
kinect_depth_message_handler(carmen_kinect_depth_message* message)
{
		if(count_init_kinect_interpolation_g == 2)
		{
		kinect_interpolated_message_g = interpolate_kinect_video_and_depth_messages(message);
		upload_depth_to_kinfu(kinect_interpolated_message_g->depth);

		if (execute_kinfu(position_g, rotation_g, orientation_g))
		{
			add_position_information_to_kinect_interpolated_message_g(position_g, rotation_g);

			if(handler_pass < 2)
			{
				add_keyframe_message_to_graph_chain(kinect_interpolated_message_prev_g);
				has_new_frame = true;
				printf("keyframes: %d\n", (int)keyframe_chain_g.size());
			}
			else if(check_kinect_interpolated_message_g_is_keyframe(position_g, orientation_g, 0.5, 15.0))
			{
				add_keyframe_message_to_graph_chain(kinect_interpolated_message_prev_g);
				has_new_frame = true;
				printf("keyframes: %d\n", (int)keyframe_chain_g.size());
			}

			handler_pass++;

			if(handler_pass > 2)
				handler_pass = 2;


			//check loop closure

			printf("x: %6.2f, y: %6.2f, z: %6.2f, yaw: %6.2f, pitch: %6.2f, roll: %6.2f\n", position_g[0], position_g[1], position_g[2], orientation_g[2], orientation_g[0], orientation_g[1]);
		}
		else
			printf("failed!\n");

		copy_global_kinect_current_interpolated_message_to_previous_message();
	}
	else
		count_init_kinect_interpolation_g++;
}

void 
shutdown_module(int signo)
{
  if (signo == SIGINT)
  {
    carmen_ipc_disconnect();
    printf("slam6d: disconnected.\n");

    exit(0);
  }
}

static int read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
	  {(char *)"slam6d", (char *)"volume_size", CARMEN_PARAM_DOUBLE, &slam6d_volume_size_g, 0, NULL},
	  {(char *)"slam6d", (char *)"cuda_enable", CARMEN_PARAM_INT, &is_cuda_enable_g, 0, NULL},
	  {(char *)"kinect", (char *)"width", CARMEN_PARAM_INT, &kinect_width_g, 0, NULL},
	  {(char *)"kinect", (char *)"height", CARMEN_PARAM_INT, &kinect_height_g, 0, NULL},
	  {(char *)"kinect", (char *)"depth_focal_length_x", CARMEN_PARAM_DOUBLE, &kinect_depth_focal_length_x_g, 0, NULL},
	  {(char *)"kinect", (char *)"depth_focal_length_y", CARMEN_PARAM_DOUBLE, &kinect_depth_focal_length_y_g, 0, NULL},
	  {(char *)"kinect", (char *)"rgb_focal_length_x", CARMEN_PARAM_DOUBLE, &kinect_rgb_focal_length_x_g, 0, NULL},
		{(char *)"kinect", (char *)"rgb_focal_length_y", CARMEN_PARAM_DOUBLE, &kinect_rgb_focal_length_y_g, 0, NULL},
	  {(char *)"kinect", (char *)"rgb_center_x", CARMEN_PARAM_DOUBLE, &kinect_rgb_center_x_g, 0, NULL},
		{(char *)"kinect", (char *)"rgb_center_y", CARMEN_PARAM_DOUBLE, &kinect_rgb_center_y_g, 0, NULL},
	  {(char *)"kinect", (char *)"baseline", CARMEN_PARAM_DOUBLE, &kinect_baseline_g, 0, NULL},
	  {(char *)"kinect", (char *)"max_depth", CARMEN_PARAM_DOUBLE, &kinect_max_depth_g, 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}

void ipc_callbacks(int ms)
{
	IPC_handleMessage(0);
	glutTimerFunc(30, ipc_callbacks, 30);
}

void
recostrucao3d (unsigned short* depth, unsigned char* color, float* vmap, float *cmap, int depth_width, int depth_height, double dfx_inv, double dfy_inv, double rfx, double rfy, double d_cx, double d_cy, double rgb_cx, double rgb_cy/*,double* Rmat, double* tvec*/)
{//copy of computeVmapKernel in kernlsVbo.cu
   // int u = threadIdx.x + blockIdx.x * blockDim.x;
  //  int v = threadIdx.y + blockIdx.y * blockDim.y;
    int pos = 0, posc = 0;
    float z, vx, vy, vz/*, avx, avy, avz*/;
    int clx, cly;

	for (int v = 0; v < depth_height; v++)
	{
		for (int u = 0; u < depth_width; u++)
		{
		    if (u < depth_width && v < depth_height)
		    {
				pos = v * depth_width + u;			//i * width + j; 

				z = depth[pos] / 1000.0f; // load and convert: mm -> meters

				if (z >= 0.5f)
				{
					vx = z * (u - d_cx) * dfx_inv;
					vy = z * (v - d_cy) * dfy_inv;
					vz = z;

					clx = (int) (((vx * (rfx)) / vz) + rgb_cx);
					cly = (int) (((vy * (rfy)) / vz) + rgb_cy);

					/*avx = (float)((Rmat[0] * vx + Rmat[1] * vy + Rmat[2] * vz) + tvec[0]);
					avy = (float)((Rmat[3] * vx + Rmat[4] * vy + Rmat[5] * vz) + tvec[1]);
					avz = (float)((Rmat[6] * vx + Rmat[7] * vy + Rmat[8] * vz) + tvec[2]);*/

					/*vx = -avx;
					vy = -avy;
					vz = avz;*/			

					posc = cly * depth_width + clx;
			
					vmap[3 * pos] = vx;
					vmap[3 * pos + 1] = vy;
					vmap[3 * pos + 2] = vz;

					if((cly >=0 && cly < depth_height) && (clx >= 0 && clx < depth_width))
					{
					cmap[3 * pos] = color[3 * posc] / 255.0f;
					cmap[3 * pos + 1] = color[3 * posc + 1] / 255.0f;
					cmap[3 * pos + 2] = color[3 * posc + 2] / 255.0f;
					}
				}
				else
				{
					vmap[3 * pos] = 0.0f;
					vmap[3 * pos + 1] = 0.0f;
					vmap[3 * pos + 2] = 0.0f;

					cmap[3 * pos] = 0;
					cmap[3 * pos + 1] = 0.0f;
					cmap[3 * pos + 2] = 0.0f;
				}
	    		}
		}
	}
}

void upload_last_frame_in_chain(int ms)
{
	float* point_cloud, *point_cloud_colors;
	point_cloud = (float*)malloc(sizeof(float)*(3*kinect_width_g*kinect_height_g));
	point_cloud_colors = (float*)malloc(sizeof(float)*(3*kinect_width_g*kinect_height_g));

	if(has_new_frame)
	{
		if(is_cuda_enable_g)
		{	
		opengl.UploadChainCuda(keyframe_chain_g, kinect_width_g, kinect_height_g, kinect_depth_focal_length_x_g, kinect_depth_focal_length_y_g, kinect_rgb_focal_length_x_g, kinect_rgb_focal_length_y_g, kinect_rgb_center_x_g, kinect_rgb_center_y_g);
		}
		else
		{
			//reconstrucao 3d;
			recostrucao3d (keyframe_chain_g[keyframe_chain_g.size()-1].depth, keyframe_chain_g[keyframe_chain_g.size()-1].image, point_cloud, point_cloud_colors, kinect_width_g, kinect_height_g, 1/kinect_depth_focal_length_x_g, 1/kinect_depth_focal_length_y_g, kinect_rgb_focal_length_x_g, kinect_rgb_focal_length_y_g, kinect_rgb_center_x_g, kinect_rgb_center_y_g, kinect_rgb_center_x_g, kinect_rgb_center_y_g);

			opengl.Upload(point_cloud, point_cloud_colors);
		}

		has_new_frame = false;
	}
}
  
int 
main(int argc, char **argv) 
{
  /* Connect to IPC Server */
  carmen_ipc_initialize(argc, argv);

  /* Check the param server version */
  carmen_param_check_version(argv[0]);

  /* Register shutdown cleaner handler */
  signal(SIGINT, shutdown_module);

  /* Read carmen.ini dependent parameters */
  read_parameters(argc, argv);

  initialize_slam6d();

  /* Define messages that your module publishes */
  carmen_slam6d_define_messages();

  /* Subscribe to sensor messages */
  carmen_kinect_subscribe_depth_message(0, NULL, (carmen_handler_t) kinect_depth_message_handler, CARMEN_SUBSCRIBE_LATEST);
  carmen_kinect_subscribe_video_message(0, NULL, (carmen_handler_t) kinect_image_message_handler, CARMEN_SUBSCRIBE_LATEST);

  opengl.Start(argc, argv, 307200, 3, 10, upload_last_frame_in_chain, is_cuda_enable_g, 0);

//  run_glut();
  carmen_ipc_dispatch();
}
