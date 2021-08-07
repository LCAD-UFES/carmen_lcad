
/**
 * @description
 * This code have been created to export camera poses
 *
 * @author avelino forechi
 */

#include <stdio.h>
#include <string.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <carmen/carmen.h>
#include <carmen/gps_nmea_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/web_cam_interface.h>
#include <carmen/stereo_interface.h>
#include <carmen/base_ackerman_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/xsens_interface.h>

//#include "localize_neural_util.h"

#include <tf.h>

tf::Transformer transformer(false);

static int camera;
static int camera_type;	// 0 - Bumblebee, 1 - Intelbras, 2 - velodyne, 3 - lidar
static int bumblebee_basic_width;
static int bumblebee_basic_height;
static carmen_pose_3D_t car_pose_g;
static carmen_pose_3D_t camera_pose_g;
static carmen_pose_3D_t board_pose_g;
static double between_axis_distance = 0.0;

static char* output_dir_name;
static char* image_pose_output_filename;
static char* log_filename;
static FILE* image_pose_output_file = NULL;
carmen_FILE *outfile = NULL;
static int globalpos_message_index = 0;
static carmen_localize_ackerman_globalpos_message globalpos_message_buffer[100];


int
find_nearest_globalpos_message(double timestamp)
{
	int nearest_index = -1;
	double shortest_interval = MAXDOUBLE;

	for (int i = 0; i < 10000; i++)
	{
		if (globalpos_message_buffer[i].host != NULL)
		{
			double delta_t = timestamp - globalpos_message_buffer[i].timestamp;
			if ((delta_t >= 0.0) && (delta_t < shortest_interval))
			{
				shortest_interval = delta_t;
				nearest_index = i;
			}
		}
	}
	return nearest_index;
}


void create_lidar_filename_from_timestamp(double timestamp, char **pointcloud_filename, int camera_type)
{
	//const double HIGH_LEVEL_SUBDIR_TIME = 100.0 * 100.0; // new each 100 x 100 seconds
	//const double LOW_LEVEL_SUBDIR_TIME = 100.0; // new each 100 seconds

	int high_level_subdir = ((int) (timestamp / HIGH_LEVEL_SUBDIR_TIME))
			* HIGH_LEVEL_SUBDIR_TIME;
	int low_level_subdir = ((int) (timestamp / LOW_LEVEL_SUBDIR_TIME))
			* LOW_LEVEL_SUBDIR_TIME;

	int i;
	static char directory[1024];
	static char subdir[1024];
	static char path[1024];

	/**
	 * TODO: @Filipe: Check if the mkdir call is time consuming.
	 */
	if(camera_type==2)
		sprintf(directory, "%s_velodyne", log_filename);
	else
		sprintf(directory, "%s_lidar", log_filename);
	//mkdir(directory, ACCESSPERMS); // if the directory exists, mkdir returns an error silently

	sprintf(subdir, "%s/%d", directory, high_level_subdir);
	///mkdir(subdir, ACCESSPERMS); // if the directory exists, mkdir returns an error silently

	sprintf(subdir, "%s/%d/%d", directory, high_level_subdir, low_level_subdir);
	//mkdir(subdir, ACCESSPERMS); // if the directory exists, mkdir returns an error silently
	
	sprintf(path, "%s/%lf.pointcloud", subdir, timestamp);
	
	*pointcloud_filename = (char *) malloc (
			(strlen(path) + 2 /* 1 for '\0' e 1 for the '/' between <dirname>/<filename> */) * sizeof(char));

	sprintf((*pointcloud_filename),"%s",path);

}

void
compose_output_path(char *dirname, char *filename, char **composed_path)
{
	*composed_path = (char *) malloc (
			(strlen(dirname) + strlen(filename) + 2 /* 1 for '\0' e 1 for the '/' between <dirname>/<filename> */) * sizeof(char));

	sprintf((*composed_path), "%s/%s", dirname, filename);
}


void
compose_filename_from_timestamp_bb(double timestamp, char **filename, char *extension, int camera)
{
	*filename = (char*) malloc (256 * sizeof(char));
	sprintf((*filename), "%lf.bb%d.%s", timestamp, camera, extension);
}


void
compose_filename_from_timestamp(double timestamp, char **filename, char *extension, int camera)
{
	*filename = (char*) malloc (256 * sizeof(char));
	sprintf((*filename), "%lf.intelbras%d%s", timestamp, camera, extension);
}


void
create_stereo_filename_from_timestamp(double timestamp, char **left_img_filename, char **right_img_filename, int camera)
{
	compose_filename_from_timestamp_bb(timestamp, left_img_filename, (char *)"l.png", camera);
	compose_filename_from_timestamp_bb(timestamp, right_img_filename, (char *)"r.png", camera);
}


void
create_image_filename_from_timestamp(double timestamp, char **img_filename, int camera)
{
	compose_filename_from_timestamp(timestamp, img_filename, (char *) ".png", camera);
}


void
save_image_to_file(carmen_bumblebee_basic_stereoimage_message *stereo_image, int camera)
{
	char *left_img_filename, *right_img_filename;
	char *left_composed_path, *right_composed_path;
	IplImage *left_img = cvCreateImage(cvSize(stereo_image->width, stereo_image->height), IPL_DEPTH_8U, 3);
	IplImage *right_img = cvCreateImage(cvSize(stereo_image->width, stereo_image->height), IPL_DEPTH_8U, 3);
	cv::Rect crop;
	crop.x = 0;
	crop.y = 0;
	crop.width = 640;
	crop.height = 380;

	create_stereo_filename_from_timestamp(stereo_image->timestamp, &left_img_filename, &right_img_filename, camera);
	compose_output_path(output_dir_name, left_img_filename, &left_composed_path);
	compose_output_path(output_dir_name, right_img_filename, &right_composed_path);

	// copy_image((char *)stereo_image->raw_left, left_img, stereo_image->width, stereo_image->height);
	// copy_image((char *)stereo_image->raw_right, right_img, stereo_image->width, stereo_image->height);

	// resize_image(&left_img, 640, 480);
	// resize_image(&right_img, 640, 480);

	cvSetImageROI(left_img, crop);
	cvSetImageROI(right_img, crop);

	cvSaveImage(left_composed_path, left_img, NULL);
	cvSaveImage(right_composed_path, right_img, NULL);

	//printf("left image saved: %s\n", left_composed_path);
	//printf("right image saved: %s\n", right_composed_path);

	free(left_img_filename);
	free(left_composed_path);
	free(right_img_filename);
	free(right_composed_path);

	cvRelease((void**) &left_img);
	cvRelease((void**) &right_img);
}


tf::Pose
get_camera_pose_wrt_world(carmen_point_t robot_pose)
{
	tf::Pose camera_pose;
	tf::Transform world_to_car_pose;
	tf::StampedTransform world_to_camera_pose;
	tf::StampedTransform camera_to_camera_zyx_pose;
	double roll, pitch, yaw;

	world_to_car_pose.setOrigin(tf::Vector3(robot_pose.x, robot_pose.y, car_pose_g.position.z));
	world_to_car_pose.setRotation(tf::Quaternion(robot_pose.theta, 0, 0));

	tf::StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
	transformer.setTransform(world_to_car_transform, "world_to_car_transform");

	transformer.lookupTransform("/world", "/camera", tf::Time(0), world_to_camera_pose);
	camera_pose = world_to_camera_pose;

	tf::Matrix3x3(camera_pose.getRotation()).getRPY(roll, pitch, yaw);
	carmen_warn("%lf %lf %lf %lf %lf %lf\n", camera_pose.getOrigin().x(), camera_pose.getOrigin().y(), camera_pose.getOrigin().z(), carmen_radians_to_degrees(yaw), carmen_radians_to_degrees(pitch), carmen_radians_to_degrees(roll));

	return camera_pose;
}


carmen_pose_3D_t
get_interpolated_pose_at_time(carmen_pose_3D_t robot_pose, double dt, double v, double phi)
{
	carmen_pose_3D_t pose = robot_pose;
	double ds = v * dt;

	pose.position.x = pose.position.x + ds * cos(pose.orientation.yaw);
	pose.position.y = pose.position.y + ds * sin(pose.orientation.yaw);
	pose.orientation.yaw = carmen_normalize_theta(pose.orientation.yaw + ds * (tan(phi) / between_axis_distance));

	return pose;
}

void
save_pose_to_file(carmen_velodyne_partial_scan_message *lidar)
{
	char *lidar_filename;

	if ((lidar == NULL) || (image_pose_output_file == NULL))
		carmen_die("no lidar received\n");
	
	int nearest_message_index = find_nearest_globalpos_message(lidar->timestamp);

	if (nearest_message_index < 0)
	{
		carmen_warn("nearest_message_index < 0\n");
		return;
	}

	carmen_localize_ackerman_globalpos_message globalpos_message = globalpos_message_buffer[nearest_message_index];
	carmen_pose_3D_t globalpos = globalpos_message.pose;
	globalpos.position.x = globalpos_message.globalpos.x;
	globalpos.position.y = globalpos_message.globalpos.y;
	globalpos.orientation.yaw = globalpos_message.globalpos.theta;

	double dt = lidar->timestamp - globalpos_message.timestamp;
	if (dt >= 0.0)
		globalpos = get_interpolated_pose_at_time(globalpos, dt, globalpos_message.v, globalpos_message.phi);
	else
		carmen_die("dt < 0\n");

	create_lidar_filename_from_timestamp(lidar->timestamp, &lidar_filename, camera_type);
	//printf("linha: %s\n", lidar_filename);
	fprintf(image_pose_output_file, "%lf %lf %lf %lf %lf %lf %lf %s\n",
			globalpos.position.x, globalpos.position.y, globalpos.position.z,	//tx, ty, tz
			globalpos.orientation.roll, globalpos.orientation.pitch, globalpos.orientation.yaw,		//rx, ry, rz,
			lidar->timestamp,
			lidar_filename);
	fflush(image_pose_output_file);
}

void
save_pose_to_file(camera_message *image, int camera)
{
	char *img_filename;

	if ((image == NULL) || (image_pose_output_file == NULL))
		carmen_die("no image received\n");

	int nearest_message_index = find_nearest_globalpos_message(image->timestamp);

	if (nearest_message_index < 0)
	{
		carmen_warn("nearest_message_index < 0\n");
		return;
	}

	carmen_localize_ackerman_globalpos_message globalpos_message = globalpos_message_buffer[nearest_message_index];
	carmen_pose_3D_t globalpos = globalpos_message.pose;
	globalpos.position.x = globalpos_message.globalpos.x;
	globalpos.position.y = globalpos_message.globalpos.y;
	globalpos.orientation.yaw = globalpos_message.globalpos.theta;

	double dt = image->timestamp - globalpos_message.timestamp;
	if (dt >= 0.0)
		globalpos = get_interpolated_pose_at_time(globalpos, dt, globalpos_message.v, globalpos_message.phi);
	else
		carmen_die("dt < 0\n");

	create_image_filename_from_timestamp(image->timestamp, &img_filename, camera);

	fprintf(image_pose_output_file, "%lf %lf %lf %lf %lf %lf %lf %s/%s\n",
			globalpos.position.x, globalpos.position.y, globalpos.position.z,	//tx, ty, tz
			globalpos.orientation.roll, globalpos.orientation.pitch, globalpos.orientation.yaw,		//rx, ry, rz,
			image->timestamp,
			(char *) output_dir_name, img_filename);
	fflush(image_pose_output_file);
}


void
save_pose_to_file(carmen_bumblebee_basic_stereoimage_message *stereo_image, int camera)
{
	//double roll, pitch, yaw;
	char *left_img_filename, *right_img_filename;

	if ((stereo_image == NULL) || (image_pose_output_file == NULL))
	{
		carmen_die("no image received\n");
	}
	
	int nearest_message_index = find_nearest_globalpos_message(stereo_image->timestamp);
	
	if (nearest_message_index < 0)
	{
		carmen_warn("nearest_message_index < 0\n");
		return;
	}
	
	carmen_localize_ackerman_globalpos_message globalpos_message = globalpos_message_buffer[nearest_message_index];
	carmen_pose_3D_t globalpos = globalpos_message.pose;
	globalpos.position.x = globalpos_message.globalpos.x;
	globalpos.position.y = globalpos_message.globalpos.y;
	globalpos.orientation.yaw = globalpos_message.globalpos.theta;

	// printf("pose: %i\n",nearest_message_index);
	// printf("pose: %d %d %d %d %f %f \n",
	// 	globalpos.position.x,
	// 	globalpos.position.y,
	// 	globalpos.position.z,
	// 	globalpos.orientation.yaw,
	// 	stereo_image->timestamp,
	// 	globalpos_message.timestamp
	// 	);

	// if (nearest_message_index < 0)
	// {
	// 	carmen_warn("nearest_message_index < 0\n");
	// 	return;
	// }

	double dt = stereo_image->timestamp - globalpos_message.timestamp;
	if (dt >= 0.0)
	{
		globalpos = get_interpolated_pose_at_time(globalpos, dt, globalpos_message.v, globalpos_message.phi);
	}
	else
	{
		carmen_die("dt < 0\n");
	}
	
	// tf::Pose camera_wrt_world = get_camera_pose_wrt_world(globalpos);

	// tf::Vector3 position = camera_wrt_world.getOrigin();
	// tf::Quaternion orientation = camera_wrt_world.getRotation();

	// tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
	
	create_stereo_filename_from_timestamp(stereo_image->timestamp, &left_img_filename, &right_img_filename, camera);

	fprintf(image_pose_output_file, "%lf %lf %lf %lf %lf %lf %lf %s/%s %s/%s\n",
			globalpos.position.x, globalpos.position.y, globalpos.position.z,	//tx, ty, tz
			globalpos.orientation.roll, globalpos.orientation.pitch, globalpos.orientation.yaw,		//rx, ry, rz,
			stereo_image->timestamp,
			(char*)output_dir_name, left_img_filename,
			(char*)output_dir_name, right_img_filename);
	fflush(image_pose_output_file);
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
void velodyne_partial_scan_handler( carmen_velodyne_partial_scan_message* lidar)
{
	//printf("CHEGOU VELODYNE \n");
	save_pose_to_file(lidar);
}

void
bumblebee_basic_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	//printf("CHEGOU IMAGEM BB\n");
	save_pose_to_file(stereo_image, camera);
	//use log2png.py for newer logs instead
	//save_image_to_file(stereo_image, camera);
}


void
camera_drivers_message_handler(camera_message *image)
{
	// printf("CHEGOU IMAGEM INTELBRAS\n");
	save_pose_to_file(image, camera);
}


void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *message)
{
	//printf("CHEGOU LOCALIZE ACKERMAN %lf %lf %lf\n",message->globalpos.x,message->globalpos.y,message->globalpos.theta);
	// printf("CHEGOU LOCALIZE ACKERMAN \n");
	globalpos_message_buffer[globalpos_message_index] = *message;
	globalpos_message_index = (globalpos_message_index + 1) % 100;
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("log_filter: disconnected\n");

		if (image_pose_output_file != NULL)
			fclose(image_pose_output_file);
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
read_parameters(int argc, char **argv)
{
	char bumblebee_string[256];
	char camera_string[256];

	carmen_param_t param_cmd_list[] =
	{
		{(char *) "commandline", (char *) "camera_id", CARMEN_PARAM_INT, &camera, 0, NULL},
		{(char *) "commandline", (char *) "camera_type", CARMEN_PARAM_INT, &camera_type, 0, NULL},
		{(char *) "commandline", (char *) "output_dir", CARMEN_PARAM_STRING, &output_dir_name, 0, NULL},
		{(char *) "commandline", (char *) "output_txt", CARMEN_PARAM_STRING, &image_pose_output_filename, 0, NULL},
		{(char *) "commandline", (char *) "log_filename", CARMEN_PARAM_STRING, &log_filename, 0, NULL}
	};

	carmen_param_install_params(argc, argv, param_cmd_list, sizeof(param_cmd_list)/sizeof(param_cmd_list[0]));

	sprintf(camera_string, "%s%d", "camera", camera);
	sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera);

	carmen_param_t param_list[] =
	{
		{bumblebee_string, (char *) "width", CARMEN_PARAM_INT, &bumblebee_basic_width, 0, NULL},
		{bumblebee_string, (char *) "height", CARMEN_PARAM_INT, &bumblebee_basic_height, 0, NULL},

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
		{(char *) "sensor_board_1",  (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(board_pose_g.orientation.roll), 0, NULL},
		{(char *) "robot", 	    	(char *) "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &between_axis_distance, 0, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list)/sizeof(param_list[0]));
}


void
initialize_transformations()
{
	tf::Transform world_to_carmen_pose;
	tf::Transform board_to_camera_pose;
	tf::Transform car_to_board_pose;
	tf::Transform world_to_car_pose;

	tf::Time::init();

	// initial car pose with respect to the world
	world_to_car_pose.setOrigin(tf::Vector3(car_pose_g.position.x, car_pose_g.position.y, car_pose_g.position.z));
	//world_to_car_pose.setRotation(tf::createQuaternionFromRPY(car_pose_g.orientation.roll, car_pose_g.orientation.pitch, car_pose_g.orientation.yaw));
	world_to_car_pose.setRotation(tf::Quaternion(car_pose_g.orientation.yaw, car_pose_g.orientation.pitch, car_pose_g.orientation.roll));
	tf::StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
	transformer.setTransform(world_to_car_transform, "world_to_car_transform");

	// board pose with respect to the car
	car_to_board_pose.setOrigin(tf::Vector3(board_pose_g.position.x, board_pose_g.position.y, board_pose_g.position.z));
	//car_to_board_pose.setRotation(tf::createQuaternionFromRPY(board_pose_g.orientation.roll, board_pose_g.orientation.pitch, board_pose_g.orientation.yaw));
	car_to_board_pose.setRotation(tf::Quaternion(board_pose_g.orientation.yaw, board_pose_g.orientation.pitch, board_pose_g.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform car_to_board_transform(car_to_board_pose, tf::Time(0), "/car", "/board");
	transformer.setTransform(car_to_board_transform, "car_to_board_transform");

	// camera pose with respect to the board
	board_to_camera_pose.setOrigin(tf::Vector3(camera_pose_g.position.x, camera_pose_g.position.y, camera_pose_g.position.z));
	//board_to_camera_pose.setRotation(tf::createQuaternionFromRPY(camera_pose_g.orientation.roll, camera_pose_g.orientation.pitch, camera_pose_g.orientation.yaw));
	board_to_camera_pose.setRotation(tf::Quaternion(camera_pose_g.orientation.yaw, camera_pose_g.orientation.pitch, camera_pose_g.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform board_to_camera_transform(board_to_camera_pose, tf::Time(0), "/board", "/camera");
	transformer.setTransform(board_to_camera_transform, "board_to_camera_transform");

	carmen_warn("%lf %lf %lf %lf %lf %lf\n", camera_pose_g.position.x, camera_pose_g.position.y, camera_pose_g.position.z,
			carmen_radians_to_degrees(camera_pose_g.orientation.yaw), carmen_radians_to_degrees(camera_pose_g.orientation.pitch),
			carmen_radians_to_degrees(camera_pose_g.orientation.roll));
}


void
initialize_structures()
{
	image_pose_output_file = fopen(image_pose_output_filename, "w");
	if (camera_type == 0)	// Bumblebee
		fprintf(image_pose_output_file, "x y z rx ry rz timestamp left_image right_image\n");
	else
		fprintf(image_pose_output_file, "x y z rx ry rz timestamp left_image\n"); // Intelbras ou lidar
	memset(globalpos_message_buffer, 0, 100 * sizeof(carmen_localize_ackerman_globalpos_message));
}


void
subscribe_messages()
{
	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
	//if (camera_type>=2)
	//{
		carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_handler, CARMEN_SUBSCRIBE_LATEST);
	//}
	//else
	//{
		carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) bumblebee_basic_handler, CARMEN_SUBSCRIBE_LATEST);
		camera_drivers_subscribe_message(camera, NULL, (carmen_handler_t) camera_drivers_message_handler, CARMEN_SUBSCRIBE_LATEST);
	//}
}
///////////////////////////////////////////////////////////////////////////////////////////////

int
main(int argc, char *argv[])
{
	signal(SIGINT, shutdown_module);

	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	read_parameters(argc, argv);

	initialize_transformations();

	initialize_structures();

	subscribe_messages();

	carmen_ipc_dispatch();

	return(0);
}

