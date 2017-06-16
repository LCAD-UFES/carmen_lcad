
/**
 * @description
 * This code have been created to filter some messages of the log
 * The messages of the chosen sensor are saved to a file
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

#include <tf.h>

tf::Transformer transformer;

static int camera;
static int bumblebee_basic_width;
static int bumblebee_basic_height;
static carmen_pose_3D_t car_pose_g;
static carmen_pose_3D_t camera_pose_g;
static carmen_pose_3D_t board_pose_g;

static char* output_dir_name;
static char* image_pose_output_filename = (char*)"image_pose.txt";
static FILE* gps_gpgga_output_file = NULL;
static FILE* fused_odometry_output_file = NULL;
static FILE* image_pose_output_file = NULL;
static FILE* car_odometry_output_file = NULL;
static FILE* globalpos_output_file = NULL;

static unsigned short int m_gamma[2048];

static int globalpos_message_index = 0;
static carmen_localize_ackerman_globalpos_message globalpos_message_buffer[100];


int
find_nearest_globalpos_message(double timestamp)
{
	int i, nearest_index = -1;
	double shortest_interval = MAXDOUBLE;

	for (i = 0; i < 100; i++)
	{
		if (globalpos_message_buffer[i].host != NULL)
		{
			double delta_t = timestamp - globalpos_message_buffer[i].timestamp;
			if ((delta_t > 0.0) && (delta_t < shortest_interval))
			{
				shortest_interval = delta_t;
				nearest_index = i;
			}
		}
	}
	return nearest_index;
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
	sprintf((*filename), "%.25f.bb%02d.%s", timestamp, camera, extension);
}


void
create_stereo_filename_from_timestamp(double timestamp, char **left_img_filename, char **right_img_filename, int camera)
{
	compose_filename_from_timestamp_bb(timestamp, left_img_filename, (char*)"l.png", camera);
	compose_filename_from_timestamp_bb(timestamp, right_img_filename, (char*)"r.png", camera);
}


void
create_image_from_rgb_buffer (unsigned char *rgb_buffer, IplImage **img, int width, int height)
{
	int i;

	(*img) = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

	for(i = 0; i < (height * width); i++)
	{
		/**
		 * A imagem da bumblebee usa o formato rgb-rgb-rgb, enquanto
		 * a imagem da opencv usa o formato bgr-bgr-bgr. As linhas
		 * abaixo fazem essa conversao.
		 */
		(*img)->imageData[3 * i] = rgb_buffer[3 * i + 2];
		(*img)->imageData[3 * i + 1] = rgb_buffer[3 * i + 1];
		(*img)->imageData[3 * i + 2] = rgb_buffer[3 * i];
	}
}


void
save_image_to_file(carmen_bumblebee_basic_stereoimage_message *stereo_image, int camera)
{
	char *left_img_filename, *right_img_filename;
	char *left_composed_path, *right_composed_path;
	IplImage *left_img, *right_img;
	cv::Rect roi;
	roi.x = 0;
	roi.y = 0;
	roi.width = 640;
	roi.height = 364;

	create_stereo_filename_from_timestamp(stereo_image->timestamp, &left_img_filename, &right_img_filename, camera);
	compose_output_path(output_dir_name, left_img_filename, &left_composed_path);
	compose_output_path(output_dir_name, right_img_filename, &right_composed_path);

	create_image_from_rgb_buffer(stereo_image->raw_left, &left_img, stereo_image->width, stereo_image->height);
	create_image_from_rgb_buffer(stereo_image->raw_right, &right_img, stereo_image->width, stereo_image->height);

	if (stereo_image->width == 1280)
	{
		IplImage *img = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);

		cvResize(left_img, img, CV_INTER_AREA);
		cvRelease((void**) &left_img);
		left_img = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
		cvCopy(img, left_img);

		cvResize(right_img, img, CV_INTER_AREA);
		cvRelease((void**) &right_img);
		right_img = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
		cvCopy(img, right_img);

		cvRelease((void**) &img);
	}
	cvSetImageROI(left_img, roi);
	cvSetImageROI(right_img, roi);
	cvSaveImage(left_composed_path, left_img, NULL);
	cvSaveImage(right_composed_path, right_img, NULL);

	printf("left image saved: %s\n", left_composed_path);
//	printf("right image saved: %s\n", right_composed_path);

	free(left_img_filename);
	free(left_composed_path);
	free(right_img_filename);
	free(right_composed_path);

	cvRelease((void**) &left_img);
	cvRelease((void**) &right_img);
}


tf::StampedTransform
get_transforms_from_camera_to_world(double car_x, double car_y, double car_theta)
{
	tf::StampedTransform camera_to_world_transform;
	tf::Transform car_to_world_transform;

	car_to_world_transform.setOrigin(tf::Vector3(car_x, car_y, 0.0));
	car_to_world_transform.setRotation(tf::Quaternion(car_theta, 0.0, 0.0));

	tf::StampedTransform car_to_world_stamped_transform(car_to_world_transform, tf::Time(0), "/world", "/car");
	transformer.setTransform(car_to_world_stamped_transform, "car_to_world_stamped_transform");
	transformer.lookupTransform("/world", "/camera", tf::Time(0), camera_to_world_transform);

	return camera_to_world_transform;
}


void
save_pose_to_file(carmen_bumblebee_basic_stereoimage_message *stereo_image, int camera)
{
	int nearest_message_index = -1;
	char *left_img_filename, *right_img_filename;
	tf::StampedTransform camera_to_world_transform;

	if (stereo_image != NULL)
	{
		nearest_message_index = find_nearest_globalpos_message(stereo_image->timestamp);
		if (nearest_message_index >= 0)
		{
			create_stereo_filename_from_timestamp(stereo_image->timestamp, &left_img_filename, &right_img_filename, camera);

			if (image_pose_output_file != NULL)
			{
				double x = globalpos_message_buffer[nearest_message_index].pose.position.x;
				double y = globalpos_message_buffer[nearest_message_index].pose.position.y;
				//double z = globalpos_message_buffer[nearest_message_index].pose.position.z;

				double roll = globalpos_message_buffer[nearest_message_index].pose.orientation.roll;
				double pitch = globalpos_message_buffer[nearest_message_index].pose.orientation.pitch;
				double yaw = globalpos_message_buffer[nearest_message_index].pose.orientation.yaw;

				double v = globalpos_message_buffer[nearest_message_index].v;
				double delta_t = stereo_image->timestamp - globalpos_message_buffer[nearest_message_index].timestamp;

				x += v * delta_t * cos(yaw);
				y += v * delta_t * sin(yaw);

				tf::StampedTransform camera_to_world = get_transforms_from_camera_to_world(x, y, yaw);

				tf::Vector3 position = camera_to_world.getOrigin();
				tf::Quaternion orientation = camera_to_world.getRotation();

				tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

				fprintf(image_pose_output_file, "%s/%s %.25f %.25f %.25f %.25f %.25f %.25f %.25f %.25f %.25f %.25f %.25f\n",
						(char*)output_dir_name, left_img_filename,
						position.getX(), position.getY(), position.getZ(),
						orientation.getW(), orientation.getX(), orientation.getY(), orientation.getZ(),
						roll, pitch, yaw, stereo_image->timestamp);
				fflush(image_pose_output_file);

				//printf("%.6lf %.6lf %.6lf %.6lf %.6lf %.6lf\n", x, y, z, roll, pitch, yaw);
			}
		}
	}
}


void
bumblebee_basic_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	save_pose_to_file(stereo_image, camera);
	save_image_to_file(stereo_image, camera);
}


void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *message)
{
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

		if (gps_gpgga_output_file != NULL)
			fclose(gps_gpgga_output_file);
		if (fused_odometry_output_file != NULL)
			fclose(fused_odometry_output_file);
		if (image_pose_output_file != NULL)
			fclose(image_pose_output_file);
		if (car_odometry_output_file != NULL)
			fclose(car_odometry_output_file);
		if (globalpos_output_file != NULL)
			fclose(globalpos_output_file);

		exit(0);
	}
}



void
create_image_from_depth_buffer (float *depth_buffer, IplImage **img, int size, int height, int width)
{
	int i;

	(*img) = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);

	for(i = 0; i < size; i++)
	{
		(*img)->imageData[i] = depth_buffer[i];
	}
}



void
initialize_m_gamma ()
{
	int i;
	// initialize m_gamma
	for(i = 0 ; i < 2048 ; i++)
	{
		float v = i / 2048.0;
		v = pow(v, 3)* 6;
		m_gamma[i] = v * 6 * 256;
	}

}


void
read_camera_parameters(int argc, char **argv)
{
	int num_items;
	char bumblebee_string[256];
	char camera_string[256];

	if (argc < 3)
	{
		printf("\nUsage: %s <camera_id> <image_dir> with_image_pose\n", argv[0]);
		exit(-1);
	}

	camera = atoi(argv[1]);

	output_dir_name = argv[2];

	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_ALL);
	image_pose_output_file = fopen(image_pose_output_filename, "w");
	fprintf(image_pose_output_file, "image x y z w p q r roll pitch yaw timestamp\n");

	carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) bumblebee_basic_handler, CARMEN_SUBSCRIBE_ALL);

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
		{(char *) "sensor_board_1",  (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(board_pose_g.orientation.roll), 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

}

void initialize_transformations()
{
	tf::Transform board_to_camera_pose;
	tf::Transform car_to_board_pose;
	tf::Transform world_to_car_pose;

	tf::Time::init();

	// initial car pose with respect to the world
	world_to_car_pose.setOrigin(tf::Vector3(car_pose_g.position.x, car_pose_g.position.y, car_pose_g.position.z));
	world_to_car_pose.setRotation(tf::Quaternion(car_pose_g.orientation.yaw, car_pose_g.orientation.pitch, car_pose_g.orientation.roll));
	tf::StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
	transformer.setTransform(world_to_car_transform, "world_to_car_transform");

	// board pose with respect to the car
	car_to_board_pose.setOrigin(tf::Vector3(board_pose_g.position.x, board_pose_g.position.y, board_pose_g.position.z));
	car_to_board_pose.setRotation(tf::Quaternion(board_pose_g.orientation.yaw, board_pose_g.orientation.pitch, board_pose_g.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform car_to_board_transform(car_to_board_pose, tf::Time(0), "/car", "/board");
	transformer.setTransform(car_to_board_transform, "car_to_board_transform");

	// camera pose with respect to the board
	board_to_camera_pose.setOrigin(tf::Vector3(camera_pose_g.position.x, camera_pose_g.position.y, camera_pose_g.position.z));
	board_to_camera_pose.setRotation(tf::Quaternion(camera_pose_g.orientation.yaw, camera_pose_g.orientation.pitch, camera_pose_g.orientation.roll)); 				// yaw, pitch, roll
	tf::StampedTransform board_to_camera_transform(board_to_camera_pose, tf::Time(0), "/board", "/camera");
	transformer.setTransform(board_to_camera_transform, "board_to_camera_transform");
}


int
main(int argc, char *argv[])
{
	signal(SIGINT, shutdown_module);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_camera_parameters(argc, argv);

	initialize_transformations();

	memset(globalpos_message_buffer, 0, 100 * sizeof(carmen_localize_ackerman_globalpos_message));

	carmen_ipc_dispatch();

	return(0);
}

