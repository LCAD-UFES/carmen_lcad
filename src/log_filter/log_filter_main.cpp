
#include <stdio.h>
#include <string.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <carmen/carmen.h>
#include <carmen/gps_nmea_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/kinect_interface.h>
#include <carmen/web_cam_interface.h>
#include <carmen/stereo_interface.h>
#include <carmen/base_ackerman_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/stereo_util.h>
#include <carmen/xsens_interface.h>
#include <carmen/kinect_util.h>

#include <tf.h>

static char* output_dir_name;
static char* gps_gpgga_output_filename = (char*)"gps.txt";
static char* gps_xyz_output_filename = (char*)"gps_xyz.csv";
static char* fused_odometry_output_filename = (char*)"fused_odometry.txt";
static char* car_odometry_output_filename = (char*)"car_odometry.txt";
static char* globalpos_output_filename = (char*)"globalpos.txt";
static char* imagepos_output_filename = (char*)"imagepos.txt";
static FILE* gps_gpgga_output_file = NULL;
static FILE* gps_xyz_output_file = NULL;
static FILE* fused_odometry_output_file = NULL;
static FILE* image_pose_output_file = NULL;
static FILE* car_odometry_output_file = NULL;
static FILE* globalpos_output_file = NULL;
static long message_index = 0;

static unsigned short int m_gamma[2048];

static int gps_xyz_message_index = 0;
static carmen_gps_xyz_message gps_xyz_message_buffer[100];

static int car_odometry_message_index = 0;
static carmen_base_ackerman_odometry_message car_odometry_message_buffer[100];

static int imu_odometry_message_index = 0;
static carmen_xsens_global_quat_message imu_odometry_message_buffer[100];

static int globalpos_message_index = 0;
static carmen_localize_ackerman_globalpos_message globalpos_message_buffer[100];

static int fused_odometry_message_index = 0;
static carmen_fused_odometry_message fused_odometry_message_buffer[100];

int
find_nearest_gps_xyz_message(double timestamp)
{
	int i, nearest_index = -1;
	double shortest_interval = MAXDOUBLE;

	for(i=0; i < 100; i++)
		if (gps_xyz_message_buffer[i].host != NULL)
			if (fabs(timestamp - gps_xyz_message_buffer[i].timestamp) < shortest_interval)
			{
				shortest_interval = timestamp - gps_xyz_message_buffer[i].timestamp;
				nearest_index = i;
			}

	return nearest_index;
}


int
find_nearest_odometry_message(double timestamp)
{
	int i, nearest_index = -1;
	double shortest_interval = MAXDOUBLE;

	for(i=0; i < 100; i++)
		if (car_odometry_message_buffer[i].host != NULL)
			if (fabs(timestamp - car_odometry_message_buffer[i].timestamp) < shortest_interval)
			{
				shortest_interval = timestamp - car_odometry_message_buffer[i].timestamp;
				nearest_index = i;
			}

	return nearest_index;
}


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


int
find_nearest_xsens_message(double timestamp)
{
	int i, nearest_index = -1;
	double shortest_interval = MAXDOUBLE;

	for(i=0; i < 100; i++)
		if (imu_odometry_message_buffer[i].host != NULL)
			if (fabs(timestamp - imu_odometry_message_buffer[i].timestamp) < shortest_interval)
			{
				shortest_interval = timestamp - imu_odometry_message_buffer[i].timestamp;
				nearest_index = i;
			}

	return nearest_index;
}


int
find_nearest_fused_odometry_message(double timestamp)
{
	int i, nearest_index = -1;
	double shortest_interval = MAXDOUBLE;

	for (i = 0; i < 100; i++)
	{
		if (fused_odometry_message_buffer[i].host != NULL)
		{
			double delta_t = timestamp - fused_odometry_message_buffer[i].timestamp;
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
compose_filename_from_timestamp(double timestamp, char **filename, char *extension)
{
	*filename = (char*) malloc (256 * sizeof(char));
	sprintf((*filename), "%.25f.%s", timestamp, extension);
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
create_disparity_filename_from_timestamp(double timestamp, char **ref_img_filename, char **disp_img_filename, int camera)
{
	compose_filename_from_timestamp_bb(timestamp, disp_img_filename, (char*)"disp.png", camera);
	compose_filename_from_timestamp_bb(timestamp, ref_img_filename, (char*)"ref.png", camera);
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
create_image_from_float_buffer(float *buffer, IplImage **img, int width, int height)
{
	int i;

	(*img) = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

	for(i = 0; i < (height * width); i++)
		(*img)->imageData[i] = (int) buffer[i];
}


void
save_image_to_file(carmen_bumblebee_basic_stereoimage_message *stereo_image, int camera)
{
	char *left_img_filename, *right_img_filename;
	char *left_composed_path, *right_composed_path;
	IplImage *left_img, *right_img;

	create_stereo_filename_from_timestamp(stereo_image->timestamp, &left_img_filename, &right_img_filename, camera);
	compose_output_path(output_dir_name, left_img_filename, &left_composed_path);
	compose_output_path(output_dir_name, right_img_filename, &right_composed_path);

	create_image_from_rgb_buffer(stereo_image->raw_left, &left_img, stereo_image->width, stereo_image->height);
	create_image_from_rgb_buffer(stereo_image->raw_right, &right_img, stereo_image->width, stereo_image->height);

	cvSaveImage(left_composed_path, left_img, NULL);
	cvSaveImage(right_composed_path, right_img, NULL);

//	printf("left image saved: %s\n", left_composed_path);
//	printf("right image saved: %s\n", right_composed_path);

	free(left_img_filename);
	free(left_composed_path);
	free(right_img_filename);
	free(right_composed_path);

	cvRelease((void**) &left_img);
	cvRelease((void**) &right_img);
}


void
save_stereo_image_to_file(carmen_simple_stereo_disparity_message *stereo_message, int camera)
{
	stereo_util util = get_stereo_instance(camera, 0, 0);

	char *disparity_img_filename, *reference_img_filename;
	char *disparity_composed_path, *reference_composed_path;
	IplImage *disparity_img, *reference_img;

	create_disparity_filename_from_timestamp(stereo_message->timestamp, &reference_img_filename, &disparity_img_filename, camera);
	compose_output_path(output_dir_name, disparity_img_filename, &disparity_composed_path);
	compose_output_path(output_dir_name, reference_img_filename, &reference_composed_path);

	create_image_from_float_buffer(stereo_message->disparity, &disparity_img, util.width, util.height);
	create_image_from_rgb_buffer(stereo_message->reference_image, &reference_img, util.width, util.height);

	cvSaveImage(disparity_composed_path, disparity_img, NULL);
	cvSaveImage(reference_composed_path, reference_img, NULL);

	printf("disparity image saved: %s\n", disparity_composed_path);
	printf("reference image saved: %s\n", reference_composed_path);

	free(disparity_img_filename);
	free(disparity_composed_path);
	free(reference_img_filename);
	free(reference_composed_path);

	cvRelease((void**) &disparity_img);
	cvRelease((void**) &reference_img);
}


void
save_odom_metadata_to_file(carmen_bumblebee_basic_stereoimage_message *stereo_image, int camera)
{
	int nearest_odometry_message_index = -1;
	int nearest_xsens_message_index = -1;
	char *left_img_filename, *right_img_filename;

	if (stereo_image != NULL)
	{
		nearest_odometry_message_index = find_nearest_odometry_message(stereo_image->timestamp);

		nearest_xsens_message_index = find_nearest_xsens_message(stereo_image->timestamp);

		if (nearest_odometry_message_index >= 0 && nearest_xsens_message_index >= 0)
		{
			create_stereo_filename_from_timestamp(stereo_image->timestamp, &left_img_filename, &right_img_filename, camera);

			if (car_odometry_output_file != NULL)
			{
				fprintf(car_odometry_output_file, "%.25f;%.25f;%.25f;%.25f;%.25f;%s;%s\n",
						car_odometry_message_buffer[nearest_odometry_message_index].timestamp,
						car_odometry_message_buffer[nearest_odometry_message_index].v,
						imu_odometry_message_buffer[nearest_xsens_message_index].timestamp,
						imu_odometry_message_buffer[nearest_xsens_message_index].m_gyr.z,
						stereo_image->timestamp, left_img_filename, right_img_filename
				);
				fflush(car_odometry_output_file);
			}
		}
	}
}


void
save_globalpos_metadata_to_file(carmen_bumblebee_basic_stereoimage_message *stereo_image, int camera)
{
	int nearest_globalpos_message_index = -1;
	char *left_img_filename, *right_img_filename;

	if (stereo_image != NULL)
	{
		nearest_globalpos_message_index = find_nearest_globalpos_message(stereo_image->timestamp);
		if (nearest_globalpos_message_index >= 0)
		{
			create_stereo_filename_from_timestamp(stereo_image->timestamp, &left_img_filename, &right_img_filename, camera);

			if (globalpos_output_file != NULL)
			{
				double x = globalpos_message_buffer[nearest_globalpos_message_index].globalpos.x;
				double y = globalpos_message_buffer[nearest_globalpos_message_index].globalpos.y;
				double theta = globalpos_message_buffer[nearest_globalpos_message_index].globalpos.theta;
				double v = globalpos_message_buffer[nearest_globalpos_message_index].v;
				double delta_t = stereo_image->timestamp -
						globalpos_message_buffer[nearest_globalpos_message_index].timestamp;

				x += v * delta_t * cos(theta);
				y += v * delta_t * sin(theta);

				fprintf(globalpos_output_file, "%.3lf,%.3lf,%.3lf,%.3lf,%.25lf,%s,%s\n",
						x, y, theta, v,
						stereo_image->timestamp,
						left_img_filename, right_img_filename
				);
				fflush(globalpos_output_file);
			}
		}
	}
}


void
save_image_pose_to_file(carmen_bumblebee_basic_stereoimage_message *stereo_image, int camera)
{
	int nearest_message_index = -1;
	char *left_img_filename, *right_img_filename;
	static double global_x = 0.0, global_y = 0.0;

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
				double z = globalpos_message_buffer[nearest_message_index].pose.position.z;

				double roll = globalpos_message_buffer[nearest_message_index].pose.orientation.roll;
				double pitch = globalpos_message_buffer[nearest_message_index].pose.orientation.pitch;
				double yaw = globalpos_message_buffer[nearest_message_index].pose.orientation.yaw;

				tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);

				if (global_x == 0.0)
					global_x = x;
				if (global_y == 0.0)
					global_y = y;
				x += -global_x;
				y += -global_y;

				fprintf(image_pose_output_file, "%s/%s %.6lf %.6lf %.6lf %.6lf %.6lf %.6lf %.6lf\n",
						(char*)"images",left_img_filename,
						x, y, z,
						q.getW(), //W
						q.getX(), //P -> roll
						q.getY(), //Q -> pitch
						q.getZ() //R -> yaw
				);
				fflush(image_pose_output_file);
				double roll2, pitch2, yaw2;
				tf::Matrix3x3(q).getRPY(roll2, pitch2, yaw2);
				printf("%.6lf %.6lf %.6lf %.6lf %.6lf %.6lf (%.6lf %.6lf %.6lf)\n", x, y, z, roll, pitch, yaw, roll-roll2, pitch-pitch2, yaw-yaw2);
			}
		}
	}
}


void
save_gps_metadata_to_file(carmen_bumblebee_basic_stereoimage_message *stereo_image, int camera)
{
	int nearest_gps_xyz_message_index = -1;
//	char *left_img_filename, *right_img_filename;

	(void) camera;

	if (stereo_image != NULL)
	{
		nearest_gps_xyz_message_index = find_nearest_gps_xyz_message(stereo_image->timestamp);

		if (nearest_gps_xyz_message_index >= 0)
		{
//			create_stereo_filename_from_timestamp(stereo_image->timestamp, &left_img_filename, &right_img_filename, camera);

			if (gps_xyz_output_file != NULL)
			{
//				fprintf(gps_xyz_output_file, "%.25f;%.25f;%.25f;%.25f;%s;%s\n",
//						gps_xyz_message_buffer[nearest_gps_xyz_message_index].timestamp,
//						gps_xyz_message_buffer[nearest_gps_xyz_message_index].x,
//						gps_xyz_message_buffer[nearest_gps_xyz_message_index].y,
//						stereo_image->timestamp, left_img_filename, right_img_filename
//				);
				fprintf(gps_xyz_output_file, "%.25f,%.25f,%.25f\n",
						stereo_image->timestamp,
						gps_xyz_message_buffer[nearest_gps_xyz_message_index].x,
						gps_xyz_message_buffer[nearest_gps_xyz_message_index].y
				);
				fflush(gps_xyz_output_file);
			}
		}
	}
}


void
bumblebee_basic_handler_1(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	save_image_to_file(stereo_image, 1);
}


void
bumblebee_basic_handler_2(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	save_image_to_file(stereo_image, 2);
}


void
bumblebee_basic_handler_3(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	save_image_pose_to_file(stereo_image, 3);
	save_image_to_file(stereo_image, 3);
}


void
bumblebee_basic_handler_4(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	save_image_to_file(stereo_image, 4);
}


void
bumblebee_basic_handler_5(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	save_image_to_file(stereo_image, 5);
}


void
bumblebee_basic_handler_6(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	save_image_to_file(stereo_image, 6);
}


void
bumblebee_basic_handler_7(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	save_image_to_file(stereo_image, 7);
}


void
bumblebee_basic_handler_8(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	save_image_pose_to_file(stereo_image, 8);
	save_globalpos_metadata_to_file(stereo_image, 8);
	save_odom_metadata_to_file(stereo_image, 8);
	save_gps_metadata_to_file(stereo_image, 8);
	save_image_to_file(stereo_image, 8);
}


void
bumblebee_basic_handler_9(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	save_image_to_file(stereo_image, 9);
}


void
stereo_handler_1(carmen_simple_stereo_disparity_message *stereo_message)
{
	save_stereo_image_to_file(stereo_message, 1);
}


void
stereo_handler_2(carmen_simple_stereo_disparity_message *stereo_message)
{
	save_stereo_image_to_file(stereo_message, 2);
}


void
stereo_handler_3(carmen_simple_stereo_disparity_message *stereo_message)
{
	save_stereo_image_to_file(stereo_message, 3);
}


void
stereo_handler_4(carmen_simple_stereo_disparity_message *stereo_message)
{
	save_stereo_image_to_file(stereo_message, 4);
}


void
stereo_handler_5(carmen_simple_stereo_disparity_message *stereo_message)
{
	save_stereo_image_to_file(stereo_message, 5);
}


void
stereo_handler_6(carmen_simple_stereo_disparity_message *stereo_message)
{
	save_stereo_image_to_file(stereo_message, 6);
}


void
stereo_handler_7(carmen_simple_stereo_disparity_message *stereo_message)
{
	save_stereo_image_to_file(stereo_message, 7);
}


void
stereo_handler_8(carmen_simple_stereo_disparity_message *stereo_message)
{
	save_stereo_image_to_file(stereo_message, 8);
}


void
stereo_handler_9(carmen_simple_stereo_disparity_message *stereo_message)
{
	save_stereo_image_to_file(stereo_message, 9);
}


void
gps_gpgga_handler(carmen_gps_gpgga_message *gps_gpgga_message)
{
	printf("GPS!\n");

	if (gps_gpgga_output_file != NULL)
	{
		fprintf(gps_gpgga_output_file, "%.25f %.25f %c %c %.25f\n",
				gps_gpgga_message-> latitude,
				gps_gpgga_message-> longitude,
				gps_gpgga_message-> lat_orient,
				gps_gpgga_message-> long_orient,
				gps_gpgga_message-> timestamp
		);
	}
}

void
image_pose_handler(carmen_localize_ackerman_globalpos_message *message)
{
	globalpos_message_buffer[globalpos_message_index] = *message;
	globalpos_message_index = (globalpos_message_index + 1) % 100;
}


void
fused_odometry_handler(carmen_fused_odometry_message *fused_odometry_message)
{
	fused_odometry_message_buffer[fused_odometry_message_index] = *fused_odometry_message;
	fused_odometry_message_index = (fused_odometry_message_index + 1) % 100;
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
		if (gps_xyz_output_file != NULL)
			fclose(gps_xyz_output_file);
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
create_kinect_depth_img_filename_from_timestamp(double timestamp, char **depth_img_filename)
{
	compose_filename_from_timestamp(timestamp, depth_img_filename, (char*)".depth.meters.png");
}


void
create_kinect_rgb_depth_img_filename_from_timestamp(double timestamp, char **depth_img_filename)
{
	compose_filename_from_timestamp(timestamp, depth_img_filename, (char*)".depth.rgb.png");
}


void
create_kinect_video_img_filename_from_timestamp(double timestamp, char **video_img_filename)
{
	compose_filename_from_timestamp(timestamp, video_img_filename, (char*)".video.png");
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
convert_kinect_meters_to_rgb (float meters, uchar *r, uchar *g, uchar *b, unsigned short int *m_gamma)
{
	int depth = convert_kinect_depth_meters_to_raw(meters);

	int pval = m_gamma[depth];
	int lb = pval & 0xff;

	switch (pval >> 8)
	{
	case 0:
		*r = 255;
		*g= 255-lb;
		*b = 255-lb;
		break;
	case 1:
		*r = 255;
		*g= lb;
		*b = 0;
		break;
	case 2:
		*r = 255-lb;
		*g= 255;
		*b = 0;
		break;
	case 3:
		*r = 0;
		*g= 255;
		*b = lb;
		break;
	case 4:
		*r = 0;
		*g= 255-lb;
		*b = 255;
		break;
	case 5:
		*r = 0;
		*g= 0;
		*b = 255-lb;
		break;
	default:
		*r = 0;
		*g= 0;
		*b = 0;
		break;
	}
}


void
create_rgb_image_from_depth_buffer (float *depth_buffer, IplImage **img, int height, int width)
{
	int i;
	uchar r, g, b;

	(*img) = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

	// convert depth image to rgb
	for (i = 0; i < width * height; i++)
	{
		convert_kinect_meters_to_rgb (depth_buffer[i], &r, &g, &b, m_gamma);

		(*img)->imageData[3 * i + 0] = b;
		(*img)->imageData[3 * i + 1] = g;
		(*img)->imageData[3 * i + 2] = r;
	}
}


void
kinect_depth_handler (carmen_kinect_depth_message *kinect_depth_message)
{
	char *depth_img_filename, *depth_rgb_filename;
	char *depth_img_composed_path, *depth_rgb_composed_path;
	IplImage *depth_img, *depth_rgb_img;

	create_kinect_depth_img_filename_from_timestamp(message_index, &depth_img_filename);
	create_kinect_rgb_depth_img_filename_from_timestamp(message_index, &depth_rgb_filename);

	compose_output_path(output_dir_name, depth_img_filename, &depth_img_composed_path);
	compose_output_path(output_dir_name, depth_rgb_filename, &depth_rgb_composed_path);

	create_image_from_depth_buffer(kinect_depth_message->depth, &depth_img, kinect_depth_message->size, kinect_depth_message->height, kinect_depth_message->width);
	create_rgb_image_from_depth_buffer(kinect_depth_message->depth, &depth_rgb_img, kinect_depth_message->height, kinect_depth_message->width);

	cvSaveImage(depth_img_composed_path, depth_img, NULL);
	cvSaveImage(depth_rgb_composed_path, depth_rgb_img, NULL);

	printf("depth image saved: %s\n", depth_img_composed_path);

	free(depth_img_filename);
	free(depth_rgb_filename);
	free(depth_img_composed_path);
	free(depth_rgb_composed_path);

	cvRelease((void**) &depth_img);
	cvRelease((void**) &depth_rgb_img);

	message_index++;
}


void
kinect_video_handler (carmen_kinect_video_message *kinect_video_message)
{
	char *video_img_filename;
	char *video_img_composed_path;
	IplImage *video_img;

	create_kinect_video_img_filename_from_timestamp(message_index, &video_img_filename);
	compose_output_path(output_dir_name, video_img_filename, &video_img_composed_path);

	create_image_from_rgb_buffer(kinect_video_message->video, &video_img, kinect_video_message->width, kinect_video_message->height);

	cvSaveImage(video_img_composed_path, video_img, NULL);

	printf("video image saved: %s\n", video_img_composed_path);

	free(video_img_filename);
	free(video_img_composed_path);

	cvRelease((void**) &video_img);
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
carmen_web_cam_message_handler (carmen_web_cam_message *message)
{
	char *filename, *composed_path;

	IplImage *img = cvCreateImageHeader(cvSize(message->width, message->height), IPL_DEPTH_8U, 3);
	img->imageData = message->img_data;

	compose_filename_from_timestamp(message->timestamp, &filename, (char*)"bmp");
	compose_output_path(output_dir_name, filename, &composed_path);

	printf("web_cam image: %s\n", composed_path);

	cvSaveImage(composed_path, img, NULL);
	cvReleaseImageHeader(&img);

	free(filename);
	free(composed_path);
}


static void
gps_xyz_message_handler(carmen_gps_xyz_message *gps_xyz_message)
{
	gps_xyz_message_buffer[(gps_xyz_message_index + 1) % 100] = *gps_xyz_message;
}


static void
car_odometry_message_handler(carmen_base_ackerman_odometry_message *car_odometry_message)
{
	car_odometry_message_buffer[(car_odometry_message_index + 1) % 100] = *car_odometry_message;
}


static void
xsens_mti_message_handler(carmen_xsens_global_quat_message *xsens_xyz)
{
	imu_odometry_message_buffer[(imu_odometry_message_index + 1) % 100] = *xsens_xyz;
}


static void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *globalpos)
{
	globalpos_message_buffer[globalpos_message_index] = *globalpos;
	globalpos_message_index = (globalpos_message_index + 1) % 100;
}


void
initialize_module_args(int argc, char **argv)
{
	// verifica se os parametros
	// foram passados corretamente
	if (argc < 2)
	{
		printf("Use: \n %s see code\n\n", argv[0]);
		printf("\nOBS: output_path must be used if sensor is bumblebee or kinect.\n\n");
		exit(-1);
	}

	char *sensor = argv[1];

	// se o sensor escolhido foi a bumblebee
	if (!strcmp(sensor, "bumblebee"))
	{
		if (argc < 3)
		{
			printf("\nError: You must set the output path for images!\n\n");
			exit(-1);
		}

		output_dir_name = argv[2];

		if (argc == 4)
		{
			if (!strcmp(argv[3], "with_gps"))
			{
				carmen_gps_xyz_subscribe_message(NULL, (carmen_handler_t) gps_xyz_message_handler, CARMEN_SUBSCRIBE_LATEST);
				gps_xyz_output_file = fopen(gps_xyz_output_filename, "w");
				fprintf(gps_xyz_output_file, "timestamp,x,y\n");
			}
			else if (!strcmp(argv[3], "with_odometry"))
			{
				carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) car_odometry_message_handler, CARMEN_SUBSCRIBE_LATEST);
				carmen_xsens_subscribe_xsens_global_quat_message(NULL, (carmen_handler_t) xsens_mti_message_handler, CARMEN_SUBSCRIBE_LATEST);
				car_odometry_output_file = fopen(car_odometry_output_filename, "w");
			}
			else if (!strcmp(argv[3], "with_globalpos"))
			{
				carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
				globalpos_output_file = fopen(globalpos_output_filename, "w");
				fprintf(globalpos_output_file, "x, y, theta, v, timestamp, left_img, right_img\n");
			}
			else if (!strcmp(argv[3], "with_image_pose"))
			{
				carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) image_pose_handler, CARMEN_SUBSCRIBE_LATEST);
				image_pose_output_file = fopen(imagepos_output_filename, "w");
				fprintf(image_pose_output_file, "image x y z w p q r\n");
			}
		}

		carmen_bumblebee_basic_subscribe_stereoimage(1, NULL, (carmen_handler_t) bumblebee_basic_handler_1, CARMEN_SUBSCRIBE_LATEST);
		carmen_bumblebee_basic_subscribe_stereoimage(2, NULL, (carmen_handler_t) bumblebee_basic_handler_2, CARMEN_SUBSCRIBE_LATEST);
		carmen_bumblebee_basic_subscribe_stereoimage(3, NULL, (carmen_handler_t) bumblebee_basic_handler_3, CARMEN_SUBSCRIBE_LATEST);
		carmen_bumblebee_basic_subscribe_stereoimage(4, NULL, (carmen_handler_t) bumblebee_basic_handler_4, CARMEN_SUBSCRIBE_LATEST);
		carmen_bumblebee_basic_subscribe_stereoimage(5, NULL, (carmen_handler_t) bumblebee_basic_handler_5, CARMEN_SUBSCRIBE_LATEST);
		carmen_bumblebee_basic_subscribe_stereoimage(6, NULL, (carmen_handler_t) bumblebee_basic_handler_6, CARMEN_SUBSCRIBE_LATEST);
		carmen_bumblebee_basic_subscribe_stereoimage(7, NULL, (carmen_handler_t) bumblebee_basic_handler_7, CARMEN_SUBSCRIBE_LATEST);
		carmen_bumblebee_basic_subscribe_stereoimage(8, NULL, (carmen_handler_t) bumblebee_basic_handler_8, CARMEN_SUBSCRIBE_LATEST);
		carmen_bumblebee_basic_subscribe_stereoimage(9, NULL, (carmen_handler_t) bumblebee_basic_handler_9, CARMEN_SUBSCRIBE_LATEST);

		return;
	}

	// se o sensor escolhido foi o gps
	else if (!strcmp(sensor, "gps"))
	{
		carmen_gps_subscribe_nmea_message(NULL, (carmen_handler_t) gps_gpgga_handler, CARMEN_SUBSCRIBE_LATEST);
		gps_gpgga_output_file = fopen(gps_gpgga_output_filename, "w");

		return;
	}

	// se o sensor escolhido foi o fused_odometry
	else if (!strcmp(sensor, "fused_odometry"))
	{
		carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t) fused_odometry_handler, CARMEN_SUBSCRIBE_LATEST);
		fused_odometry_output_file = fopen(fused_odometry_output_filename, "w");

		return;
	}

	else if (!strcmp(sensor, "kinect"))
	{
		if (argc < 3)
		{
			printf("\nError: You must set kinect output_path!\n\n");
			exit(-1);
		}

		output_dir_name = argv[2];

		initialize_m_gamma ();
		carmen_kinect_subscribe_depth_message(0, NULL, (carmen_handler_t) kinect_depth_handler, CARMEN_SUBSCRIBE_LATEST);
		carmen_kinect_subscribe_video_message(0, NULL, (carmen_handler_t) kinect_video_handler, CARMEN_SUBSCRIBE_LATEST);

		return;
	}

	else if (!strcmp(sensor, "web_cam"))
	{
		if (argc < 3)
		{
			printf("\nError: You must set web_cam output_path!\n\n");
			exit(-1);
		}

		output_dir_name = argv[2];

		carmen_web_cam_subscribe_message (NULL, (carmen_handler_t) carmen_web_cam_message_handler, CARMEN_SUBSCRIBE_LATEST);

		return;
	}

	else if (!strcmp(sensor, "stereo"))
	{
		if (argc < 3)
		{
			printf("\nError: You must set images output_path!\n\n");
			exit(-1);
		}

		output_dir_name = argv[2];

		carmen_stereo_subscribe(1, NULL, (carmen_handler_t) stereo_handler_1, CARMEN_SUBSCRIBE_LATEST);
		carmen_stereo_subscribe(2, NULL, (carmen_handler_t) stereo_handler_2, CARMEN_SUBSCRIBE_LATEST);
		carmen_stereo_subscribe(3, NULL, (carmen_handler_t) stereo_handler_3, CARMEN_SUBSCRIBE_LATEST);
		carmen_stereo_subscribe(4, NULL, (carmen_handler_t) stereo_handler_4, CARMEN_SUBSCRIBE_LATEST);
		carmen_stereo_subscribe(5, NULL, (carmen_handler_t) stereo_handler_5, CARMEN_SUBSCRIBE_LATEST);
		carmen_stereo_subscribe(6, NULL, (carmen_handler_t) stereo_handler_6, CARMEN_SUBSCRIBE_LATEST);
		carmen_stereo_subscribe(7, NULL, (carmen_handler_t) stereo_handler_7, CARMEN_SUBSCRIBE_LATEST);
		carmen_stereo_subscribe(8, NULL, (carmen_handler_t) stereo_handler_8, CARMEN_SUBSCRIBE_LATEST);
		carmen_stereo_subscribe(9, NULL, (carmen_handler_t) stereo_handler_9, CARMEN_SUBSCRIBE_LATEST);

		return;
	}

}


int
main(int argc, char *argv[])
{
	signal(SIGINT, shutdown_module);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	carmen_kinect_define_kinect_messages(0);
	initialize_module_args(argc, argv);

	memset(gps_xyz_message_buffer, 0, 100 * sizeof(carmen_gps_xyz_message));
	memset(car_odometry_message_buffer, 0, 100 * sizeof(carmen_base_ackerman_odometry_message));
	memset(imu_odometry_message_buffer, 0, 100 * sizeof(carmen_xsens_global_quat_message));
	memset(globalpos_message_buffer, 0, 100 * sizeof(carmen_localize_ackerman_globalpos_message));

	carmen_ipc_dispatch();

	return(0);
}

