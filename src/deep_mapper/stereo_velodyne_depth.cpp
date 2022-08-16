#include <carmen/carmen.h>
#include <carmen/global.h>
#include <carmen/libadabins.h>
#include <carmen/libdpt.h>
#include <carmen/libglpdepth.h>
#include <string.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <carmen/camera_drivers_messages.h>
#include <carmen/camera_drivers_interface.h>
#include <carmen/param_interface.h>
#include <carmen/param_messages.h>

#include <carmen/velodyne_camera_calibration.h>
#include <fstream>
#include <tf.h>

/*Message stereo*/
#include <carmen/stereo_interface.h>
#include <carmen/stereo_util.h>
#include <carmen/stereo_point_cloud_interface.h>

using namespace std;
using namespace cv;

static int camera;
static int vertical_resolution;
static int horizontal_resolution;
static double range_max;
static double horizontal_camera_angle;
static double horizontal_start_angle;
static double vertical_camera_angle;
static double range_multiplier_factor;
static int vertical_top_cut;
static int vertical_down_cut;
static int lidar_num;

static int camera_width = 0;
static int camera_height = 0;

static carmen_velodyne_shot *scan;
static carmen_velodyne_variable_scan_message velodyne_partial_scan;

static IplImage *reference_image;
static IplImage *reference_image_gray;

/*Message stereo*/
// static carmen_stereo_point_cloud_message point_cloud_message;
// static stereo_util su;

Mat cameraMatrix, distCoeffs, newcameramtx, R1;
double fx, fy, cu, camera_cv, k1, k2, p1, p2, k3;
static Mat MapX, MapY;

static char **modules;
static int num_modules;

char *neural_network = (char *)"glpdepth";
unsigned char *depth_pred;
const static double sorted_vertical_angles[32] =
	{
		-30.67, -29.33, -28.0, -26.67, -25.33, -24.0, -22.67, -21.33, -20.0,
		-18.67, -17.33, -16.0, -14.67, -13.33, -12.0, -10.67, -9.3299999, -8.0,
		-6.6700001, -5.3299999, -4.0, -2.6700001, -1.33, 0.0, 1.33, 2.6700001, 4.0,
		5.3299999, 6.6700001, 8.0, 9.3299999, 10.67};
const static double sorted_lidar8_vertical_angles[480] = {-25.5000, -25.3938, -25.2875, -25.1813, -25.0750, -24.9688, -24.8625, -24.7563, -24.6500, -24.5438, -24.4375, -24.3313, -24.2250, -24.1188, -24.0125, -23.9063, -23.8000, -23.6938, -23.5875, -23.4813, -23.3750, -23.2688, -23.1625, -23.0563, -22.9500, -22.8438, -22.7375, -22.6313, -22.5250, -22.4188, -22.3125, -22.2063, -22.1000, -21.9938, -21.8875, -21.7813, -21.6750, -21.5688, -21.4625, -21.3563, -21.2500, -21.1438, -21.0375, -20.9313, -20.8250, -20.7188, -20.6125, -20.5063, -20.4000, -20.2938, -20.1875, -20.0813, -19.9750, -19.8688, -19.7625, -19.6563, -19.5500, -19.4438, -19.3375, -19.2313, -19.1250, -19.0188, -18.9125, -18.8063, -18.7000, -18.5938, -18.4875, -18.3813, -18.2750, -18.1688, -18.0625, -17.9563, -17.8500, -17.7438, -17.6375, -17.5313, -17.4250, -17.3188, -17.2125, -17.1063, -17.0000, -16.8938, -16.7875, -16.6813, -16.5750, -16.4688, -16.3625, -16.2563, -16.1500, -16.0438, -15.9375, -15.8313, -15.7250, -15.6188, -15.5125, -15.4063, -15.3000, -15.1938, -15.0875, -14.9813, -14.8750, -14.7688, -14.6625, -14.5563, -14.4500, -14.3438, -14.2375, -14.1313, -14.0250, -13.9188, -13.8125, -13.7063, -13.6000, -13.4938, -13.3875, -13.2813, -13.1750, -13.0688, -12.9625, -12.8563, -12.7500, -12.6438, -12.5375, -12.4313, -12.3250, -12.2188, -12.1125, -12.0063, -11.9000, -11.7938, -11.6875, -11.5813, -11.4750, -11.3688, -11.2625, -11.1563, -11.0500, -10.9438, -10.8375, -10.7313, -10.6250, -10.5188, -10.4125, -10.3063, -10.2000, -10.0938, -9.9875, -9.8813, -9.7750, -9.6688, -9.5625, -9.4563, -9.3500, -9.2438, -9.1375, -9.0313, -8.9250, -8.8188, -8.7125, -8.6063, -8.5000, -8.3938, -8.2875, -8.1813, -8.0750, -7.9688, -7.8625, -7.7563, -7.6500, -7.5438, -7.4375, -7.3313, -7.2250, -7.1188, -7.0125, -6.9063, -6.8000, -6.6938, -6.5875, -6.4813, -6.3750, -6.2688, -6.1625, -6.0563, -5.9500, -5.8438, -5.7375, -5.6313, -5.5250, -5.4188, -5.3125, -5.2063, -5.1000, -4.9938, -4.8875, -4.7813, -4.6750, -4.5688, -4.4625, -4.3563, -4.2500, -4.1438, -4.0375, -3.9313, -3.8250, -3.7188, -3.6125, -3.5063, -3.4000, -3.2938, -3.1875, -3.0813, -2.9750, -2.8688, -2.7625, -2.6563, -2.5500, -2.4438, -2.3375, -2.2313, -2.1250, -2.0188, -1.9125, -1.8063, -1.7000, -1.5938, -1.4875, -1.3813, -1.2750, -1.1688, -1.0625, -0.9563, -0.8500, -0.7438, -0.6375, -0.5313, -0.4250, -0.3188, -0.2125, -0.1063, -0.0000, 0.1062, 0.2125, 0.3187, 0.4250, 0.5312, 0.6375, 0.7437, 0.8500, 0.9562, 1.0625, 1.1687, 1.2750, 1.3812, 1.4875, 1.5937, 1.7000, 1.8062, 1.9125, 2.0187, 2.1250, 2.2312, 2.3375, 2.4437, 2.5500, 2.6562, 2.7625, 2.8687, 2.9750, 3.0812, 3.1875, 3.2937, 3.4000, 3.5062, 3.6125, 3.7187, 3.8250, 3.9312, 4.0375, 4.1437, 4.2500, 4.3562, 4.4625, 4.5687, 4.6750, 4.7812, 4.8875, 4.9937, 5.1000, 5.2062, 5.3125, 5.4187, 5.5250, 5.6312, 5.7375, 5.8437, 5.9500, 6.0562, 6.1625, 6.2687, 6.3750, 6.4812, 6.5875, 6.6937, 6.8000, 6.9062, 7.0125, 7.1187, 7.2250, 7.3312, 7.4375, 7.5437, 7.6500, 7.7562, 7.8625, 7.9687, 8.0750, 8.1812, 8.2875, 8.3937, 8.5000, 8.6062, 8.7125, 8.8187, 8.9250, 9.0312, 9.1375, 9.2437, 9.3500, 9.4562, 9.5625, 9.6687, 9.7750, 9.8812, 9.9875, 10.0937, 10.2000, 10.3062, 10.4125, 10.5187, 10.6250, 10.7312, 10.8375, 10.9437, 11.0500, 11.1562, 11.2625, 11.3687, 11.4750, 11.5812, 11.6875, 11.7937, 11.9000, 12.0062, 12.1125, 12.2187, 12.3250, 12.4312, 12.5375, 12.6437, 12.7500, 12.8562, 12.9625, 13.0687, 13.1750, 13.2812, 13.3875, 13.4937, 13.6000, 13.7062, 13.8125, 13.9187, 14.0250, 14.1312, 14.2375, 14.3437, 14.4500, 14.5562, 14.6625, 14.7687, 14.8750, 14.9812, 15.0875, 15.1937, 15.3000, 15.4062, 15.5125, 15.6187, 15.7250, 15.8312, 15.9375, 16.0437, 16.1500, 16.2562, 16.3625, 16.4687, 16.5750, 16.6812, 16.7875, 16.8937, 17.0000, 17.1062, 17.2125, 17.3187, 17.4250, 17.5312, 17.6375, 17.7437, 17.8500, 17.9562, 18.0625, 18.1687, 18.2750, 18.3812, 18.4875, 18.5937, 18.7000, 18.8062, 18.9125, 19.0187, 19.1250, 19.2312, 19.3375, 19.4437, 19.5500, 19.6562, 19.7625, 19.8687, 19.9750, 20.0812, 20.1875, 20.2937, 20.4000, 20.5062, 20.6125, 20.7187, 20.8250, 20.9312, 21.0375, 21.1437, 21.2500, 21.3562, 21.4625, 21.5687, 21.6750, 21.7812, 21.8875, 21.9937, 22.1000, 22.2062, 22.3125, 22.4187, 22.5250, 22.6312, 22.7375, 22.8437, 22.9500, 23.0562, 23.1625, 23.2687, 23.3750, 23.4812, 23.5875, 23.6937, 23.8000, 23.9062, 24.0125, 24.1187, 24.2250, 24.3312, 24.4375, 24.5437, 24.6500, 24.7562, 24.8625, 24.9687, 25.0750, 25.1812, 25.2875, 25.3937};
const static double sorted_lidar0_vertical_angles[480] = {-25.5000, -25.3938, -25.2875, -25.1813, -25.0750, -24.9688, -24.8625, -24.7563, -24.6500, -24.5438, -24.4375, -24.3313, -24.2250, -24.1188, -24.0125, -23.9063, -23.8000, -23.6938, -23.5875, -23.4813, -23.3750, -23.2688, -23.1625, -23.0563, -22.9500, -22.8438, -22.7375, -22.6313, -22.5250, -22.4188, -22.3125, -22.2063, -22.1000, -21.9938, -21.8875, -21.7813, -21.6750, -21.5688, -21.4625, -21.3563, -21.2500, -21.1438, -21.0375, -20.9313, -20.8250, -20.7188, -20.6125, -20.5063, -20.4000, -20.2938, -20.1875, -20.0813, -19.9750, -19.8688, -19.7625, -19.6563, -19.5500, -19.4438, -19.3375, -19.2313, -19.1250, -19.0188, -18.9125, -18.8063, -18.7000, -18.5938, -18.4875, -18.3813, -18.2750, -18.1688, -18.0625, -17.9563, -17.8500, -17.7438, -17.6375, -17.5313, -17.4250, -17.3188, -17.2125, -17.1063, -17.0000, -16.8938, -16.7875, -16.6813, -16.5750, -16.4688, -16.3625, -16.2563, -16.1500, -16.0438, -15.9375, -15.8313, -15.7250, -15.6188, -15.5125, -15.4063, -15.3000, -15.1938, -15.0875, -14.9813, -14.8750, -14.7688, -14.6625, -14.5563, -14.4500, -14.3438, -14.2375, -14.1313, -14.0250, -13.9188, -13.8125, -13.7063, -13.6000, -13.4938, -13.3875, -13.2813, -13.1750, -13.0688, -12.9625, -12.8563, -12.7500, -12.6438, -12.5375, -12.4313, -12.3250, -12.2188, -12.1125, -12.0063, -11.9000, -11.7938, -11.6875, -11.5813, -11.4750, -11.3688, -11.2625, -11.1563, -11.0500, -10.9438, -10.8375, -10.7313, -10.6250, -10.5188, -10.4125, -10.3063, -10.2000, -10.0938, -9.9875, -9.8813, -9.7750, -9.6688, -9.5625, -9.4563, -9.3500, -9.2438, -9.1375, -9.0313, -8.9250, -8.8188, -8.7125, -8.6063, -8.5000, -8.3938, -8.2875, -8.1813, -8.0750, -7.9688, -7.8625, -7.7563, -7.6500, -7.5438, -7.4375, -7.3313, -7.2250, -7.1188, -7.0125, -6.9063, -6.8000, -6.6938, -6.5875, -6.4813, -6.3750, -6.2688, -6.1625, -6.0563, -5.9500, -5.8438, -5.7375, -5.6313, -5.5250, -5.4188, -5.3125, -5.2063, -5.1000, -4.9938, -4.8875, -4.7813, -4.6750, -4.5688, -4.4625, -4.3563, -4.2500, -4.1438, -4.0375, -3.9313, -3.8250, -3.7188, -3.6125, -3.5063, -3.4000, -3.2938, -3.1875, -3.0813, -2.9750, -2.8688, -2.7625, -2.6563, -2.5500, -2.4438, -2.3375, -2.2313, -2.1250, -2.0188, -1.9125, -1.8063, -1.7000, -1.5938, -1.4875, -1.3813, -1.2750, -1.1688, -1.0625, -0.9563, -0.8500, -0.7438, -0.6375, -0.5313, -0.4250, -0.3188, -0.2125, -0.1063, -0.0000, 0.1062, 0.2125, 0.3187, 0.4250, 0.5312, 0.6375, 0.7437, 0.8500, 0.9562, 1.0625, 1.1687, 1.2750, 1.3812, 1.4875, 1.5937, 1.7000, 1.8062, 1.9125, 2.0187, 2.1250, 2.2312, 2.3375, 2.4437, 2.5500, 2.6562, 2.7625, 2.8687, 2.9750, 3.0812, 3.1875, 3.2937, 3.4000, 3.5062, 3.6125, 3.7187, 3.8250, 3.9312, 4.0375, 4.1437, 4.2500, 4.3562, 4.4625, 4.5687, 4.6750, 4.7812, 4.8875, 4.9937, 5.1000, 5.2062, 5.3125, 5.4187, 5.5250, 5.6312, 5.7375, 5.8437, 5.9500, 6.0562, 6.1625, 6.2687, 6.3750, 6.4812, 6.5875, 6.6937, 6.8000, 6.9062, 7.0125, 7.1187, 7.2250, 7.3312, 7.4375, 7.5437, 7.6500, 7.7562, 7.8625, 7.9687, 8.0750, 8.1812, 8.2875, 8.3937, 8.5000, 8.6062, 8.7125, 8.8187, 8.9250, 9.0312, 9.1375, 9.2437, 9.3500, 9.4562, 9.5625, 9.6687, 9.7750, 9.8812, 9.9875, 10.0937, 10.2000, 10.3062, 10.4125, 10.5187, 10.6250, 10.7312, 10.8375, 10.9437, 11.0500, 11.1562, 11.2625, 11.3687, 11.4750, 11.5812, 11.6875, 11.7937, 11.9000, 12.0062, 12.1125, 12.2187, 12.3250, 12.4312, 12.5375, 12.6437, 12.7500, 12.8562, 12.9625, 13.0687, 13.1750, 13.2812, 13.3875, 13.4937, 13.6000, 13.7062, 13.8125, 13.9187, 14.0250, 14.1312, 14.2375, 14.3437, 14.4500, 14.5562, 14.6625, 14.7687, 14.8750, 14.9812, 15.0875, 15.1937, 15.3000, 15.4062, 15.5125, 15.6187, 15.7250, 15.8312, 15.9375, 16.0437, 16.1500, 16.2562, 16.3625, 16.4687, 16.5750, 16.6812, 16.7875, 16.8937, 17.0000, 17.1062, 17.2125, 17.3187, 17.4250, 17.5312, 17.6375, 17.7437, 17.8500, 17.9562, 18.0625, 18.1687, 18.2750, 18.3812, 18.4875, 18.5937, 18.7000, 18.8062, 18.9125, 19.0187, 19.1250, 19.2312, 19.3375, 19.4437, 19.5500, 19.6562, 19.7625, 19.8687, 19.9750, 20.0812, 20.1875, 20.2937, 20.4000, 20.5062, 20.6125, 20.7187, 20.8250, 20.9312, 21.0375, 21.1437, 21.2500, 21.3562, 21.4625, 21.5687, 21.6750, 21.7812, 21.8875, 21.9937, 22.1000, 22.2062, 22.3125, 22.4187, 22.5250, 22.6312, 22.7375, 22.8437, 22.9500, 23.0562, 23.1625, 23.2687, 23.3750, 23.4812, 23.5875, 23.6937, 23.8000, 23.9062, 24.0125, 24.1187, 24.2250, 24.3312, 24.4375, 24.5437, 24.6500, 24.7562, 24.8625, 24.9687, 25.0750, 25.1812, 25.2875, 25.3937};
inline double round(double val)
{
	if (val < 0)
		return ceil(val - 0.5);
	return floor(val + 0.5);
}

///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
/*Message stereo*/
// IPC_RETURN_TYPE
// publish_stereo_point_cloud(void)
// {
// 	IPC_RETURN_TYPE err;

// 	err = IPC_publishData(CARMEN_STEREO_POINT_CLOUD_NAME, &point_cloud_message);
// 	carmen_test_ipc_exit(err, "Could not publish stereo point cloud", CARMEN_STEREO_POINT_CLOUD_NAME);

// 	return err;
// }
// /*Message stereo*/
// void convert_depth_to_stereo_point_cloud(unsigned char *depth, int vertical_resolution,
// 										 int horizontal_resolution,	 unsigned char *reference_image, double timestamp)
// {
// 	carmen_vector_3D_t *image3D = (carmen_vector_3D_t *) malloc(vertical_resolution * horizontal_resolution * sizeof(carmen_vector_3D_t));
// 	float *points = (float *)depth;
// 	reproject_to_3D(points, image3D, 0.0, su);

// 	point_cloud_message.num_points = vertical_resolution * horizontal_resolution;
// 	point_cloud_message.points = (carmen_vector_3D_t *)realloc(point_cloud_message.points, point_cloud_message.num_points * sizeof(carmen_vector_3D_t));
// 	point_cloud_message.point_color = (carmen_vector_3D_t *)realloc(point_cloud_message.point_color, point_cloud_message.num_points * sizeof(carmen_vector_3D_t));

// 	int i;
// 	for (i = 0; i < point_cloud_message.num_points; i++)
// 	{
// 		point_cloud_message.points[i].x = image3D[i].x;
// 		point_cloud_message.points[i].y = image3D[i].y;
// 		point_cloud_message.points[i].z = image3D[i].z;

// 		point_cloud_message.point_color[i].x = ((double)reference_image[3 * i]) / 255.0;
// 		point_cloud_message.point_color[i].y = ((double)reference_image[3 * i + 1]) / 255.0;
// 		point_cloud_message.point_color[i].z = ((double)reference_image[3 * i + 2]) / 255.0;
// 	}

// 	point_cloud_message.timestamp = timestamp;
// 	point_cloud_message.host = carmen_get_host();

// 	publish_stereo_point_cloud();

// 	free(image3D);
// }

void convert_depth_to_velodyne_beams(unsigned char *depth, int vertical_resolution,
									 int horizontal_resolution, carmen_velodyne_shot *stereo_velodyne_scan,
									 double range_max, unsigned char *image_gray)
{
	int i, j, x, y;

	unsigned short int *points = (unsigned short int *)depth;

	double angle = horizontal_start_angle; // Horizontal angle
	double delta_angle = horizontal_camera_angle / (double)horizontal_resolution;
	for (x = 0, j = horizontal_resolution - 1; x < horizontal_resolution; x += 1, j--)
	{
		stereo_velodyne_scan[j].angle = angle;
		angle += delta_angle;
		for (y = 0, i = vertical_resolution - 1; y < vertical_resolution; y += 1, i--)
		{
			// double horizontal_angle = carmen_normalize_theta(-stereo_velodyne_scan[j].angle) / 180.0;
			//  double horizontal_angle = stereo_velodyne_scan[j].angle * M_PI / 180.0;
			double horizontal_angle = carmen_normalize_theta(carmen_degrees_to_radians(stereo_velodyne_scan[j].angle)) / M_PI;
			double range = points[(int)(y * (double)horizontal_resolution + x)] * (2 - cos(abs(horizontal_angle)));
			range = range > range_max ? 0.0 : range;
			stereo_velodyne_scan[j].distance[i] = (unsigned short)(range * range_multiplier_factor);
			stereo_velodyne_scan[j].intensity[i] = image_gray[(int)(y * (double)horizontal_resolution + x)];
		}
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void bumblebee_basic_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	// printf("bumblebee_basic_handler\n");
	Mat open_cv_image = Mat(stereo_image->height, stereo_image->width, CV_8UC3, stereo_image->raw_right, 0); // CV_32FC3 float 32 bit 3 channels (to char image use CV_8UC3)
	cv::Mat imggray;
	cv::cvtColor(open_cv_image, imggray, cv::COLOR_BGR2GRAY);
	unsigned char *image_gray = imggray.data;

	if (!strcmp(neural_network, "adabins"))
		depth_pred = libadabins_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, vertical_top_cut, vertical_down_cut);
	if (!strcmp(neural_network, "dpt"))
		depth_pred = libdpt_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, vertical_top_cut, vertical_down_cut);
	if (!strcmp(neural_network, "glpdepth"))
		depth_pred = libglpdepth_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, vertical_top_cut, vertical_down_cut);

	// cv::Rect myROI(0, 200, stereo_image->width, stereo_image->height - 200);
	// open_cv_image = open_cv_image(myROI);
	cv::Mat imgdepth = cv::Mat(open_cv_image.rows, open_cv_image.cols, CV_16U, depth_pred);

	convert_depth_to_velodyne_beams(depth_pred, vertical_resolution,
									horizontal_resolution, scan,
									range_max, image_gray);

	velodyne_partial_scan.partial_scan = scan;
	velodyne_partial_scan.number_of_shots = horizontal_resolution;
	velodyne_partial_scan.host = carmen_get_host();
	velodyne_partial_scan.timestamp = stereo_image->timestamp;
	// write_pointcloud_variable_txt(&velodyne_partial_scan, (char *)"lidar8");

	carmen_velodyne_publish_variable_scan_message(&velodyne_partial_scan, lidar_num);
	

	// cv::imshow("Bumblebee Image", open_cv_image);
	cv::imshow(neural_network, imgdepth * 500);
	waitKey(1);
}

void image_handler(camera_message *msg)
{
	// printf("camera_image_handler\n");
	camera_image *stereo_image = msg->images;
	Mat open_cv_image = Mat(stereo_image->height, stereo_image->width, CV_8UC3, stereo_image->raw_data, 0); // CV_32FC3 float 32 bit 3 channels (to char image use CV_8UC3)
	remap(open_cv_image, open_cv_image, MapX, MapY, INTER_LINEAR);											// Transforms the image to compensate for lens distortion

	cv::Mat imggray;
	cv::cvtColor(open_cv_image, imggray, cv::COLOR_BGR2GRAY);
	unsigned char *image_gray = imggray.data;
	unsigned char *image_color = open_cv_image.data;

	if (!strcmp(neural_network, "adabins"))
		depth_pred = libadabins_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, vertical_top_cut, vertical_down_cut);
	if (!strcmp(neural_network, "dpt"))
		depth_pred = libdpt_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, vertical_top_cut, vertical_down_cut);
	if (!strcmp(neural_network, "glpdepth"))
		depth_pred = libglpdepth_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, vertical_top_cut, vertical_down_cut);

	// img(cv::Rect(xMin,yMin,xMax-xMin,yMax-yMin)).copyTo(croppedImg);
	// cv::Rect myROI(0, camera_height - vertical_resolution, stereo_image->width, camera_height - (camera_height - vertical_resolution));
	// open_cv_image = open_cv_image(myROI);
	cv::Mat imgdepth = cv::Mat(open_cv_image.rows, open_cv_image.cols, CV_16U, depth_pred);

	convert_depth_to_velodyne_beams(depth_pred, vertical_resolution,
									horizontal_resolution, scan,
									range_max, image_gray);

	velodyne_partial_scan.partial_scan = scan;
	velodyne_partial_scan.number_of_shots = horizontal_resolution;
	velodyne_partial_scan.host = carmen_get_host();
	velodyne_partial_scan.timestamp = msg->timestamp;
	// write_pointcloud_variable_txt(&velodyne_partial_scan, (char *)"lidar8");
	carmen_velodyne_publish_variable_scan_message(&velodyne_partial_scan, lidar_num);
	// convert_depth_to_stereo_point_cloud(depth_pred, vertical_resolution,
	// 								horizontal_resolution, image_color, msg->timestamp);

	// cv::imshow("Camera Driver Image", open_cv_image);
	cv::imshow(neural_network, imgdepth * 500.0);
	waitKey(1);
}

///////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////

carmen_velodyne_shot *
alloc_velodyne_shot_scan_vector(int horizontal_resolution_l, int vertical_resolution_l)
{
	int i;

	carmen_velodyne_shot *vector = (carmen_velodyne_shot *)malloc(horizontal_resolution_l * sizeof(carmen_velodyne_shot));
	carmen_test_alloc(vector);

	for (i = 0; i < horizontal_resolution_l; i++)
	{
		vector[i].distance = (unsigned int *)calloc(vertical_resolution_l, sizeof(unsigned int));
		carmen_test_alloc(vector[i].distance);
		vector[i].intensity = (unsigned short *)calloc(vertical_resolution_l, sizeof(unsigned short));
		carmen_test_alloc(vector[i].distance);
		vector[i].shot_size = vertical_resolution_l;
	}

	return (vector);
}

static void
init_stereo_velodyne()
{
	scan = alloc_velodyne_shot_scan_vector(horizontal_resolution, vertical_resolution);

	reference_image = cvCreateImage(cvSize(camera_width, camera_height), IPL_DEPTH_8U, 3);
	reference_image_gray = cvCreateImage(cvSize(camera_width, camera_height), IPL_DEPTH_8U, 1);
}
/* per module */
static char ***variables, ***values;
static int **expert;
static int *num_params;
static int **update_param_mask;
static int **num_param_mask;

void concatenate(std::string &s, const char *c)
{
	s.reserve(s.size() + strlen(c));
	s.append(c);
}

static void init_params_edit()
{
	int m, p;

	carmen_param_get_modules(&modules, &num_modules);
	variables = (char ***)calloc(num_modules, sizeof(void *));
	carmen_test_alloc(variables);
	values = (char ***)calloc(num_modules, sizeof(void *));
	carmen_test_alloc(values);
	expert = (int **)calloc(num_modules, sizeof(void *));
	carmen_test_alloc(expert);
	num_params = (int *)calloc(num_modules, sizeof(int));
	carmen_test_alloc(num_params);
	update_param_mask = (int **)calloc(num_modules, sizeof(int *));
	carmen_test_alloc(update_param_mask);
	num_param_mask = (int **)calloc(num_modules, sizeof(int *));
	carmen_test_alloc(num_param_mask);

	for (m = 0; m < num_modules; m++)
	{
		carmen_param_get_all(modules[m], variables + m, values + m, expert + m,
							 num_params + m);
		update_param_mask[m] = (int *)calloc(num_params[m], sizeof(int));
		carmen_test_alloc(update_param_mask[m]);
		num_param_mask[m] = (int *)calloc(num_params[m], sizeof(int));
		carmen_test_alloc(num_param_mask[m]);
	}
	char module[256];
	char variable[256];
	char variable_ray_order[256];
	char variable_shot_size[256];

	string vertical_angles;
	string ray_order;
	double v_angle = (vertical_camera_angle / 2.0) * (-1);
	double delta_v_angle = vertical_camera_angle / camera_height;

	sprintf(module, "%s%d", "lidar", lidar_num);
	sprintf(variable, "%s", "vertical_angles");
	sprintf(variable_ray_order, "%s", "ray_order");
	sprintf(variable_shot_size, "%s", "shot_size");

	char value_string[256];
	char value_ray_string[256];
	for (int i = 0; i < camera_height; i++)
	{ // 480
		sprintf(value_string, "%0.4f ", v_angle);
		concatenate(vertical_angles, value_string);
		sprintf(value_ray_string, "%d ", camera_height - i - 1);
		concatenate(ray_order, value_ray_string);
		v_angle += delta_v_angle;
	}
	char *return_value;
	int status = 0;
	int n = vertical_angles.length();
	char char_vertical_angles[n + 1];
	strcpy(char_vertical_angles, vertical_angles.c_str());

	n = ray_order.length();
	char char_ray_order[n + 1];
	strcpy(char_ray_order, ray_order.c_str());

	char char_shot_size[256];
	sprintf(char_shot_size, "%d", camera_height);

	for (m = 0; m < num_modules; m++)
	{
		if (!strcmp(modules[m], module))
		{
			carmen_param_set_module(modules[m]);
			for (p = 0; p < num_params[m]; p++)
			{
				if (!strcmp(variables[m][p], variable))
				{
					update_param_mask[m][p] = -2;
					if (carmen_param_set_variable(variables[m][p], char_vertical_angles,
												  &return_value) < 0)
						status = -1;
					else
					{
						status = 1;
						update_param_mask[m][p] = 0;
					}
					cout << module << "_" << variable << " " << char_vertical_angles << endl;
				}
				if (!strcmp(variables[m][p], variable_ray_order))
				{
					update_param_mask[m][p] = -2;
					if (carmen_param_set_variable(variables[m][p], char_ray_order,
												  &return_value) < 0)
						status = -1;
					else
					{
						status = 1;
						update_param_mask[m][p] = 0;
					}
					cout << module << "_" << variable_ray_order << " " << char_ray_order << endl;
				}
				if (!strcmp(variables[m][p], variable_shot_size))
				{
					update_param_mask[m][p] = -2;
					if (carmen_param_set_variable(variables[m][p], char_shot_size,
												  &return_value) < 0)
						status = -1;
					else
					{
						status = 1;
						update_param_mask[m][p] = 0;
					}
					cout << module << "_" << variable_shot_size << " " << char_shot_size << endl;
				}
			}
			break;
		}
	}

	if (status == 1)
		cout << "Saving parameters...done" << endl;
	else if (status == 0)
		cout << "Saving parameters...nothing to save" << endl;
	else
		cout << "Saving parameters...failed" << endl;

	for (m = 0; m < num_modules; m++)
	{
		for (p = 0; p < num_params[m]; p++)
		{
			free(variables[m][p]);
			free(values[m][p]);
		}
		free(modules[m]);
		free(variables[m]);
		free(values[m]);
		free(update_param_mask[m]);
		free(num_param_mask[m]);
	}
	free(modules);
	free(variables);
	free(values);
	free(num_params);
	free(update_param_mask);
	free(num_param_mask);
}

int read_parameters(int argc, char **argv)
{
	int num_items;

	char stereo_string[256];
	char camera_string[256];

	sprintf(stereo_string, "%s%d", "stereo", atoi(argv[1]));
	sprintf(camera_string, "%s%d", "intelbras", atoi(argv[1]));

	carmen_param_t param_list[] = {
		{(char *)stereo_string, (char *)"vertical_resolution", CARMEN_PARAM_INT, &vertical_resolution, 1, NULL},
		{(char *)stereo_string, (char *)"horizontal_resolution", CARMEN_PARAM_INT, &horizontal_resolution, 1, NULL},
		{(char *)stereo_string, (char *)"range_max", CARMEN_PARAM_DOUBLE, &range_max, 1, NULL},
		{(char *)stereo_string, (char *)"width", CARMEN_PARAM_INT, &camera_width, 1, NULL},
		{(char *)stereo_string, (char *)"height", CARMEN_PARAM_INT, &camera_height, 1, NULL},
		{(char *)stereo_string, (char *)"horizontal_camera_angle", CARMEN_PARAM_DOUBLE, &horizontal_camera_angle, 1, NULL},
		{(char *)stereo_string, (char *)"vertical_camera_angle", CARMEN_PARAM_DOUBLE, &vertical_camera_angle, 1, NULL},
		{(char *)stereo_string, (char *)"vertical_top_cut", CARMEN_PARAM_INT, &vertical_top_cut, 1, NULL},
		{(char *)stereo_string, (char *)"vertical_down_cut", CARMEN_PARAM_INT, &vertical_down_cut, 1, NULL},
		{(char *)stereo_string, (char *)"horizontal_start_angle", CARMEN_PARAM_DOUBLE, &horizontal_start_angle, 1, NULL},
		{(char *)stereo_string, (char *)"range_multiplier_factor", CARMEN_PARAM_DOUBLE, &range_multiplier_factor, 1, NULL},
		{(char *)stereo_string, (char *)"lidar", CARMEN_PARAM_INT, &lidar_num, 1, NULL},
		{(char *)camera_string, (char *)"fx", CARMEN_PARAM_DOUBLE, &fx, 0, NULL},
		{(char *)camera_string, (char *)"fy", CARMEN_PARAM_DOUBLE, &fy, 0, NULL},
		{(char *)camera_string, (char *)"cu", CARMEN_PARAM_DOUBLE, &cu, 0, NULL},
		{(char *)camera_string, (char *)"cv", CARMEN_PARAM_DOUBLE, &camera_cv, 0, NULL},
		{(char *)camera_string, (char *)"k1", CARMEN_PARAM_DOUBLE, &k1, 0, NULL},
		{(char *)camera_string, (char *)"k2", CARMEN_PARAM_DOUBLE, &k2, 0, NULL},
		{(char *)camera_string, (char *)"k3", CARMEN_PARAM_DOUBLE, &k3, 0, NULL},
		{(char *)camera_string, (char *)"p1", CARMEN_PARAM_DOUBLE, &p1, 0, NULL},
		{(char *)camera_string, (char *)"p2", CARMEN_PARAM_DOUBLE, &p2, 0, NULL}};

	/*SU stereo*/
	// if ((camera_width > 0) && (camera_height > 0))
	// {
	// 	su.width = camera_width;
	// 	su.height = camera_height;
	// }
	// su.baseline = 0.0;
	// su.fx = fx * su.width;
	// su.fy = fy * su.height;
	// su.xc = cu * su.width;
	// su.yc = camera_cv * su.height;
	// su.vfov = 2 * atan(camera_height / (2 * su.fy));
	// su.hfov = 2 * atan(camera_width / (2 * su.fx));
	// su.stereo_stride_x = 1.0;
	// su.stereo_stride_y = 1.0;
	/*----*/

	if (vertical_resolution > vertical_resolution - 0)
		carmen_die("The stereo_velodyne_vertical_resolution is bigger than stereo point cloud height");

	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	carmen_param_t param_optional_list[] =
		{
			{(char *)"commandline", (char *)"neural_network", CARMEN_PARAM_STRING, &neural_network, 0, NULL},
		};
	carmen_param_install_params(argc, argv, param_optional_list, sizeof(param_optional_list) / sizeof(param_optional_list[0]));

	double fx_rect = fx * camera_width;
	double fy_rect = fy * camera_height;
	double cu_rect = cu * camera_width;
	double cv_rect = camera_cv * camera_height;

	cameraMatrix = (Mat_<double>(3, 3) << fx_rect, 0, cu_rect, 0, fy_rect, cv_rect, 0, 0, 1);
	newcameramtx = (Mat_<double>(3, 3) << fx_rect, 0, cu_rect, 0, fy_rect, cv_rect, 0, 0, 1);
	distCoeffs = (Mat_<double>(5, 1) << k1, k2, p1, p2, k3);
	R1 = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

	initUndistortRectifyMap(cameraMatrix, distCoeffs, R1, newcameramtx, Size(camera_width, camera_height), CV_16SC2, MapX, MapY);

	init_params_edit();
	return (0);
}

int main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	if (argc - 1 != 3)
		carmen_die("%s: Wrong number of parameters. stereo requires 3 parameters and received %d parameter(s). \n"
				   "Usage:\n %s <camera_number>"
				   " -neural_network <n>    : neural_network option (adabins, dpt or glpdepth)\n",
				   argv[0], argc - 1, argv[0]);

	camera = atoi(argv[1]);
	read_parameters(argc, argv);

	if (!strcmp(neural_network, "adabins"))
		initialize_python_context_adabins();

	if (!strcmp(neural_network, "dpt"))
		initialize_python_context_dpt();

	if (!strcmp(neural_network, "glpdepth"))
	{
		initialize_python_context_glpdepth();
	}

	init_stereo_velodyne();

	carmen_velodyne_define_messages();
	if (camera == 3)
	{
		carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t)bumblebee_basic_handler, CARMEN_SUBSCRIBE_LATEST);
	}
	else
	{
		camera_drivers_subscribe_message(camera, NULL, (carmen_handler_t)image_handler, CARMEN_SUBSCRIBE_LATEST);
	}

	carmen_ipc_dispatch();

	return 0;
}
