#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/visual_tracker_interface.h>
#include <string>
#include "velodyne_camera_calibration.h"
#include <tf.h>

using namespace cv;
using namespace tf;

const int RANGE_WINDOW_ZOOM = 5;
const int WINDOW_WIDTH = 1366;

int bumblebee_received = 0;

int camera;
int camera_id;
int bumblebee_basic_width;
int bumblebee_basic_height;

carmen_bumblebee_basic_stereoimage_message bumblebee_message;

static cv::Rect box;
static double confidence;
std::vector<carmen_velodyne_points_in_cam_t> laser_in_cam_px;
// CAM POSE: 0.245000 0.283000 0.075000 0.017453 0.026300 0.000000 FX: 0.753883
// CAM POSE: 0.245000 0.283000 0.085000 0.017453 0.026300 -0.017453 FX: 0.753883

//CAM POSE: 0.245000 -0.207000 0.085000 0.017453 0.026300 -0.017453 FX: 0.753883


//double camera_x = 0.245;
//double camera_y = -0.207000; //0.283; //-0.287;
//double camera_z = 0.085;
//double camera_roll = 0.017453; //0;
//double camera_pitch = 0.0263;
//double camera_yaw = -0.017453; // 0;
//
//double fx = 0.753883;
//double fy = 1.00518;
//
//const int column_correspondence[32] =
//{
//	0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23, 8,
//	24, 9, 25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31
//};
//
//
//const static double sorted_vertical_angles[32] =
//{
//	-30.67, -29.33, -28.0, -26.67, -25.33, -24.0, -22.67, -21.33, -20.0,
//	-18.67, -17.33, -16.0, -14.67, -13.33, -12.0, -10.67, -9.3299999, -8.0,
//	-6.6700001, -5.3299999, -4.0, -2.6700001, -1.33, 0.0, 1.33, 2.6700001, 4.0,
//	5.3299999, 6.6700001, 8.0, 9.3299999, 10.67
//};



void
show_velodyne(carmen_velodyne_partial_scan_message *velodyne_message)
{
	if (!bumblebee_received)
		return;

	static int first = 1;

	double average_range = 0.0;
	int count_range = 0;


	if (first)
	{
		namedWindow("Bumblebee");
		//moveWindow("Bumblebee", 900, 300);

		first = 0;
	}

	static Mat *bumb_image = new Mat(Size(bumblebee_message.width, bumblebee_message.height), CV_8UC3);
	static Mat *bumb_zoom = new Mat(Size(bumblebee_message.width / 2, bumblebee_message.height / 2), CV_8UC3);

	static Mat *bum_image_640 = new Mat(Size(640, 480), CV_8UC3);


	memset(bumb_image->data, 0, bumb_image->step * bumb_image->rows * sizeof(uchar));

	for (int i = 0; i < bumblebee_message.height; i++)
	{
		for (int j = 0; j < bumblebee_message.width; j++)
		{
			bumb_image->data[i * bumb_image->step + 3 * j + 0] = (uchar) bumblebee_message.raw_right[3 * (i * bumblebee_message.width + j) + 2];
			bumb_image->data[i * bumb_image->step + 3 * j + 1] = (uchar) bumblebee_message.raw_right[3 * (i * bumblebee_message.width + j) + 1];
			bumb_image->data[i * bumb_image->step + 3 * j + 2] = (uchar) bumblebee_message.raw_right[3 * (i * bumblebee_message.width + j) + 0];
		}
	}


	float r = 0;
//	float b = 0;
//	float g = 0;
	const float MAX_RANGE = 30.0;
	const float MIN_RANGE = 0.5;


	laser_in_cam_px = carmen_velodyne_camera_calibration_lasers_points_in_camera(velodyne_message, &bumblebee_message);
	rectangle(*bumb_image, cv::Point(box.x, box.y),cv::Point(box.x + box.width, box.y + box.height), Scalar( 0, r, 0 ), 1, 4 );

	for(int i = 0; i < laser_in_cam_px.size(); i++)
	{
		if((laser_in_cam_px.at(i).ipx > box.x) && ( laser_in_cam_px.at(i).ipx < (box.x + box.width)) && (laser_in_cam_px.at(i).ipy > box.y) &&
				(laser_in_cam_px.at(i).ipy < (box.y + box.height)) &&	(confidence > 0))
		{
			if (laser_in_cam_px.at(i).laser_polar.length <= MIN_RANGE)
				laser_in_cam_px.at(i).laser_polar.length = MAX_RANGE;

			if (laser_in_cam_px.at(i).laser_polar.length > MAX_RANGE)
				laser_in_cam_px.at(i).laser_polar.length = MAX_RANGE;

			r = laser_in_cam_px.at(i).laser_polar.length / MAX_RANGE;
			r *= 255;
			r = 255 - r;

			average_range += laser_in_cam_px.at(i).laser_polar.length;
			count_range += 1;


			//printf("%f\t %f\t %f\t \n",r, range, velodyne_message->partial_scan[j].distance[i] );
			circle(*bumb_image, cv::Point(laser_in_cam_px.at(i).ipx, laser_in_cam_px.at(i).ipy), 2, Scalar(0, r, 0), -1);

		}

	}

	if (confidence > 0)
	{
		double averange_distance = average_range / count_range;
		char str[50];
		sprintf(str,"range: %f",averange_distance);
		putText(*bumb_image, str, Point2f(10,20), FONT_HERSHEY_PLAIN, 1.2,  Scalar(0,0,0));
	}

	resize(*bumb_image, *bum_image_640, bum_image_640->size());

	resize(*bumb_image, *bumb_zoom, bumb_zoom->size());

	imshow("Bumblebee", *bum_image_640);

	int k = waitKey(5);

}


//void
//show_velodyne_old(carmen_velodyne_partial_scan_message *velodyne_message)
//{
//	if (!bumblebee_received)
//		return;
//
//	static int first = 1;
//
//	double average_range = 0.0;
//	int count_range = 0;
//
//
//	if (first)
//	{
//		namedWindow("Range");
//		//namedWindow("Intensity");
//		namedWindow("Colored");
//		//namedWindow("Bumblebee");
//
//		moveWindow("Range", 50, 400);
//		//moveWindow("Intensity", 50, 300);
//		moveWindow("Colored", 50, 600);
//		//moveWindow("Bumblebee", 900, 300);
//
//		first = 0;
//	}
//
//	static Mat *range_orig = new Mat(Size(velodyne_message->number_of_32_laser_shots, 32), CV_8UC3);
//	static Mat *range_zoom = new Mat(Size(WINDOW_WIDTH /*range_zoom * velodyne_message->number_of_32_laser_shots*/, RANGE_WINDOW_ZOOM * 32), CV_8UC3);
//	static Mat *intensity_orig = new Mat(Size(velodyne_message->number_of_32_laser_shots, 32), CV_8UC3);
//	static Mat *intensity_zoom = new Mat(Size(WINDOW_WIDTH /*range_zoom * velodyne_message->number_of_32_laser_shots*/, RANGE_WINDOW_ZOOM * 32), CV_8UC3);
//	static Mat *bumb_image = new Mat(Size(bumblebee_message.width, bumblebee_message.height), CV_8UC3);
//	static Mat *bumb_zoom = new Mat(Size(bumblebee_message.width / 2, bumblebee_message.height / 2), CV_8UC3);
//
//	static Mat *bum_image_640 = new Mat(Size(640, 480), CV_8UC3);
//
//
//	static Mat *colored_orig = new Mat(Size(velodyne_message->number_of_32_laser_shots, 32), CV_8UC3);
//	static Mat *colored_zoom = new Mat(Size(WINDOW_WIDTH /*range_zoom * velodyne_message->number_of_32_laser_shots*/, RANGE_WINDOW_ZOOM * 32), CV_8UC3);
//
//	if (range_orig->cols != velodyne_message->number_of_32_laser_shots)
//	{
//		delete(range_orig);
//		delete(range_zoom);
//
//		delete(intensity_orig);
//		delete(intensity_zoom);
//
//		delete(colored_orig);
//		delete(colored_zoom);
//
//		range_orig = new Mat(Size(velodyne_message->number_of_32_laser_shots, 32), CV_8UC3);
//		range_zoom = new Mat(Size(WINDOW_WIDTH /*range_zoom * velodyne_message->number_of_32_laser_shots*/, RANGE_WINDOW_ZOOM * 32), CV_8UC3);
//
//		intensity_orig = new Mat(Size(velodyne_message->number_of_32_laser_shots, 32), CV_8UC3);
//		intensity_zoom = new Mat(Size(WINDOW_WIDTH /*range_zoom * velodyne_message->number_of_32_laser_shots*/, RANGE_WINDOW_ZOOM * 32), CV_8UC3);
//
//		colored_orig = new Mat(Size(velodyne_message->number_of_32_laser_shots, 32), CV_8UC3);
//		colored_zoom = new Mat(Size(WINDOW_WIDTH /*range_zoom * velodyne_message->number_of_32_laser_shots*/, RANGE_WINDOW_ZOOM * 32), CV_8UC3);
//	}
//
//	memset(bumb_image->data, 0, bumb_image->step * bumb_image->rows * sizeof(uchar));
//	memset(colored_orig->data, 0, colored_orig->step * colored_orig->rows * sizeof(uchar));
//
//	for (int i = 0; i < bumblebee_message.height; i++)
//	{
//		for (int j = 0; j < bumblebee_message.width; j++)
//		{
//			bumb_image->data[i * bumb_image->step + 3 * j + 0] = (uchar) bumblebee_message.raw_right[3 * (i * bumblebee_message.width + j) + 2];
//			bumb_image->data[i * bumb_image->step + 3 * j + 1] = (uchar) bumblebee_message.raw_right[3 * (i * bumblebee_message.width + j) + 1];
//			bumb_image->data[i * bumb_image->step + 3 * j + 2] = (uchar) bumblebee_message.raw_right[3 * (i * bumblebee_message.width + j) + 0];
//		}
//	}
//
//	for (int i = 0; i < 32; i++)
//	{
//		double v = carmen_normalize_theta(carmen_degrees_to_radians(sorted_vertical_angles[i]));
//
//		for (int j = 0; j < velodyne_message->number_of_32_laser_shots; j++)
//		{
//			double range = (((double) velodyne_message->partial_scan[j].distance[i]) / 500.0);
//			double intensity = (((double) velodyne_message->partial_scan[j].intensity[i])) *  10;
//
//
//			int l = range_orig->rows - i - 1;
//			double h = carmen_normalize_theta(carmen_degrees_to_radians(velodyne_message->partial_scan[j].angle));
//
//			float r = 0;
//			float b = 0;
//			float g = 0;
//
//			const float MAX_RANGE = 30.0;
//			const float MIN_RANGE = 0.5;
//
//			if (range <= MIN_RANGE)
//				range = MAX_RANGE;
//
//			if (range > MAX_RANGE)
//				range = MAX_RANGE;
//
//			r = range / MAX_RANGE;
//			r *= 255;
//			r = 255 - r;
//
//			tf::Point p3d_velodyne_reference = spherical_to_cartesian(h, v, range);
//
//			range_orig->data[l * range_orig->step + 3 * j + 0] = (uchar) b;
//			range_orig->data[l * range_orig->step + 3 * j + 1] = (uchar) g;
//			range_orig->data[l * range_orig->step + 3 * j + 2] = (uchar) r;
//
//			intensity_orig->data[l * intensity_orig->step + 3 * j + 0] = (uchar) intensity;
//			intensity_orig->data[l * intensity_orig->step + 3 * j + 1] = (uchar) intensity;
//			intensity_orig->data[l * intensity_orig->step + 3 * j + 2] = (uchar) intensity;
//
//			if (p3d_velodyne_reference.x() > 0)
//			{
////				g = 254;
//
//				tf::Point p3d_camera_reference = move_to_camera_reference(p3d_velodyne_reference);
//
//				const int XB3_MAX_PIXEL_WIDTH = 640;//1280;
//				const int XB3_MAX_PIXEL_HEIGHT = 480; //960;
//				const double XB3_PIXEL_SIZE = 0.00000375f;//pixel size (in meters)
//
//				double ccd_width = XB3_MAX_PIXEL_WIDTH * XB3_PIXEL_SIZE;
//
//				double f_meters = fx * XB3_MAX_PIXEL_WIDTH * XB3_PIXEL_SIZE;
//
//				double cu = 0.500662 * (double) bumblebee_message.width;
//				double cv = 0.506046 * (double) bumblebee_message.height;
//
//				double px = (f_meters * (p3d_camera_reference.y() / p3d_camera_reference.x()) / XB3_PIXEL_SIZE + cu);
//				double py = (f_meters * (-p3d_camera_reference.z() / p3d_camera_reference.x()) / XB3_PIXEL_SIZE + cv);
//
//				int ipx = (int) px;
//				int ipy = (int) py;
//
//				if (px >= 0 && px <= bumblebee_message.width && py >= 0 && py <= bumblebee_message.height)
//				{
//					if (px < 10 || px >= (bumblebee_message.width - 10))
//						b = 254;
//
//					colored_orig->data[l * colored_orig->step + 3 * j + 0] =
//							(uchar) bumblebee_message.raw_right[3 * (ipy * bumblebee_message.width + ipx) + 2];
//					colored_orig->data[l * colored_orig->step + 3 * j + 1] =
//							(uchar) bumblebee_message.raw_right[3 * (ipy * bumblebee_message.width + ipx) + 1];
//					colored_orig->data[l * colored_orig->step + 3 * j + 2] =
//							(uchar) bumblebee_message.raw_right[3 * (ipy * bumblebee_message.width + ipx) + 0];
//
////					if (h < 0)
////					{
////						bumb_image->data[ipy * bumb_image->step + 3 * ipx + 0] = (uchar) 0;
////						bumb_image->data[ipy * bumb_image->step + 3 * ipx + 1] = (uchar) 0;
////						bumb_image->data[ipy * bumb_image->step + 3 * ipx + 2] = (uchar) r;
//
//						if (r > 5)
//						{
//							//	a < x0 < a+c and b < y0 < b + d
//							if((ipx > box.x) && ( ipx < (box.x + box.width)) && (ipy > box.y) && ( ipy < (box.y + box.height)) &&
//									(confidence > 0)){
//
//								average_range += range;
//								count_range += 1;
//
//
////								printf("%f\t %f\t %f\t \n",r, range, velodyne_message->partial_scan[j].distance[i] );
//								circle(*bumb_image, cv::Point(ipx, ipy), 2, Scalar(0, r, 0), -1);
//								rectangle(*bumb_image, cv::Point(box.x, box.y),cv::Point(box.x + box.width, box.y + box.height), Scalar( 0, r, 0 ), 1, 4 );
//
//							}
//							else
//							{
//								circle(*bumb_image, cv::Point(ipx, ipy), 2, Scalar(0, 0, r), -1);
//
//							}
//
//							range_orig->data[l * range_orig->step + 3 * j + 0] ^= (uchar) bumblebee_message.raw_right[3 * (ipy * bumblebee_message.width + ipx) + 2];
//							range_orig->data[l * range_orig->step + 3 * j + 1] ^= (uchar) bumblebee_message.raw_right[3 * (ipy * bumblebee_message.width + ipx) + 1];
//							range_orig->data[l * range_orig->step + 3 * j + 2] ^= (uchar) bumblebee_message.raw_right[3 * (ipy * bumblebee_message.width + ipx) + 0];
//
//
//						}
//
////					}
////					else
////					{
////						bumb_image->data[ipy * bumb_image->step + 3 * ipx + 0] = (uchar) 0;
////						bumb_image->data[ipy * bumb_image->step + 3 * ipx + 1] = (uchar) 0;
////						bumb_image->data[ipy * bumb_image->step + 3 * ipx + 2] = (uchar) 255;
////					}
//				}
//
////				if (h < 0.1 && h > -0.1)
////				{
////					printf("VELODYNE: %lf %lf %lf\n", p3d_velodyne_reference.x(), p3d_velodyne_reference.y(), p3d_velodyne_reference.z());
////					printf("CAMERA: %lf %lf %lf\n", p3d_camera_reference.x(), p3d_camera_reference.y(), p3d_camera_reference.z());
////					printf("PIXEL: %lf %lf\n", px, py);
////					printf("cu: %lf cv: %lf focal_p: %lf f_meteres: %lf\n", cu, cv, focal_pixel, f_meters);
////
////					getchar();
////				}
//			}
//
//			//glColor3f(((range - min_range) / (max_range - min_range)), 0.0, 0.0);
//			//glVertex2i((zoom * x + l), (zoom * j + k));
//		}
//	}
//
//	if (confidence > 0)
//	{
//		double averange_distance = average_range / count_range;
//		char str[50];
//		sprintf(str,"range: %f",averange_distance);
//		putText(*bumb_image, str, Point2f(10,20), FONT_HERSHEY_PLAIN, 1.2,  Scalar(0,0,0));
//	}
////	if( confidence > 0)
////		printf("averange_distance = %f\n", averange_distance);
//
//	resize(*range_orig, *range_zoom, range_zoom->size());
//	resize(*intensity_orig, *intensity_zoom, intensity_zoom->size());
//
//
//	resize(*bumb_image, *bum_image_640, bum_image_640->size());
//
//	resize(*bumb_image, *bumb_zoom, bumb_zoom->size());
//	resize(*colored_orig, *colored_zoom, colored_zoom->size());
//
//	imshow("Range", *range_zoom);
////	imshow("Range ORIG", *range_orig);
////	imshow("Intensity", *intensity_zoom);
//	imshow("Bumblebee", *bum_image_640);
//	imshow("Colored", *colored_zoom);
//
//	int k = waitKey(5);
//
//	if (k == 'q') camera_x += 0.01;
//	if (k == 'a') camera_x -= 0.01;
//	if (k == 'w') camera_y += 0.01;
//	if (k == 's') camera_y -= 0.01;
//	if (k == 'e') camera_z += 0.01;
//	if (k == 'd') camera_z -= 0.01;
//
//	if (k == 'r') camera_roll += carmen_degrees_to_radians(1);
//	if (k == 'f') camera_roll -= carmen_degrees_to_radians(1);
//	if (k == 't') camera_pitch += carmen_degrees_to_radians(1);
//	if (k == 'g') camera_pitch -= carmen_degrees_to_radians(1);
//	if (k == 'y') camera_yaw += carmen_degrees_to_radians(1);
//	if (k == 'h') camera_yaw -= carmen_degrees_to_radians(1);
//
//	if (k == 'u') fx += 0.01;
//	if (k == 'j') fx -= 0.01;
//
//	if (k == 'q' || k == 'a' || k == 'w' || k == 's' || k == 'e' || k == 'd' ||
//			k == 'r' || k == 'f' || k == 't' || k == 'g' || k == 'y' || k == 'h' ||
//			k == 'u' || k == 'j')
//	{
//		printf("CAM POSE: %lf %lf %lf %lf %lf %lf FX: %lf\n", camera_x, camera_y, camera_z,
//				camera_roll, camera_pitch, camera_yaw, fx);
//	}
//}


void
bumblebee_basic_image_handler(carmen_bumblebee_basic_stereoimage_message *bumblebee_basic_message)
{
	bumblebee_received = 1;

//	static int n = 0;
//	static double time = 0;
//
//	if (bumblebee_basic_message->timestamp - time > 1.0)
//	{
//		printf("BUMB: %d\n", n);
//		time = bumblebee_basic_message->timestamp;
//		n = 0;
//	}
//	else
//		n++;
}


void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
//	static int n = 0;
//	static double time = 0;
//
//	if (velodyne_message->timestamp - time > 1.0)
//	{
//		printf("VELO: %d\n", n);
//		time = velodyne_message->timestamp;
//		n = 0;
//	}
//	else
//		n++;

	carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(velodyne_message);
	show_velodyne(velodyne_message);
}

void
carmen_visual_tracker_output_message_handler(carmen_visual_tracker_output_message *visual_tracker_output_message)
{

	box.x = visual_tracker_output_message->rect.x;
	box.y = visual_tracker_output_message->rect.y;
	box.width = visual_tracker_output_message->rect.width;
	box.height = visual_tracker_output_message->rect.height;

	confidence = visual_tracker_output_message->confidence;

}

int
read_parameters(int argc, char **argv, int camera)
{
	int num_items;
	char bumblebee_string[256];

	sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera);

	carmen_param_t param_list[] = {
			{ bumblebee_string, (char*) "width", CARMEN_PARAM_INT, &bumblebee_basic_width, 0, NULL },
			{ bumblebee_string, (char*) "height", CARMEN_PARAM_INT, &bumblebee_basic_height, 0, NULL } };

	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}


int
main(int argc, char **argv)
{
	if (argc != 2)
	{
		fprintf(stderr, "%s: Wrong number of parameters. Requires 2 parameter and received %d. \n Usage: %s <camera_number>\n>", argv[0], argc - 1, argv[0]);
		exit(1);
	}

	camera_id = atoi(argv[1]);
//	camera_side = atoi(argv[2]);

	carmen_ipc_initialize(argc, argv);

	read_parameters(argc, argv, camera_id);

	carmen_bumblebee_basic_subscribe_stereoimage(camera_id, &bumblebee_message,
			(carmen_handler_t) bumblebee_basic_image_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_velodyne_subscribe_partial_scan_message(NULL,
			(carmen_handler_t)velodyne_partial_scan_message_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_visual_tracker_subscribe_output(NULL,
				(carmen_handler_t)carmen_visual_tracker_output_message_handler,
				CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();

	return 0;
}

