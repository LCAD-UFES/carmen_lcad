#include <carmen/carmen.h>
#include <carmen/global.h>
#include <carmen/stereo_util.h>
#include <carmen/stereo_velodyne_interface.h>
#include <carmen/stereo_messages.h>
#include <carmen/stereo_interface.h>
#include <carmen/stereo_velodyne.h>
#include <carmen/libadabins.h>
#include <string.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <string.h>
#include <iostream>


using namespace std;
using namespace cv;

static int camera;
static stereo_util instance;
static int vertical_resolution;
static int horizontal_resolution;
static double range_max;
static int flipped;
static int vertical_roi_ini;
static int vertical_roi_end;

static int horizontal_roi_ini;
static int horizontal_roi_end;

static int bumblebee_basic_width = 0;
static int bumblebee_basic_height = 0;

static carmen_velodyne_shot *scan;

static IplImage *reference_image;
static IplImage *reference_image_gray;

float * depth;


void
copy_channel(IplImage *src, IplImage *dst, int channel)
{
	int i;

	for (i = 0; i < reference_image->width * reference_image->height; i++)
		dst->imageData[3 * i + channel] = src->imageData[i];
}


void
copy_one_channel(IplImage *src, IplImage *dst, int channel)
{
	int i;

	for (i = 0; i < reference_image->width * reference_image->height; i++)
		dst->imageData[i] = src->imageData[3 * i + channel];
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_point_cloud_old(unsigned char *depth_pred, int number_of_rows, int number_of_cols, double timestamp)
{
	carmen_velodyne_variable_scan_message msg;

	msg.host = carmen_get_host();
	msg.timestamp = timestamp;
	msg.number_of_shots = number_of_cols;

	msg.partial_scan = (carmen_velodyne_shot *) malloc(msg.number_of_shots * sizeof(carmen_velodyne_shot));
	//for (int i = 0; i < number_of_cols; i--){
	// 	printf("%d ", i);
	// }


	unsigned short int *points = (unsigned short int *) depth_pred;
	double angle = -25.0;
	double delta_angle = 50.0 / (double) msg.number_of_shots;
	for (int i = 0; i < number_of_cols; i++)
	{
		msg.partial_scan[i].shot_size = number_of_rows;
		msg.partial_scan[i].angle = angle;
		angle += delta_angle;

		msg.partial_scan[i].distance = (unsigned int *) malloc(msg.partial_scan[i].shot_size * sizeof(unsigned int));
		msg.partial_scan[i].intensity = (unsigned short int *) malloc(msg.partial_scan[i].shot_size * sizeof(unsigned short int));
//		double ag = -25.0 / 2.0;
//		double delta_ag = 25.0 / msg.partial_scan[i].shot_size;
		for (int j = 0; j < msg.partial_scan[i].shot_size; j++)
		{
//			printf("%lf ", ag);
//			ag += delta_ag;
			double horizontal_angle = angle * M_PI / 180.0;
			msg.partial_scan[i].distance[j] = points[i + j * number_of_cols] * ( 2 - cos(abs(horizontal_angle))); // * ( 2 - cos(abs(horizontal_angle))); // points[i + j * number_of_cols];
			// msg.partial_scan[i].distance[j] = ((unsigned int) imgdepth.at<unsigned short int>(j, i)) * ( 2 - cos(abs(horizontal_angle))); 
			msg.partial_scan[i].intensity[j] = 100;
		}
//		printf("\n\n");
	}

	carmen_velodyne_publish_variable_scan_message(&msg, 8);

	for (int i = 0; i < msg.number_of_shots; i++)
	{
		free(msg.partial_scan[i].distance);
		free(msg.partial_scan[i].intensity);
	}
	free(msg.partial_scan);
}

void
publish_point_cloud(unsigned char *depth_pred, int number_of_rows, int number_of_cols, double timestamp, unsigned char *gray)
{
	carmen_velodyne_variable_scan_message msg;

	msg.host = carmen_get_host();
	msg.timestamp = timestamp;
	msg.number_of_shots = number_of_cols;

	// double angle = -10.0; // vertical angle
	// double delta_angle = 49.5 / (double) msg.number_of_shots;
	// for (int i = 0; i < number_of_rows; i++){
	// 	// printf("%d ", i);
	// 	angle += delta_angle;
	// 	printf("%.3f ", angle);
	// }
	// for (int i = number_of_rows-1; i >=0 ; i--){
	// 	printf("%d ", i);
	// }

	double angle = -33.0; // Horizontal angle
	double delta_angle = 66.0 / (double) msg.number_of_shots;

	unsigned short int *points = (unsigned short int *) depth_pred;
	
	msg.partial_scan = (carmen_velodyne_shot *) malloc(msg.number_of_shots * sizeof(carmen_velodyne_shot));

	for (int i = 0; i < number_of_cols; i++){
		msg.partial_scan[i].angle = angle;
		angle += delta_angle;
		msg.partial_scan[i].shot_size = number_of_rows;
		msg.partial_scan[i].distance = (unsigned int *) malloc(msg.partial_scan[i].shot_size * sizeof(unsigned int));
		msg.partial_scan[i].intensity = (unsigned short int *) malloc(msg.partial_scan[i].shot_size * sizeof(unsigned short int));
	}


	//unsigned short int *points = (unsigned short int *) depth_pred;
	
	// for (int i = 0, k= number_of_cols-1; i < number_of_cols; i++, k--)
	// {
	// 	for (int j = 0, l= msg.partial_scan[i].shot_size-1; j < msg.partial_scan[i].shot_size; j++, l--)
	// 	{
	// 		double horizontal_angle = msg.partial_scan[i].angle * M_PI / 180.0;
	// 		//msg.partial_scan[i].distance[j] = points[i + j * number_of_cols] * ( 2 - cos(abs(horizontal_angle))); // * ( 2 - cos(abs(horizontal_angle))); // points[i + j * number_of_cols];
	// 		msg.partial_scan[k].distance[j] = ((unsigned int) imgdepth.at<unsigned short int>(j, k)) * ( 2 - cos(abs(horizontal_angle))); 
	// 		//msg.partial_scan[k].distance[j] = points[k + j * number_of_cols] * ( 2 - cos(abs(horizontal_angle)));
	// 		msg.partial_scan[i].intensity[j] = 100;
	// 	}
	// }
	// i < 640
	for (int i = 0, k= number_of_cols-1 ; i < number_of_cols; i++, k--)
	{
		for (int j = 0, l= msg.partial_scan[i].shot_size-1; j < msg.partial_scan[i].shot_size; j++, l--)
		{
			double horizontal_angle = msg.partial_scan[i].angle * M_PI / 180.0;
			msg.partial_scan[i].distance[j] = points[i + j * number_of_cols] * ( 2 - cos(abs(horizontal_angle))); // * ( 2 - cos(abs(horizontal_angle))); // points[i + j * number_of_cols];
			// msg.partial_scan[i].distance[j] =  (unsigned int) ((unsigned int) imgdepth.at<unsigned short int>(j, i)) * 50.0 / 256.0; // * ( 2 - cos(abs(horizontal_angle))); 
			
			//msg.partial_scan[k].distance[j] = ((unsigned int) imgdepth.at<unsigned short int>(i, j)) * ( 2 - cos(abs(horizontal_angle))); 
			//msg.partial_scan[k].distance[j] = points[k + j * number_of_cols] * ( 2 - cos(abs(horizontal_angle)));
			msg.partial_scan[i].intensity[j] = gray[(int)(i + j * number_of_cols)];;
		}
	}

	carmen_velodyne_publish_variable_scan_message(&msg, 8);

	for (int i = 0; i < msg.number_of_shots; i++)
	{
		free(msg.partial_scan[i].distance);
		free(msg.partial_scan[i].intensity);
	}
	free(msg.partial_scan);
}

void rotate(cv::Mat& src, double angle, cv::Mat& dst){
    cv::Point2f ptCp(src.cols*0.5, src.rows*0.5);
    cv::Mat M = cv::getRotationMatrix2D(ptCp, angle, 1.0);
    cv::warpAffine(src, dst, M, src.size(), cv::INTER_CUBIC); //Nearest is too rough, 
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
bumblebee_basic_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	Mat open_cv_image = Mat(stereo_image->height, stereo_image->width, CV_8UC3, stereo_image->raw_right, 0); // CV_32FC3 float 32 bit 3 channels (to char image use CV_8UC3)
	cv::Rect myROI(0, 190, stereo_image->width, stereo_image->height - 190);
	open_cv_image = open_cv_image(myROI);

	unsigned char *depth_pred = libadabins_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, stereo_image->timestamp);
    cv::Mat imgdepth = cv::Mat(open_cv_image.rows, open_cv_image.cols, CV_16U, depth_pred);
	//cv::Mat imgrotate;
	//rotate(imgdepth, 180, imgrotate);
	cv::Mat imggray;
	cv::cvtColor(open_cv_image, imggray, cv::COLOR_BGR2GRAY);
	unsigned char*image_gray = imggray.data;

	publish_point_cloud(depth_pred, open_cv_image.rows, open_cv_image.cols, stereo_image->timestamp, image_gray);

    

    cv::imshow("Image", open_cv_image);
	// cv::imshow("Rotate", imgrotate);
	cv::imshow("Depth Prediction Transformer", imgdepth);
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

	carmen_velodyne_shot *vector = (carmen_velodyne_shot *) malloc(horizontal_resolution_l * sizeof(carmen_velodyne_shot));
	carmen_test_alloc(vector);

	for (i = 0; i < horizontal_resolution_l; i++)
	{
		vector[i].distance = (unsigned int *) calloc(vertical_resolution_l, sizeof(unsigned int));
		carmen_test_alloc(vector[i].distance);
		vector[i].intensity = (unsigned short *) calloc(vertical_resolution_l, sizeof(unsigned short));
		carmen_test_alloc(vector[i].distance);
		vector[i].shot_size = vertical_resolution_l;
	}

	return (vector);
}


static void
init_stereo_velodyne()
{
	if (!flipped)
		scan = alloc_velodyne_shot_scan_vector(horizontal_resolution, vertical_resolution);
	else
		scan = alloc_velodyne_shot_scan_vector(vertical_resolution, horizontal_resolution);

	reference_image = cvCreateImage(cvSize(bumblebee_basic_width, bumblebee_basic_height), IPL_DEPTH_8U, 3);
	reference_image_gray = cvCreateImage(cvSize(bumblebee_basic_width, bumblebee_basic_height), IPL_DEPTH_8U, 1);
}


int
read_parameters(int argc, char **argv)
{
	int num_items;

	char stereo_velodyne_string[256];
	char stereo_string[256];
	char camera_string[256];

	sprintf(stereo_velodyne_string, "%s%d", "stereo_velodyne", atoi(argv[1]));
	sprintf(stereo_string, "%s%d", "stereo", atoi(argv[1]));
	sprintf(camera_string, "%s%d", "bumblebee_basic", atoi(argv[1]));

	carmen_param_t param_list[] = {
		{ (char *) stereo_velodyne_string, (char *) "vertical_resolution", CARMEN_PARAM_INT, &vertical_resolution, 1, NULL },
		{ (char *) stereo_velodyne_string, (char *) "horizontal_resolution", CARMEN_PARAM_INT, &horizontal_resolution, 1, NULL },
		{ (char *) stereo_velodyne_string, (char *) "range_max", CARMEN_PARAM_DOUBLE, &range_max, 1, NULL },
		{ (char *) stereo_velodyne_string, (char *) "flipped", CARMEN_PARAM_ONOFF, &flipped, 1, NULL },
		{ (char *) stereo_velodyne_string, (char *) "vertical_roi_ini", CARMEN_PARAM_INT, &vertical_roi_ini, 1, NULL },
		{ (char *) stereo_velodyne_string, (char *) "vertical_roi_end", CARMEN_PARAM_INT, &vertical_roi_end, 1, NULL },
		{ (char *) stereo_velodyne_string, (char *) "horizontal_roi_ini", CARMEN_PARAM_INT, &horizontal_roi_ini, 1, NULL },
		{ (char *) stereo_velodyne_string, (char *) "horizontal_roi_end", CARMEN_PARAM_INT, &horizontal_roi_end, 1, NULL },
		{ (char *) camera_string, (char *) "width", CARMEN_PARAM_INT, &bumblebee_basic_width, 1, NULL },
		{ (char *) camera_string, (char *) "height", CARMEN_PARAM_INT, &bumblebee_basic_height, 1, NULL }
	};

	if (vertical_resolution > vertical_roi_end - vertical_roi_ini)
		carmen_die("The stereo_velodyne_vertical_resolution is bigger than stereo point cloud height");

	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return (0);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	if ((argc != 2))
		carmen_die("%s: Wrong number of parameters. stereo requires either 1 or 2 parameters and received %d parameter(s). \nUsage:\n %s <camera_number>",
				argv[0], argc - 1, argv[0]);

	camera = atoi(argv[1]);
	read_parameters(argc, argv);

	instance = get_stereo_instance(camera, bumblebee_basic_width, bumblebee_basic_height);

	/* Register Python Context for adabins*/
	initialize_python_context();

	init_stereo_velodyne();

	carmen_velodyne_define_messages();

    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) bumblebee_basic_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();

	return 0;
}

