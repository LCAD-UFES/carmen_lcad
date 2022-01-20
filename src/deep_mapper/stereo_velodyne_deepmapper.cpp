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
static carmen_velodyne_variable_scan_message velodyne_partial_scan;

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
publish_point_cloud()
{
	carmen_stereo_velodyne_publish_message(camera, &velodyne_partial_scan);
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

static void
carmen_simple_stereo_disparity_message_handler(carmen_simple_stereo_disparity_message *message)
{
	memcpy(reference_image->imageData, message->reference_image, message->reference_image_size);
    
    
    // Mat open_cv_image = Mat(480, 640, CV_8UC3, message->reference_image, 0);              // CV_32FC3 float 32 bit 3 channels (to char image use CV_8UC3)
	// /*ADABINS*/
	// // unsigned char *depth_pred;
	// unsigned char *depth_pred;
	// depth_pred = libadabins_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, message->timestamp);

    // cv::Mat imgdepth = cv::Mat(open_cv_image.cols, open_cv_image.rows, CV_16U, depth_pred);
	// for (int i=0; i<open_cv_image.rows; i++){
	// 	for(int j=0; j<open_cv_image.cols; j++){
	// 		message->disparity[i * open_cv_image.rows + j] = (float) imgdepth.at<uint16_t>(j, i) / 256.0;		
	// 	}
	// }
	
	// imshow("Depth Prediction Transformer", imgdepth);
	// waitKey(1);


	cvCvtColor(reference_image, reference_image, CV_RGB2BGR);

//	copy_one_channel(reference_image, reference_image_gray, 0);
//	cvEqualizeHist(reference_image_gray, reference_image_gray);
//	copy_channel(reference_image_gray, reference_image, 0);
//
//	copy_one_channel(reference_image, reference_image_gray, 1);
//	cvEqualizeHist(reference_image_gray, reference_image_gray);
//	copy_channel(reference_image_gray, reference_image, 1);
//
//	copy_one_channel(reference_image, reference_image_gray, 2);
//	cvEqualizeHist(reference_image_gray, reference_image_gray);
//	copy_channel(reference_image_gray, reference_image, 2);

	cvCvtColor(reference_image, reference_image_gray, CV_BGR2GRAY);
	//cvSmooth(reference_image_gray,reference_image_gray,CV_BLUR, 0,0, 21, 3);

//	cvShowImage("1", reference_image);
//	cvWaitKey(33);
//    cvShowImage("2", reference_image_gray);
//    cvWaitKey(33);

	if (!flipped)
	{
		convert_stereo_depth_map_to_velodyne_beams(instance, message->disparity, vertical_resolution, horizontal_resolution, scan, range_max, vertical_roi_ini,
				vertical_roi_end, horizontal_roi_ini, horizontal_roi_end, (unsigned char *) reference_image_gray->imageData);

		velodyne_partial_scan.number_of_shots = horizontal_resolution;
	}
	else
	{
		convert_stereo_depth_map_to_velodyne_beams_and_flip(instance, message->disparity, vertical_resolution, horizontal_resolution, scan, range_max,
				vertical_roi_ini, vertical_roi_end, horizontal_roi_ini, horizontal_roi_end);

		velodyne_partial_scan.number_of_shots = vertical_resolution;
	}

	velodyne_partial_scan.partial_scan = scan;
	velodyne_partial_scan.host = carmen_get_host();
	velodyne_partial_scan.timestamp = message->timestamp;

	publish_point_cloud();

	scan->angle = 0.0;
	memset(scan->distance, 0, scan->shot_size * sizeof(unsigned short));
}


void
bumblebee_basic_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	Mat open_cv_image = Mat(stereo_image->height, stereo_image->width, CV_8UC3, stereo_image->raw_right, 0);              // CV_32FC3 float 32 bit 3 channels (to char image use CV_8UC3)
	/*ADABINS*/
	unsigned char *depth_pred;
	depth_pred = libadabins_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, stereo_image->timestamp);
    
    cv::Mat imgdepth = cv::Mat(open_cv_image.rows, open_cv_image.cols, CV_16U, depth_pred);
    cv::imshow("Image", open_cv_image);
	cv::imshow("Depth Prediction Transformer", imgdepth);
	waitKey(1);

	// publicar o mapa de profundidade
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
	carmen_stereo_velodyne_define_messages(camera);

    carmen_stereo_subscribe(camera, NULL, (carmen_handler_t) carmen_simple_stereo_disparity_message_handler, CARMEN_SUBSCRIBE_LATEST);
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t)bumblebee_basic_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();

	return 0;
}

