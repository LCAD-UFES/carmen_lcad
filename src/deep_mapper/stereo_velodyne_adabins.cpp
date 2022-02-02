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
static double horizontal_camera_angle;
static double horizontal_start_angle;
static double range_multiplier_factor;
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

void
convert_depth_to_velodyne_beams(stereo_util interface, unsigned char* depth, int vertical_resolution,
		int horizontal_resolution, carmen_velodyne_shot *stereo_velodyne_scan,
		double range_max, int vertical_roi_ini, int vertical_roi_end, int horizontal_roi_ini, int horizontal_roi_end, unsigned char *image)
{
	int i, j;
	double inc_vertical, inc_horizontal, x, y;
	int width = horizontal_roi_end - horizontal_roi_ini;
	int height = vertical_roi_end - vertical_roi_ini;

	inc_vertical = ((double) height / (double) vertical_resolution);
	inc_horizontal =  ((double) width / (double) horizontal_resolution);
	unsigned short int *points = (unsigned short int *) depth;

	double angle = horizontal_start_angle; // Horizontal angle
	double delta_angle = horizontal_camera_angle / (double) width;
	for (x = horizontal_roi_ini, j = horizontal_resolution/interface.stereo_stride_x - 1; x <  horizontal_roi_end; x += interface.stereo_stride_x*inc_horizontal, j--)
	{
		stereo_velodyne_scan[j].angle = angle;
		angle += delta_angle;
		for (y = vertical_roi_ini, i = vertical_resolution/interface.stereo_stride_y - 1; y < vertical_roi_end; y += interface.stereo_stride_y*inc_vertical, i--)
		{			
			double horizontal_angle = stereo_velodyne_scan[j].angle * M_PI / 180.0;
			double range = points[(int)(y * (double)interface.width + x)] * ( 2 - cos(abs(horizontal_angle)));
			range = range > range_max ? 0.0 : range;
			stereo_velodyne_scan[j].distance[i] = (unsigned short) (range * range_multiplier_factor);
			stereo_velodyne_scan[j].intensity[i] = image[(int)(y * (double)interface.width + x)];
		}
	}
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
	cv::Mat imggray;
	cv::cvtColor(open_cv_image, imggray, cv::COLOR_BGR2GRAY);
	unsigned char*image_gray = imggray.data;
	unsigned char *depth_pred = libadabins_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, stereo_image->timestamp);

	cv::Rect myROI(0, 200, stereo_image->width, stereo_image->height - 200);
	open_cv_image = open_cv_image(myROI);
    cv::Mat imgdepth = cv::Mat(open_cv_image.rows, open_cv_image.cols, CV_16U, depth_pred);
		
	convert_depth_to_velodyne_beams(instance, depth_pred, vertical_resolution, horizontal_resolution, scan, range_max, vertical_roi_ini,
				vertical_roi_end, horizontal_roi_ini, horizontal_roi_end, image_gray);
	
    velodyne_partial_scan.partial_scan = scan;
	velodyne_partial_scan.number_of_shots = horizontal_roi_end - horizontal_roi_ini;
	velodyne_partial_scan.host = carmen_get_host();
	velodyne_partial_scan.timestamp = stereo_image->timestamp;
	carmen_velodyne_publish_variable_scan_message(&velodyne_partial_scan, 8);
	
    cv::imshow("Image", open_cv_image);
	cv::imshow("Adabins", imgdepth * 256);
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
	scan = alloc_velodyne_shot_scan_vector(horizontal_resolution, vertical_resolution);
	
	reference_image = cvCreateImage(cvSize(bumblebee_basic_width, bumblebee_basic_height), IPL_DEPTH_8U, 3);
	reference_image_gray = cvCreateImage(cvSize(bumblebee_basic_width, bumblebee_basic_height), IPL_DEPTH_8U, 1);
}


int
read_parameters(int argc, char **argv)
{
	int num_items;

	char stereo_string[256];
	
	sprintf(stereo_string, "%s%d", "stereo", atoi(argv[1]));

	carmen_param_t param_list[] = {
		{ (char *) stereo_string, (char *) "vertical_resolution", CARMEN_PARAM_INT, &vertical_resolution, 1, NULL },
		{ (char *) stereo_string, (char *) "horizontal_resolution", CARMEN_PARAM_INT, &horizontal_resolution, 1, NULL },
		{ (char *) stereo_string, (char *) "range_max", CARMEN_PARAM_DOUBLE, &range_max, 1, NULL },
		{ (char *) stereo_string, (char *) "vertical_roi_ini", CARMEN_PARAM_INT, &vertical_roi_ini, 1, NULL },
		{ (char *) stereo_string, (char *) "vertical_roi_end", CARMEN_PARAM_INT, &vertical_roi_end, 1, NULL },
		{ (char *) stereo_string, (char *) "horizontal_roi_ini", CARMEN_PARAM_INT, &horizontal_roi_ini, 1, NULL },
		{ (char *) stereo_string, (char *) "horizontal_roi_end", CARMEN_PARAM_INT, &horizontal_roi_end, 1, NULL },
		{ (char *) stereo_string, (char *) "width", CARMEN_PARAM_INT, &bumblebee_basic_width, 1, NULL },
		{ (char *) stereo_string, (char *) "height", CARMEN_PARAM_INT, &bumblebee_basic_height, 1, NULL },
		{ (char *) stereo_string, (char *) "horizontal_camera_angle", CARMEN_PARAM_DOUBLE, &horizontal_camera_angle, 1, NULL },
		{ (char *) stereo_string, (char *) "horizontal_start_angle", CARMEN_PARAM_DOUBLE, &horizontal_start_angle, 1, NULL },
		{ (char *) stereo_string, (char *) "range_multiplier_factor", CARMEN_PARAM_DOUBLE, &range_multiplier_factor, 1, NULL }
		
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

