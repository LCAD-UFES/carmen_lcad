#include <carmen/carmen.h>
#include <carmen/global.h>
#include <carmen/stereo_util.h>
#include <carmen/stereo_velodyne_interface.h>
#include <carmen/stereo_messages.h>
#include <carmen/stereo_interface.h>
#include <carmen/stereo_velodyne.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string.h>
#include <carmen/libadabins.h>
#include <stdio.h>
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


void
convert_depth_to_velodyne_beams_old(stereo_util interface, cv::Mat depth, int vertical_resolution,
		int horizontal_resolution, carmen_velodyne_shot *stereo_velodyne_scan,
		unsigned short range_max, int vertical_roi_ini, int vertical_roi_end, int horizontal_roi_ini, int horizontal_roi_end, unsigned char *image)
{
	int i, j;
	double inc_vertical, inc_horizontal, x, y;
	carmen_vector_3D_p point;
	int width = horizontal_roi_end - horizontal_roi_ini;
	int height = vertical_roi_end - vertical_roi_ini;

	// inc_vertical = ((double) height / (double) vertical_resolution) + 0.5;
	// inc_horizontal =  ((double) width / (double) horizontal_resolution) + 0.5;

	// cout << inc_vertical << " inc_horizontal=" << inc_horizontal << endl; 1.5
	double angle = -25.0;
	double delta_angle = 50.0 / horizontal_resolution; //horizontal_angle


	for (j = horizontal_roi_ini ; j < horizontal_roi_end ; j++)
	{
		stereo_velodyne_scan[j].angle = angle;
		angle += delta_angle;
		//cout << "stereo_velodyne_scan[" << j << "].shot_size=" << stereo_velodyne_scan[j].shot_size << " angle=" << stereo_velodyne_scan[j].angle << " delta_angle=" << delta_angle << endl;
		// stereo_velodyne_scan[j].angle = carmen_radians_to_degrees(atan((x - (((double)interface.width) / 2.0)) / interface.fx));
		for (int i = vertical_roi_end+190; i >= 0; i--)
		{
			double horizontal_angle = stereo_velodyne_scan[j].angle * M_PI / 180.0;

			stereo_velodyne_scan[j].distance[i] = (unsigned short) 0.5; // * ( 2 - cos(abs(stereo_velodyne_scan[j].angle)));
			// stereo_velodyne_scan[j].distance[i] = (unsigned short) (((unsigned int) depth.at<unsigned short int>(i, j)) * ( 2 - cos(abs(stereo_velodyne_scan[j].angle))));
			stereo_velodyne_scan[j].intensity[i] = image[(int)(y * (double)interface.width + x)];
			//stereo_velodyne_scan[j].intensity[i] = (unsigned short) 100.0;
		}
	}
}
void
convert_depth_to_velodyne_beams(stereo_util interface, unsigned char* depth, int vertical_resolution,
		int horizontal_resolution, carmen_velodyne_shot *stereo_velodyne_scan,
		double range_max, int vertical_roi_ini, int vertical_roi_end, int horizontal_roi_ini, int horizontal_roi_end, unsigned char *image)
{
	int i, j;
	double inc_vertical, inc_horizontal, x, y;
	carmen_vector_3D_p point;
	int width = horizontal_roi_end - horizontal_roi_ini;
	int height = vertical_roi_end - vertical_roi_ini;

	inc_vertical = ((double) height / (double) vertical_resolution);
	inc_horizontal =  ((double) width / (double) horizontal_resolution);
	unsigned short int *disparity = (unsigned short int *) depth;

	for (x = horizontal_roi_ini, j = horizontal_resolution/interface.stereo_stride_x - 1; x <  horizontal_roi_end; x += interface.stereo_stride_x*inc_horizontal, j--)
	{
		stereo_velodyne_scan[j].angle = carmen_radians_to_degrees(atan((x - (((double)interface.width) / 2.0)) / interface.fx));

		for (y = vertical_roi_ini, i = vertical_resolution/interface.stereo_stride_y - 1; y < vertical_roi_end; y += interface.stereo_stride_y*inc_vertical, i--)
		{
			carmen_position_t p;
			p.x = x;
			p.y = y;
			point = reproject_single_point_to_3D(&interface, p, disparity[(int)(y * interface.width + x)]);

			if (point != NULL)
			{
				double range = sqrt(DOT3D((*point), (*point)));
				range = range > range_max ? 0.0 : range;
				cout << range << endl;

				stereo_velodyne_scan[j].distance[i] = (unsigned short) (range);
				stereo_velodyne_scan[j].intensity[i] = image[(int)(y * (double)interface.width + x)];
				//stereo_velodyne_scan[j].intensity[i] = image[(int)(y + x * interface.width)];

				free(point);
				point = NULL;
			}
			else
			{
				//stereo_velodyne_scan[j].distance[i] = range_max * 500.0;
				stereo_velodyne_scan[j].distance[i] = 0.0;
			}
		}
	}
}


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


void
bumblebee_basic_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	Mat open_cv_image = Mat(stereo_image->height, stereo_image->width, CV_8UC3, stereo_image->raw_right, 0); // CV_32FC3 float 32 bit 3 channels (to char image use CV_8UC3)
	cv::Rect myROI(0, 190, stereo_image->width, stereo_image->height - 190);
	cv::Mat imggray;
	cv::cvtColor(open_cv_image, imggray, cv::COLOR_BGR2GRAY);
	unsigned char*image_gray = imggray.data;
	open_cv_image = open_cv_image(myROI);

	unsigned char *depth_pred = libadabins_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, stereo_image->timestamp);
    cv::Mat imgdepth = cv::Mat(open_cv_image.rows, open_cv_image.cols, CV_16U, depth_pred);
	// transpose(imgdepth, imgdepth);  
    // flip(imgdepth, imgdepth,1);
	// transpose(imgdepth, imgdepth);  
    // flip(imgdepth, imgdepth,1);

	// unsigned short int *points = (unsigned short int *) depth_pred;
	// cout << "Depth: rows= " << imgdepth.rows << " cols=" << imgdepth.cols << endl;

	
	// cout << "GRAY: rows= " << imggray.rows << " cols=" << imggray.cols << endl; rows290 x cols640


	cv::imshow("Image", open_cv_image);
	cv::imshow("Gray", imggray);
	cv::imshow("Depth Prediction Transformer", imgdepth*256);
	waitKey(1);


	convert_depth_to_velodyne_beams(instance, depth_pred, vertical_resolution, horizontal_resolution, scan, range_max, vertical_roi_ini,
				vertical_roi_end, horizontal_roi_ini, horizontal_roi_end, image_gray);

	//convert_depth_map_to_velodyne_beams(imgdepth, vertical_resolution, horizontal_resolution, scan, image_gray);

	velodyne_partial_scan.partial_scan = scan;
	velodyne_partial_scan.number_of_shots = horizontal_roi_end - horizontal_roi_ini;
	velodyne_partial_scan.host = carmen_get_host();
	velodyne_partial_scan.timestamp = stereo_image->timestamp;

	publish_point_cloud();

	// scan->angle = 0.0;
	// memset(scan->distance, 0, scan->shot_size * sizeof(unsigned short));
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
	
	init_stereo_velodyne();
	
	/* Register Python Context for adabins*/
	initialize_python_context();

	instance = get_stereo_instance(camera, bumblebee_basic_width, bumblebee_basic_height);
	
	carmen_stereo_velodyne_define_messages(camera);

	carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) bumblebee_basic_handler, CARMEN_SUBSCRIBE_LATEST);
	
	// carmen_stereo_subscribe(camera, NULL, (carmen_handler_t) carmen_simple_stereo_disparity_message_handler, CARMEN_SUBSCRIBE_LATEST);
	
	carmen_ipc_dispatch();

	return 0;
}

