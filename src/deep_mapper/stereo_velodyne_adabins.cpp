#include <carmen/carmen.h>
#include <carmen/global.h>
#include <carmen/stereo_util.h>
#include <carmen/stereo_velodyne_interface.h>
#include <carmen/stereo_messages.h>
#include <carmen/stereo_interface.h>
#include <carmen/libadabins.h>
#include <carmen/stereo_velodyne.h>
#include <carmen/velodyne_camera_calibration.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string.h>

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
carmen_simple_stereo_disparity_message disparity_message;

static IplImage *reference_image;
static IplImage *reference_image_gray;

#define CAM_DELAY 0.25
#define MAX_POSITIONS 10

#define IMAGE_HEIGHT_CROP 0.82

#define MAX_CAMERA_INDEX	9
#define NUM_CAMERA_IMAGES	5
int camera_alive[MAX_CAMERA_INDEX + 1];
carmen_camera_parameters camera_params[MAX_CAMERA_INDEX + 1];
carmen_pose_3D_t camera_pose[MAX_CAMERA_INDEX + 1];
int active_cameras;

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void publish_point_cloud()
{
	carmen_stereo_velodyne_publish_message(camera, &velodyne_partial_scan);
}

void
publish_stereo_message()
{
	carmen_stereo_publish_message(camera, &disparity_message);
}

///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
void
carmen_bumblebee_basic_stereoimage_message_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	/*ADABINS*/
	// unsigned char *depth_pred;
	unsigned char *depth_pred;
	Mat open_cv_image = Mat(stereo_image->height, stereo_image->width, CV_8UC3, stereo_image->raw_right, 0);              // CV_32FC3 float 32 bit 3 channels (to char image use CV_8UC3)
	depth_pred = libadabins_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, stereo_image->timestamp);
	cv::Mat imgdepth = cv::Mat(stereo_image->height, stereo_image->width, CV_16U, depth_pred);
	for (int i=0; i<stereo_image->width; i++){
		for(int j=0; j<stereo_image->height; j++){
			disparity_message.disparity[i * stereo_image->width + j] = (float) depth_pred[i * stereo_image->width + j];		
		}
	}
	
	imshow("Depth Prediction Transformer", imgdepth);
	/*End of AdaBins*/
	disparity_message.reference_image_size = stereo_image->image_size;

	disparity_message.reference_image = stereo_image->raw_right;
	disparity_message.timestamp = stereo_image->timestamp;
	
	publish_stereo_message();
}


void
bumblebee_basic_image_handler(int camera, carmen_bumblebee_basic_stereoimage_message *msg)
{
	// unsigned char *img;
	
	// if (camera_alive[camera] == 1)
	// 	img = msg->raw_right;
	// else
	// 	img = msg->raw_left;

	Mat open_cv_image = Mat(msg->height, msg->width, CV_8UC3, msg->raw_right, 0);              // CV_32FC3 float 32 bit 3 channels (to char image use CV_8UC3)
	/*ADABINS*/
	// unsigned char *depth_pred;
	unsigned char *depth_pred;
	depth_pred = libadabins_process_image(open_cv_image.cols, open_cv_image.rows, open_cv_image.data, msg->timestamp);
	cv::Mat imgdepth = cv::Mat(msg->height, msg->width, CV_16U, depth_pred);
	for (int i=0; i<msg->width; i++){
		for(int j=0; j<msg->height; j++){
			disparity_message.disparity[i * msg->width + j] = (float) depth_pred[i * msg->width + j];		
		}
	}
	
	imshow("Depth Prediction Transformer", imgdepth);
	waitKey(1);
	/*End of AdaBins*/
	// disparity_message.reference_image_size = msg->image_size;

	// disparity_message.reference_image = msg->raw_right;
	// disparity_message.timestamp = msg->timestamp;
	
	// publish_stereo_message();
}


void
bumblebee_basic1_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(1, image_msg);
}


void
bumblebee_basic2_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(2, image_msg);
}


void
bumblebee_basic3_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(3, image_msg);
}


void
bumblebee_basic4_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(4, image_msg);
}


void
bumblebee_basic5_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(5, image_msg);
}


void
bumblebee_basic6_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(6, image_msg);
}


void
bumblebee_basic7_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(7, image_msg);
}


void
bumblebee_basic8_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(8, image_msg);
}


void
bumblebee_basic9_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(9, image_msg);
}

void
bumblebee_basic10_image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	bumblebee_basic_image_handler(10, image_msg);
}

void (*image_handler[]) (carmen_bumblebee_basic_stereoimage_message *) =
{
		NULL,
		bumblebee_basic1_image_handler,
		bumblebee_basic2_image_handler,
		bumblebee_basic3_image_handler,
		bumblebee_basic4_image_handler,
		bumblebee_basic5_image_handler,
		bumblebee_basic6_image_handler,
		bumblebee_basic7_image_handler,
		bumblebee_basic8_image_handler,
		bumblebee_basic9_image_handler,
		bumblebee_basic10_image_handler,
};


static void
carmen_simple_stereo_disparity_message_handler(carmen_simple_stereo_disparity_message *message)
{
	memcpy(reference_image->imageData, message->reference_image, message->reference_image_size);

	cvCvtColor(reference_image, reference_image, CV_RGB2BGR);

	cvCvtColor(reference_image, reference_image_gray, CV_BGR2GRAY);

	

	if (!flipped)
	{
		convert_stereo_depth_map_to_velodyne_beams(instance, message->disparity, vertical_resolution, horizontal_resolution, scan, range_max, vertical_roi_ini,
												   vertical_roi_end, horizontal_roi_ini, horizontal_roi_end, (unsigned char *)reference_image_gray->imageData);

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
	if (!flipped)
		scan = alloc_velodyne_shot_scan_vector(horizontal_resolution, vertical_resolution);
	else
		scan = alloc_velodyne_shot_scan_vector(vertical_resolution, horizontal_resolution);

	reference_image = cvCreateImage(cvSize(bumblebee_basic_width, bumblebee_basic_height), IPL_DEPTH_8U, 3);
	reference_image_gray = cvCreateImage(cvSize(bumblebee_basic_width, bumblebee_basic_height), IPL_DEPTH_8U, 1);
}

static void
get_camera_param(int argc, char **argv, int camera)
{
	char bumblebee_name[256];
	char camera_name[256];

	if (camera_alive[camera] >= 0)
	{
		sprintf(bumblebee_name, "bumblebee_basic%d", camera);
		sprintf(camera_name, "camera%d", camera);

		carmen_param_t param_list[] =
		{
			{bumblebee_name, (char*) "fx",         CARMEN_PARAM_DOUBLE, &camera_params[camera].fx_factor,       0, NULL },
			{bumblebee_name, (char*) "fy",         CARMEN_PARAM_DOUBLE, &camera_params[camera].fy_factor,       0, NULL },
			{bumblebee_name, (char*) "cu",         CARMEN_PARAM_DOUBLE, &camera_params[camera].cu_factor,       0, NULL },
			{bumblebee_name, (char*) "cv",         CARMEN_PARAM_DOUBLE, &camera_params[camera].cv_factor,       0, NULL },
			{bumblebee_name, (char*) "pixel_size", CARMEN_PARAM_DOUBLE, &camera_params[camera].pixel_size,      0, NULL },
			{bumblebee_name, (char*) "fov",        CARMEN_PARAM_DOUBLE, &camera_params[camera].fov,             0, NULL },
			{bumblebee_name, (char*) "tlight_dist_correction", CARMEN_PARAM_DOUBLE, &camera_params[camera].focal_length, 0, NULL },

			{camera_name,    (char*) "x",          CARMEN_PARAM_DOUBLE, &camera_pose[camera].position.x,        0, NULL },
			{camera_name,    (char*) "y",          CARMEN_PARAM_DOUBLE, &camera_pose[camera].position.y,        0, NULL },
			{camera_name,    (char*) "z",          CARMEN_PARAM_DOUBLE, &camera_pose[camera].position.z,        0, NULL },
			{camera_name,    (char*) "roll",       CARMEN_PARAM_DOUBLE, &camera_pose[camera].orientation.roll,  0, NULL },
			{camera_name,    (char*) "pitch",      CARMEN_PARAM_DOUBLE, &camera_pose[camera].orientation.pitch, 0, NULL },
			{camera_name,    (char*) "yaw",        CARMEN_PARAM_DOUBLE, &camera_pose[camera].orientation.yaw,   0, NULL },
		};

		carmen_param_allow_unfound_variables(0);
		carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));

		// camera_msg_count[camera] = 0;
		// camera_filter_count[camera] = 0;
		// camera_datmo_count[camera] = 0;
	}
}

static void
read_camera_parameters(int argc, char **argv)
{
	char *camera_side[MAX_CAMERA_INDEX + 1] = {NULL};

	carmen_param_t camera_param_list[] =
	{
		{(char *) "commandline", (char *) "camera1", CARMEN_PARAM_STRING, &camera_side[1], 0, NULL},
		{(char *) "commandline", (char *) "camera2", CARMEN_PARAM_STRING, &camera_side[2], 0, NULL},
		{(char *) "commandline", (char *) "camera3", CARMEN_PARAM_STRING, &camera_side[3], 0, NULL},
		{(char *) "commandline", (char *) "camera4", CARMEN_PARAM_STRING, &camera_side[4], 0, NULL},
		{(char *) "commandline", (char *) "camera5", CARMEN_PARAM_STRING, &camera_side[5], 0, NULL},
		{(char *) "commandline", (char *) "camera6", CARMEN_PARAM_STRING, &camera_side[6], 0, NULL},
		{(char *) "commandline", (char *) "camera7", CARMEN_PARAM_STRING, &camera_side[7], 0, NULL},
		{(char *) "commandline", (char *) "camera8", CARMEN_PARAM_STRING, &camera_side[8], 0, NULL},
		{(char *) "commandline", (char *) "camera9", CARMEN_PARAM_STRING, &camera_side[9], 0, NULL},
	};

	carmen_param_allow_unfound_variables(1);
	carmen_param_install_params(argc, argv, camera_param_list, sizeof(camera_param_list) / sizeof(camera_param_list[0]));

	active_cameras = 0;
	for (int i = 1; i <= MAX_CAMERA_INDEX; i++)
	{
		camera_alive[i] = -1;
		if (camera_side[i] == NULL)
			continue;

		if (strcmp(camera_side[i], "left") == 0 || strcmp(camera_side[i], "0") == 0)
			camera_alive[i] = 0;
		else if (strcmp(camera_side[i], "right") == 0 || strcmp(camera_side[i], "1") == 0)
			camera_alive[i] = 1;
		else
			carmen_die("-camera%d %s: Wrong camera side option. Must be either left or right\n", i, camera_side[i]);

		active_cameras++;
		get_camera_param(argc, argv, i);
	}
	if (active_cameras == 0)
		fprintf(stderr, "No cameras active for stereo_velodyne_adabins\n\n");
	
}

int read_parameters(int argc, char **argv)
{
	read_camera_parameters(argc, argv);

	int num_items;

		if ((argc < 3))
		carmen_die("%s: Wrong number of parameters. virtual_depth requires min 2 parameter and received %d. \n Usage: %s -camera<camera_number> <camera_side(left or right)>\n", argv[0], argc - 1, argv[0]);


	camera = atoi(argv[1]);

	

	char stereo_velodyne_string[256];
	char stereo_string[256];
	char camera_string[256];

	sprintf(stereo_velodyne_string, "%s%d", "stereo_velodyne", atoi(argv[1]));
	sprintf(stereo_string, "%s%d", "stereo", atoi(argv[1]));
	sprintf(camera_string, "%s%d", "bumblebee_basic", atoi(argv[1]));

	carmen_param_t param_list[] = {
		{(char *)stereo_velodyne_string, (char *)"vertical_resolution", CARMEN_PARAM_INT, &vertical_resolution, 1, NULL},
		{(char *)stereo_velodyne_string, (char *)"horizontal_resolution", CARMEN_PARAM_INT, &horizontal_resolution, 1, NULL},
		{(char *)stereo_velodyne_string, (char *)"range_max", CARMEN_PARAM_DOUBLE, &range_max, 1, NULL},
		{(char *)stereo_velodyne_string, (char *)"flipped", CARMEN_PARAM_ONOFF, &flipped, 1, NULL},
		{(char *)stereo_velodyne_string, (char *)"vertical_roi_ini", CARMEN_PARAM_INT, &vertical_roi_ini, 1, NULL},
		{(char *)stereo_velodyne_string, (char *)"vertical_roi_end", CARMEN_PARAM_INT, &vertical_roi_end, 1, NULL},
		{(char *)stereo_velodyne_string, (char *)"horizontal_roi_ini", CARMEN_PARAM_INT, &horizontal_roi_ini, 1, NULL},
		{(char *)stereo_velodyne_string, (char *)"horizontal_roi_end", CARMEN_PARAM_INT, &horizontal_roi_end, 1, NULL},
		{(char *)camera_string, (char *)"width", CARMEN_PARAM_INT, &bumblebee_basic_width, 1, NULL},
		{(char *)camera_string, (char *)"height", CARMEN_PARAM_INT, &bumblebee_basic_height, 1, NULL}};

	if (vertical_resolution > vertical_roi_end - vertical_roi_ini)
		carmen_die("The stereo_velodyne_vertical_resolution is bigger than stereo point cloud height");

	num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	instance = get_stereo_instance(camera, bumblebee_basic_width, bumblebee_basic_height);

	init_stereo_velodyne();
	carmen_stereo_velodyne_define_messages(camera);

	carmen_param_check_version(argv[0]);

	return (0);
}

static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		fprintf(stderr, "Virtual Depth: disconnected.\n");

		exit(0);
	}
}

void subscribe_messages()
{
	carmen_stereo_subscribe(camera, NULL, (carmen_handler_t)carmen_simple_stereo_disparity_message_handler, CARMEN_SUBSCRIBE_LATEST);

	for (int camera = 1; camera <= MAX_CAMERA_INDEX; camera++)
	{
		if (camera_alive[camera] >= 0)
			carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler[camera], CARMEN_SUBSCRIBE_LATEST);
	}

}

int main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	read_parameters(argc, argv);

	/* Register Python Context for adabins*/
	initialize_python_context();

	subscribe_messages();

	signal(SIGINT, shutdown_module);

	
	setlocale(LC_ALL, "C");

	carmen_ipc_dispatch();

	return 0;
}
