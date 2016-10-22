#include <stdio.h>
#include <iostream>
#include <vector>
#include <list>
#include <string>

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/bumblebee_basic_messages.h>

#include <carmen/visual_tracker_interface.h>
#include <carmen/visual_tracker_messages.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv/cv.h>
//#include <opencv/highgui.h>

//Goturn_tracker
#include "tracker/tracker.h"
#include "regressor/regressor_train.h"
#include "gui.h"
//using namespace std;

//
//#define TACKER_OPENTLD_MAX_WINDOW_WIDTH 1280
//#define TACKER_OPENTLD_MAX_WINDOW_HEIGHT 960
#define BUMBLEBEE_BASIC_VIEW_NUM_COLORS 3

static int received_image = 0;

static int tld_image_width = 0;
static int tld_image_height = 0;

static int camera_side = 0;

static carmen_bumblebee_basic_stereoimage_message last_message;

static int msg_fps = 0, msg_last_fps = 0; //message fps
static int disp_fps = 0, disp_last_fps = 0; //display fps

static carmen_visual_tracker_output_message message_output;

const double fontScale = 2.0;
const int thickness = 3;

//Goturn_tracker
std::string model_file = "tracker.prototxt";
std::string trained_file = "tracker.caffemodel";
int gpu_id = 0;
//Goturn things
Regressor regressor(model_file, trained_file, gpu_id, false);

// Ensuring randomness for fairness.
//	srandom (time(NULL));

const bool show_intermediate_output = false;
// Create a tracker object.
Tracker tracker(show_intermediate_output);

static BoundingBox box;
static std::string window_name = "GOTURN";

using namespace cv;



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_visual_tracker_output_message()
{
	IPC_RETURN_TYPE err;
	err = IPC_publishData(CARMEN_VISUAL_TRACKER_OUTPUT_MESSAGE_NAME, &message_output);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_VISUAL_TRACKER_OUTPUT_MESSAGE_NAME);
}

void
carmen_visual_tracker_define_messages()
{
	carmen_visual_tracker_define_message_output();
}

void
build_and_publish_message(char *host, double timestamp)
{
	bounding_box box_detected;
		if (box.x1_ != -1.0)
		{
			box_detected.x = box.x1_;
			box_detected.y = box.y1_;
			box_detected.width = box.get_width();
			box_detected.height = box.get_height();

			message_output.rect = box_detected;
			message_output.confidence = 1.0;
			message_output.host = host;
			message_output.timestamp = timestamp;
		}

		else
		{
			box_detected.x = -1;
			box_detected.y = -1;
			box_detected.width = -1;
			box_detected.height = -1;

			message_output.rect = box_detected;
			message_output.confidence = 0.0;
			message_output.host = host;
			message_output.timestamp = timestamp;
		}

	//fprintf(stderr, "%lf %lf\n", message_output.timestamp, message_output.confidence);

	publish_visual_tracker_output_message();
}

///////////////////////////////////////////////////////////////////////////////////////////////


void
process_goturn_detection(Mat *img, double time_stamp)
{
	char string1[128];
	CvFont font;

	//cv::Mat mat_image=cvarrToMat(&img);
	//cv::Mat prev_image;
	//prev_image = mat_image.clone();
	if(box.x1_ != -1.0)
	{
		tracker.Track(*img, &regressor, &box);
		box.DrawBoundingBox(img);
	}

	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, .4, .5, 0, 1, 8);
//	cvRectangle(&prev_image, cvPoint(0, 0), cvPoint(img.width, 50), CV_RGB(0, 0, 255), CV_FILLED, 8, 0);
//	cvPutText(&prev_image.data, string1, cvPoint(25, 25), &font, CV_RGB(255, 255, 255));
	// draw the box
	sprintf(string1, "Time:%.2f, FPS:%d", time_stamp, disp_last_fps);
	cv::Size s = img->size();
	cv::Point textOrg(25,25);
	cv::rectangle(*img, cv::Point(0, 0), cv::Point(s.width, 50), Scalar::all(0), -1);
	cv::putText(*img, string1,textOrg,FONT_HERSHEY_SIMPLEX, 0.4,Scalar::all(255),1,8);

	cv::imshow(window_name, *img);

	char c = cv::waitKey(2);

	switch (c)
	{
	case 'r':

		CvRect rect;

		if (getBBFromUser(img, rect, window_name) == 0)
		{
			return;
		}

		box.x1_ = rect.x;
		box.y1_ = rect.y;
		box.x2_ = rect.x+rect.width;
		box.y2_ = rect.y+rect.height;
		tracker.Init(*img, box, &regressor);
		break;

	case 'q':
		exit(0);
	}
		// Track and estimate the bounding box location.

}


static void
process_image(carmen_bumblebee_basic_stereoimage_message *msg)
{
	static Mat *src_image = NULL;
	static Mat *rgb_image = NULL;
	static Mat *resized_rgb_image = NULL;

	if (src_image == NULL)
	{
		src_image = new Mat(Size(msg->width, msg->height), CV_8UC3);
		rgb_image = new Mat(Size(msg->width, msg->height), CV_8UC3);
		resized_rgb_image = new Mat(Size(tld_image_width , tld_image_height), CV_8UC3);
	}

	if (camera_side == 0)
	{
		//src_image->imageData = (char*) msg->raw_left;
		memcpy(src_image->data, msg->raw_left, msg->image_size * sizeof(char));
	}
	else
	{
		//src_image->imageData = (char*) msg->raw_right;
		memcpy(src_image->data, msg->raw_right, msg->image_size * sizeof(char));
	}

	cvtColor(*src_image, *rgb_image, cv::COLOR_RGB2BGR);

	if (tld_image_width == msg->width && tld_image_height == msg->height)
		process_goturn_detection(rgb_image, msg->timestamp);
	else
	{
//		cv::resize(*rgb_image, *resized_rgb_image, Size(tld_image_width,tld_image_height));
		process_goturn_detection(rgb_image, msg->timestamp);
	}


	//cvReleaseImage(&rgb_image);
	//cvReleaseImage(&resized_rgb_image);
	//cvReleaseImage(&src_image);
}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
static void
image_handler(carmen_bumblebee_basic_stereoimage_message* image_msg)
{
	static double last_timestamp = 0.0;
	static double last_time = 0.0;
	double time_now = carmen_get_time();

	//Just process Rectified images
	if (image_msg->isRectified)
	{

		if (!received_image)
		{
			received_image = 1;
			last_timestamp = image_msg->timestamp;
			last_time = time_now;
		}


		if ((image_msg->timestamp - last_timestamp) > 1.0)
		{
			msg_last_fps = msg_fps;
			msg_fps = 0;
			last_timestamp = image_msg->timestamp;
		}
		msg_fps++;

		if ((time_now - last_time) > 1.0)
		{

			disp_last_fps = disp_fps;
			disp_fps = 0;
			last_time = time_now;
		}
		disp_fps++;


		last_message = *image_msg;
		process_image(image_msg);

		build_and_publish_message(image_msg->host, image_msg->timestamp);
		double time_f = carmen_get_time() - time_now;
		printf("tp: %lf \n", time_f);
	}

}

///////////////////////////////////////////////////////////////////////////////////////////////

static void
shutdown_camera_view(int x)
{
	if (x == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("Disconnected from robot.\n");
		exit(0);
	}
}

//
static int
read_parameters(int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] = {
			{(char*) "tracker_opentld", (char*) "view_width", CARMEN_PARAM_INT, &tld_image_width, 0, NULL},
			{(char*) "tracker_opentld", (char*) "view_height", CARMEN_PARAM_INT, &tld_image_height, 0, NULL},

	};

	num_items = sizeof (param_list) / sizeof (param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}

int
main(int argc, char **argv)
{

	int camera = 0;

	if (argc != 3)
	{
		fprintf(stderr, "%s: Wrong number of parameters. tracker_opentld requires 2 parameter and received %d. \n Usage: %s <camera_number> <camera_side(0-left; 1-right)\n>", argv[0], argc - 1, argv[0]);
		exit(1);
	}

	camera = atoi(argv[1]);
	camera_side = atoi(argv[2]);

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	read_parameters(argc, argv);

	box.x1_ = -1.0;
	box.y1_ = -1.0;
	box.x2_ = -1.0;
	box.y2_ = -1.0;

//-----

	signal(SIGINT, shutdown_camera_view);


	carmen_visual_tracker_define_messages();

	carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();

}
