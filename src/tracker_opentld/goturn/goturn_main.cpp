#include <stdio.h>
#include <iostream>
#include <vector>
#include <list>
#include <string>

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/bumblebee_basic_messages.h>
#include <carmen/velodyne_interface.h>
#include <tf.h>

#include <carmen/visual_tracker_interface.h>
#include <carmen/visual_tracker_messages.h>
#include <carmen/velodyne_camera_calibration.h>

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

std::vector<carmen_velodyne_points_in_cam_t> points_lasers_in_cam;

carmen_velodyne_partial_scan_message *velodyne_message_arrange;

static int tld_image_width = 0;
static int tld_image_height = 0;

const float MAX_RANGE = 30.0;
const float MIN_RANGE = 0.5;

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

bounding_box box_1;

double average_box_confidence;
static std::string window_name = "GOTURN";


using namespace cv;

struct Image_Box{
	cv::Mat prev_image;
	BoundingBox prev_box;
	double confidence;
};

int num_prev_frames = 2;

std::vector<Image_Box> last_track;

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
			message_output.confidence = average_box_confidence;
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
calculate_box_average(BoundingBox &box, std::vector<Image_Box> &last_track, Mat* img, int step, int cont)
{
	BoundingBox average_box;
	int repeat = 0;
	average_box.x1_ = 0.0;
	average_box.x2_ = 0.0;
	average_box.y1_ = 0.0;
	average_box.y2_ = 0.0;

	average_box.x1_ += box.x1_;
	average_box.x2_ += box.x2_;
	average_box.y1_ += box.y1_;
	average_box.y2_ += box.y2_;
	int limit = step/last_track.size();

//	for(int i = cont%last_track.size(); repeat < limit; i = (i + step)%last_track.size(), repeat++)
	for(int i = 0; i < last_track.size(); i++)
	{
		tracker.Init(last_track[i].prev_image, last_track[i].prev_box, &regressor);

		tracker.Track(*img, &regressor, &last_track[i].prev_box);

		average_box.x1_ = last_track[i].prev_box.x1_;
		average_box.x2_ = last_track[i].prev_box.x2_;
		average_box.y1_ = last_track[i].prev_box.y1_;
		average_box.y2_ = last_track[i].prev_box.y2_;
		rectangle(*img, cv::Point(average_box.x1_, average_box.y1_),cv::Point(average_box.x2_ , average_box.y2_), Scalar( 0, 255, 0 ), 1, 4 );
//		average_box.DrawBoundingBox(img);
		last_track[(cont%num_prev_frames)].confidence = 1.0; //
	}

//		printf("ok\n");
//	average_box.x1_ /= 3;
//	average_box.x2_ /= 3;
//	average_box.y1_ /= 3;
//	average_box.y2_ /= 3;

//	box = average_box;

}

void
process_goturn_detection(Mat *img, double time_stamp)
{
	char string1[128];
	CvFont font;
	bool recovery = false;
	static int cont = 0;
	static int first = 0;
	int step = 30;


	//cv::Mat mat_image=cvarrToMat(&img);
	static Mat prev_image;
	prev_image = img->clone();
	if(box.x1_ != -1.0)
	{

		tracker.Track(*img, &regressor, &box);
		if (cont > num_prev_frames)
		{
//			calculate_box_average(box, last_track, img, step, cont);
//			recovery = true;

		}
		//Apenas para publicar-ainda falta definir
//		average_box_confidence = 1.0;

		last_track[(cont%num_prev_frames)].prev_image = prev_image;
		last_track[(cont%num_prev_frames)].prev_box = box;

		box_1.x = box.x1_;
		box_1.y = box.y1_;
		box_1.width = box.get_width();
		box_1.height = box.get_height();


		last_track[(cont%num_prev_frames)].confidence = 1.0;
//		points_lasers_in_cam = carmen_velodyne_camera_calibration_lasers_points_bounding_box(velodyne_message,
//				&last_message, &last_track[(cont%num_prev_frames)].confidence, &box_1);

		points_lasers_in_cam = carmen_velodyne_camera_calibration_lasers_points_in_camera(velodyne_message_arrange,&last_message);

//		points_lasers_in_cam = carmen_velodyne_camera_calibration_lasers_points_bounding_box(velodyne_message_arrange, &last_message,
//				&last_track[(cont%num_prev_frames)].confidence, &box_1);

		cv::Rect mini_box;
		mini_box.x = box_1.x + (box_1.width/3);
		mini_box.y = box_1.y + (box_1.height/3);
		mini_box.width = (box_1.width/3);
		mini_box.height = (box_1.height/3);
		Scalar yellow = CV_RGB(255, 255, 0);
		Scalar white = CV_RGB(255, 255, 255);

		rectangle(*img, cv::Point(mini_box.x, mini_box.y),cv::Point(mini_box.x + mini_box.width, mini_box.y + mini_box.height), yellow, 1, 4 );
		for(int i = 0; i < points_lasers_in_cam.size(); i++)
			{
				if((points_lasers_in_cam.at(i).ipx > box_1.x) && ( points_lasers_in_cam.at(i).ipx < (box_1.x + box_1.width)) &&
						(points_lasers_in_cam.at(i).ipy > box_1.y) && (points_lasers_in_cam.at(i).ipy < (box_1.y + box_1.height)))
				{
					if (points_lasers_in_cam.at(i).laser_polar.length <= MIN_RANGE)
						points_lasers_in_cam.at(i).laser_polar.length = MAX_RANGE;

					if (points_lasers_in_cam.at(i).laser_polar.length > MAX_RANGE)
						points_lasers_in_cam.at(i).laser_polar.length = MAX_RANGE;


					circle(*img, cv::Point(points_lasers_in_cam.at(i).ipx, points_lasers_in_cam.at(i).ipy), 2, Scalar(0, 255, 0), -1);

					if((points_lasers_in_cam.at(i).ipx > mini_box.x) && ( points_lasers_in_cam.at(i).ipx < (mini_box.x + mini_box.width)) &&
							(points_lasers_in_cam.at(i).ipy > mini_box.y) && (points_lasers_in_cam.at(i).ipy < (mini_box.y + mini_box.height)))
					{
		//				ranges_d.push_back(points_lasers_in_cam.at(i).laser_polar.length);
						circle(*img, cv::Point(points_lasers_in_cam.at(i).ipx, points_lasers_in_cam.at(i).ipy), 2, white, -1);
		//				average_range += points_lasers_in_cam.at(i).laser_polar.length;
					}

					//printf("%f\t %f\t %f\t \n",r, range, velodyne_message->partial_scan[j].distance[i] );

				}

			}

		last_track[(cont%num_prev_frames)].confidence = 1.0; //
		cont++;
		tracker.Init(*img, box, &regressor);

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
		last_track[(cont%num_prev_frames)].prev_image = prev_image;
		last_track[(cont%num_prev_frames)].prev_box = box;
		cont++;
		tracker.Init(*img, box, &regressor);
//		first = 1;
//		tracker.Track(*img, &regressor, &box);
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
//		double time_f = carmen_get_time() - time_now;
//		printf("tp: %lf \n", time_f);
	}

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
	velodyne_message_arrange = velodyne_message;
	carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(velodyne_message_arrange);
//	show_velodyne(velodyne_message);
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
	last_track.resize(num_prev_frames);

	box.x1_ = -1.0;
	box.y1_ = -1.0;
	box.x2_ = -1.0;
	box.y2_ = -1.0;

	signal(SIGINT, shutdown_camera_view);


	carmen_visual_tracker_define_messages();

	carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_velodyne_subscribe_partial_scan_message(NULL,(carmen_handler_t)velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);


	carmen_ipc_dispatch();

}
