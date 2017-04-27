
#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include "opencv2/opencv.hpp"

using namespace cv;

#define BUMBLEBEE_ID 4

int video_id = 0;
int stop_required = 0;
int zed_height = 0;
int zed_width = 0;


void
carmen_bumblebee_publish_stereoimage_message(unsigned char *rawLeft, unsigned char *rawRight, int width, int height, int channels)
{
	carmen_bumblebee_basic_stereoimage_message stereo_msg;

	stereo_msg.timestamp = carmen_get_time();
    stereo_msg.host = carmen_get_host();
    stereo_msg.image_size = width * height * channels;
    stereo_msg.width = width;
    stereo_msg.isRectified = 1;
    stereo_msg.height = height;
    stereo_msg.raw_left = rawLeft;
    stereo_msg.raw_right = rawRight;

    carmen_bumblebee_basic_publish_message(BUMBLEBEE_ID, &stereo_msg);
}


void shutdown_module(int signo)
{
	if(signo == SIGINT)
	{
		stop_required = 1;
		printf("zed_camera_sensor: disconnected.\n");
		exit(0);
	}
}


static int read_parameters(int argc, char **argv)
{
	int num_items;
	char bb_name[64];

	sprintf(bb_name, "bumblebee_basic%d", BUMBLEBEE_ID);

	carmen_param_t param_list[] =
	{
		{(char*) "commandline", (char*) "video_id", CARMEN_PARAM_INT, &video_id, 0, NULL},
		{bb_name, (char*) "height", CARMEN_PARAM_INT, &zed_height, 0, NULL},
		{bb_name, (char*) "width", CARMEN_PARAM_INT, &zed_width, 0, NULL},
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}


void
show_debug_info(Mat &frame)
{
	static double last_time = 0;
	static int num_frames = 0;

	double current_time = carmen_get_time();
	double diff = current_time - last_time;

	if (diff > 1.0)
	{
		printf("time: %lf h: %d w: %d frame rate: %.2lf\n", current_time, frame.rows, frame.cols, (double) num_frames / diff);
		num_frames = 0;
		last_time = current_time;
	}
	else
		num_frames++;
}


void
view_frame(Mat &frame)
{
	Size s2(frame.cols / 2, frame.rows / 2);
	Mat zoom(s2, frame.type());
	resize(frame, zoom, zoom.size());
	imshow("frame", zoom);
	waitKey(1);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_module);
	read_parameters(argc, argv);

	carmen_bumblebee_basic_define_messages(BUMBLEBEE_ID);

    // TODO: a captura das imagens esta com latencia. Parece que o VideoCapture
    // tem algum buffer interno... Checar como desliga-lo.
    VideoCapture cap(video_id);

	if(!cap.isOpened())
	{
		printf("Failed to open /dev/video%d!\n", video_id);
		return -1;
	}

	cap.set(CV_CAP_PROP_FRAME_WIDTH, zed_width * 2);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, zed_height);

	while (!stop_required)
	{
        Mat frame;
        cap >> frame;

		Mat left, right, left_rgb, right_rgb;
		left = frame(Rect(0, 0, frame.cols / 2, frame.rows));
		right = frame(Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

		// TODO: checar se fazer a conversao de bgr para rgb nao eh lento. Se for lento,
		// encontrar outra forma de fazer a conversao.
		cvtColor(left, left_rgb, CV_BGR2RGB);
		cvtColor(right, right_rgb, CV_BGR2RGB);

		// DEBUG:
		// show_debug_info(frame);
		// view_frame(frame);

		carmen_bumblebee_publish_stereoimage_message(left_rgb.data, right_rgb.data, left_rgb.cols, left_rgb.rows, 3);
	}

	return 0;
}

