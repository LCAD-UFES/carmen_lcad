
#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include "opencv2/opencv.hpp"

using namespace cv;

#define CAMERA_ID 4

int stop_required = 0;


void
carmen_bumblebee_publish_stereoimage_message(unsigned char *rawLeft, unsigned char *rawRight, int width, int height, int channels)
{
	carmen_bumblebee_basic_stereoimage_message stereo_msg;

    stereo_msg.host = carmen_get_host();
    stereo_msg.image_size = width * height * channels;
    stereo_msg.width = width;
    stereo_msg.isRectified = 1;
    stereo_msg.height = height;
    stereo_msg.raw_left = rawLeft;
    stereo_msg.raw_right = rawRight;

    carmen_bumblebee_basic_publish_message(CAMERA_ID, &stereo_msg);
}


void shutdown_module(int signo)
{
	if(signo == SIGINT)
	{
		printf("zed_camera_sensor: disconnected.\n");
		stop_required = 1;
		carmen_ipc_disconnect();
		exit(0);
	}
}


//static int read_parameters(int argc, char **argv)
//{
////	int num_items;
////
////	carmen_param_t param_list[] =
////	{
////      {"zed_camera_sensor", (char*)"quality", CARMEN_PARAM_INT, &zed_camera_sensor_quality, 0, NULL},
////      {"zed_camera_sensor", (char*)"fps", CARMEN_PARAM_DOUBLE, &zed_camera_sensor_fps, 0, NULL}
////	};
////
////	num_items = sizeof(param_list)/sizeof(param_list[0]);
////	carmen_param_install_params(argc, argv, param_list, num_items);
//
//	return 0;
//}


int
main(int argc, char **argv)
{
	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	// carmen_param_check_version(argv[0]);

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);

	/* Read application specific parameters (Optional) */
	// read_parameters(argc, argv);

	/* Define published messages by your module */
	carmen_bumblebee_basic_define_messages(CAMERA_ID);

	// TODO: find out the camera number automatically
    VideoCapture cap(1); // open the default camera

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 3840);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);

    if(!cap.isOpened())  // check if we succeeded
        return -1;

    // TODO: a captura das imagens esta com latencia. Parece que o VideoCapture tem algum buffer
    // interno... Checar como desliga-lo.

	while (!stop_required)
	{
        Mat frame;
        cap >> frame;

		Mat left, right, left_rgb, right_rgb;
		left = frame(Rect(0, 0, frame.cols / 2, frame.rows));
		right = frame(Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

	    // TODO: Estou dando resize nas imagens porque o viewer da bumblebee
		// nao funciona com imagens que nao sao proporcionais a 1280x960. Melhorar
		// o viewer e parar de fazer o resize.
		Size s(1280, 960);
		Mat zoom_left(s, left.type()), zoom_right(s, left.type());
		resize(left, zoom_left, zoom_left.size());
		resize(right, zoom_right, zoom_right.size());

		// TODO: checar se fazer a conversao de bgr para rgb nao eh lento. Se for lento,
		// encontrar outra forma de fazer a conversao.
		cvtColor(zoom_left, left_rgb, CV_BGR2RGB);
		cvtColor(zoom_right, right_rgb, CV_BGR2RGB);

		// TODO: desligar esta visualizacao
		Size s2(frame.cols / 4, frame.rows / 4);
		Mat zoom(s2, frame.type());
		resize(frame, zoom, zoom.size());
		imshow("left", zoom);
		waitKey(1);

		carmen_bumblebee_publish_stereoimage_message(left_rgb.data, right_rgb.data, left_rgb.cols, left_rgb.rows, 3);
	}

	return 0;
}

