#include <carmen/carmen.h>
#include <carmen/web_cam_interface.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

//#define __CARMEN_WEB_CAM_VIEW_DEBUG_INFO

int web_cam_width = 640;
int web_cam_height = 480;

#ifdef __CARMEN_WEB_CAM_VIEW_DEBUG_INFO

	double timestamp_message_last_second = 99999;
	int num_images = 0;

#endif

void
carmen_publish_web_cam_message (IplImage *img)
{
	IPC_RETURN_TYPE err;

	carmen_web_cam_message message;

	message.width = img->width;
	message.height = img->height;
	message.image_size = img->height * img->widthStep;
	message.img_data = img->imageData;
	message.timestamp = carmen_get_time();
	message.host = carmen_get_host();

	err = IPC_publishData (CARMEN_WEB_CAM_MESSAGE_NAME, &message);
	carmen_test_ipc_exit (err, "Could not publish", CARMEN_WEB_CAM_MESSAGE_FMT);

#ifdef __CARMEN_WEB_CAM_VIEW_DEBUG_INFO

	num_images++;

	if (message.timestamp - timestamp_message_last_second > 1)
	{
		printf("num images per second: %d\n", num_images);
		timestamp_message_last_second = message.timestamp;
		num_images = 0;
	}

	printf("\tPublishing image %dx%d\n", message.width, message.height);

#endif

}

void
carmen_web_cam_shutdown_module (int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect ();
		printf ("web_cam_module: disconnected.\n");
		exit (0);
	}
}

void
carmen_read_web_cam_parameters (int argc, char **argv)
{
	int num_items;

	carmen_param_t param_list[] =
	{
			{"web_cam", "width", CARMEN_PARAM_INT, &web_cam_width, 0, NULL},
			{"web_cam", "height", CARMEN_PARAM_INT, &web_cam_height, 0, NULL},
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}

void
carmen_web_cam_capture_loop (void)
{
	CvCapture* capture = cvCaptureFromCAM (CV_CAP_ANY);

	if (!capture)
		exit(printf ("ERROR: capture is NULL \n"));

	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, web_cam_width);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, web_cam_height);

	while (1)
	{
		IplImage* frame = cvQueryFrame(capture);

		if (!frame)
			exit(printf("ERROR: frame is null...\n"));

		carmen_publish_web_cam_message (frame);

		// *************************
		// Do not release the frame!
		// *************************
	}

	cvReleaseCapture(&capture);
}

int
main (int argc, char **argv)
{
	carmen_ipc_initialize (argc, argv);

	signal (SIGINT, carmen_web_cam_shutdown_module);

	// read_parameters(argc, argv);

	carmen_web_cam_define_messages ();
	carmen_web_cam_capture_loop ();

	carmen_ipc_disconnect();
	return 0;
}
