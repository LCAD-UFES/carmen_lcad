#include <carmen/carmen.h>
#include <carmen/web_cam_interface.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

void
carmen_web_cam_test_shutdown_module (int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect ();
		cvDestroyWindow("img");

		printf ("web_cam_test_module: disconnected.\n");
		exit (0);
	}
}

void
carmen_web_cam_handler (carmen_web_cam_message *message)
{
	IplImage *img = cvCreateImageHeader (cvSize(message->width, message->height), IPL_DEPTH_8U, 3);
	img->imageData = message->img_data;

	cvShowImage("img", img);
	cvWaitKey(5);

	cvReleaseImageHeader(&img);
}

int
main (int argc, char **argv)
{
	carmen_ipc_initialize (argc, argv);
	signal (SIGINT, carmen_web_cam_test_shutdown_module);

	cvNamedWindow("img", CV_WINDOW_AUTOSIZE);

	carmen_web_cam_subscribe_message(NULL,
			(carmen_handler_t) carmen_web_cam_handler,
			CARMEN_SUBSCRIBE_LATEST);

	carmen_ipc_dispatch();
	return 0;
}
