
#include <carmen/carmen.h>
#include <semantic_segmentation_interface.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;


static void
image_handler(carmen_bumblebee_basic_stereoimage_message* msg)
{
	// Show bumblebee image
}

static void
segmented_image_handler(carmen_semantic_segmentation_image_message *msg)
{
	// Show segmented image
}


int
main(int argc, char **argv)
{
	carmen_semantic_segmentation_image_message *message;

    // Just to open an initial window
    for (int i = 0; i < 10; i++)
    {
		imshow(window_name, Mat::zeros(Size(BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH * 2, 480), CV_8UC3));
		waitKey(2);
    }

    carmen_ipc_initialize(argc, argv);

    signal(SIGINT, shutdown_camera_view);

	// Subscribe to bumblebee
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);

	// Subscribe to semantic segmentation module
	carmen_semantic_segmentation_subscribe_image_message(message, (carmen_handler_t) segmented_image_handler, CARMEN_SUBSCRIBE_ALL);

    carmen_ipc_dispatch();

    return 0;
}
