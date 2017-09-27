#include <carmen/carmen.h>
#include <semantic_segmentation_interface.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace std;
using namespace cv;

bool control = false;
carmen_bumblebee_basic_stereoimage_message *image = NULL;


static void
image_handler(carmen_bumblebee_basic_stereoimage_message* msg)
{
	// Show bumblebee image
    if(image == NULL || control)
    {
        image = msg;
        control = false;
    }
}


static void
segmented_image_handler(carmen_semantic_segmentation_image_message *msg)
{
	// Show segmented image
    if(msg->timestamp == image->timestamp)
    {
        control = true;

        Mat imageMat = Mat(image->height, image->width, CV_8UC3, image->raw_left);
        Mat segmentedImageMat = Mat(msg->height, msg->width, CV_8UC3, msg->image);

        imshow("Image", imageMat);
        waitKey(1);

        imshow("Segmented Image", segmentedImageMat);
        waitKey(1);
    }
}


int
main(int argc, char **argv)
{
    carmen_semantic_segmentation_image_message *message = NULL;

    carmen_ipc_initialize(argc, argv);

    signal(SIGINT, shutdown_camera_view);

    //Create windows for images
    cvNamedWindow("Image");
    cvNamedWindow("Segmented Image");

	// Subscribe to bumblebee
    carmen_bumblebee_basic_subscribe_stereoimage(CAMERA_NUMBER, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);

	// Subscribe to semantic segmentation module
	carmen_semantic_segmentation_subscribe_image_message(message, (carmen_handler_t) segmented_image_handler, CARMEN_SUBSCRIBE_ALL);

    carmen_ipc_dispatch();

    return 0;
}
