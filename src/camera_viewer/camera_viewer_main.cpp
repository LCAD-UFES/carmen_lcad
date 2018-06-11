/*********************************************************
	---  Camera Viewer Module ---
 **********************************************************/

#include "camera_viewer.h"

using namespace std;
using namespace cv;

static int camera;    // Number of the camera that will be used



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////



void
bumblebee_basic_image_handler(carmen_bumblebee_basic_stereoimage_message *bumblebee_basic_message)
{

    Mat image = Mat(bumblebee_basic_message->height, bumblebee_basic_message->width, CV_8UC3, bumblebee_basic_message->raw_right);

    cvtColor(image, image, COLOR_BGR2RGB);

    static bool first_time = true;

    if(first_time)
    {
        first_time = false;
        namedWindow("Camera Viewer", WINDOW_AUTOSIZE);
    }


    imshow("camera_viewer", image);
    waitKey(1);
}

void
camera_image_handler(carmen_camera_image_message *image_msg)
{
	Mat open_cv_image = Mat(image_msg->height, image_msg->width, CV_8UC3, image_msg->image, 3 * 640);

	imshow("Camera Viewer", open_cv_image);
	waitKey(1);
}



///////////////////////////////////////////////////////////////////////////////////////////////
//																						     //
// Initializations																		     //
//																						     //
///////////////////////////////////////////////////////////////////////////////////////////////



void
subscribe_ipc_messages()
{
    //carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) bumblebee_basic_image_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_camera_subscribe_images(NULL, (carmen_handler_t)camera_image_handler, CARMEN_SUBSCRIBE_LATEST);
}


void
read_parameters(int argc, char** argv)
{
    if (argc != 2)
        carmen_die("Wrong number of parameters. This module requires 1 parameter and received %d parameter(s). \nUsage: %s <camera_number>", argc - 1, argv[0]);

    camera = atoi(argv[1]);     // Defining the camera to be used
}


int
main(int argc, char **argv)
{
    read_parameters(argc, argv);

    carmen_ipc_initialize(argc, argv);

    carmen_param_check_version(argv[0]);

    subscribe_ipc_messages();

    carmen_ipc_dispatch();

    return 0;
}
