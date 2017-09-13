/*********************************************************
	---  Camera Viewer Module ---
 **********************************************************/

#include "camera_viewer.h"

/* number of the camera that will be used */
static int camera;

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////





///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
bumblebee_basic_image_handler(carmen_bumblebee_basic_stereoimage_message *bumblebee_basic_message)
{

    cv::Mat image = cv::Mat(bumblebee_basic_message->height, bumblebee_basic_message->width, CV_8UC3,
                                 bumblebee_basic_message->raw_right);

    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

    static bool first_time = true;

    if(first_time)
    {
        first_time = false;
        cv::namedWindow("camera_viewer", cv::WINDOW_AUTOSIZE);
    }


    cv::imshow("camera_viewer", image);
    cv::waitKey(5);



}

////////////////////////////////////////////////////////////////////////////////////////////////

void
subscribe_ipc_messages()
{
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL,
                                                 (carmen_handler_t) bumblebee_basic_image_handler,
                                                 CARMEN_SUBSCRIBE_LATEST);
}


void
read_parameters(int argc, char** argv)
{
    if ((argc != 2))
        carmen_die("%s: Wrong number of parameters. This module requires 1 parameter and received %d parameter(s). \nUsage:\n %s <camera_number>",
                   argv[0], argc - 1, argv[0]);

    /* defining the camera to be used */
    camera = atoi(argv[1]);
}

int
main(int argc, char **argv)
{
    /* Read the parameters from command line and .ini if necessary */
    read_parameters(argc, argv);

    /* Connect to IPC server */
    carmen_ipc_initialize(argc, argv);

    /* Check the param server version */
    carmen_param_check_version(argv[0]);

    subscribe_ipc_messages();

    /* Loop forever waiting for messages */
    carmen_ipc_dispatch();

    return 0;
}
