#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/bumblebee_basic_messages.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/legacy/legacy.hpp>

using namespace cv;
using namespace std;

static int image_number;
static int camera;
static int image_width;
static int image_height;

//Image
static IplImage *src;

carmen_localize_ackerman_globalpos_message localize_message;

/**
 * Reading parameters of initialize
 * @param argc argc of terminal
 * @param argv argv of terminal
 * @return success 
 */
static int
read_parameters(int argc, char **argv)
{
    int num_items;
    char bumblebee_string[256];

    if (argc == 2)
        camera = atoi(argv[1]);
    else
    {
        printf("Usage: %s %s", argv[0], argv[1]);
        exit(0);
    }

    sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera);

    carmen_param_t param_list[] = {
        { bumblebee_string, (char*) "width", CARMEN_PARAM_INT, &image_width, 0,
            NULL},
        { bumblebee_string, (char*) "height", CARMEN_PARAM_INT, &image_height, 0,
            NULL}
    };

    num_items = sizeof (param_list) / sizeof (param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);

    return 0;
}

/**
 * Method for compute the state of traffic light
 * @param stereo_image
 */
void
compute_traffic_light(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
    src = cvCreateImage(cvSize(image_width, image_height), IPL_DEPTH_8U, 3);

    //Copying left image of message 
    memcpy(src->imageData, stereo_image->raw_left, stereo_image->image_size);
    cvCvtColor(src, src, CV_BGR2RGB);

    image_number++;
    char image_name[100] = "";
    sprintf(image_name, "/dados/log/image_%04d_%lf_%lf_%lf.png", image_number, stereo_image->timestamp, localize_message.globalpos.x, localize_message.globalpos.y);
    cout << "Image Number " << image_name << endl;
    cvSaveImage(image_name, src);


    cvReleaseImage(&src);
}

void
localize_ackerman_handler(carmen_localize_ackerman_globalpos_message *msg)
{
    localize_message = *msg;
}

/**
 * Main method for mapping, detecting and inferring the traffic light state
 * @param stereo_image Message of stereo camera
 */
void
traffic_light_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
    compute_traffic_light(stereo_image);
}

/**
 * Method for subscribe messages of localize and mapping of traffic light
 */
void
subscribe_camera_mapping_traffic_light_messages()
{
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) traffic_light_handler, CARMEN_SUBSCRIBE_LATEST);
    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_handler, CARMEN_SUBSCRIBE_LATEST);
}

/**
 * Main of program
 * @param argc
 * @param argv
 * @return 
 */
int
main(int argc, char **argv)
{
    /* connect to IPC server */

    carmen_ipc_initialize(argc, argv);

    carmen_param_check_version(argv[0]);

    if ((argc != 2))
        carmen_die(
                   "%s: Wrong number of parameters. Traffic Light requires 1 parameters and received %d parameter(s). \nUsage:\n %s <camera_number>\n",
                   argv[0], argc - 1, argv[0]);

    camera = atoi(argv[1]);

    read_parameters(argc, argv);



    /* Subscribe messages of camera and mapping of traffic light*/
    subscribe_camera_mapping_traffic_light_messages();



    /* Loop forever waiting for messages */
    carmen_ipc_dispatch();

    return EXIT_SUCCESS;
}
