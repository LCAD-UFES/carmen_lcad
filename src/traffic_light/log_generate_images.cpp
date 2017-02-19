#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/bumblebee_basic_messages.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#if CV_MAJOR_VERSION == 2
#include <opencv2/legacy/legacy.hpp>
#elif CV_MAJOR_VERSION == 3
#include <opencv/cv.h>
#endif

using namespace cv;
using namespace std;

static int image_number;
static int camera;
static int image_width;
static int image_height;

static double images_per_second;

//Image
static IplImage *left_image;

carmen_localize_ackerman_globalpos_message localize_message;
bool have_localization = false;


void
save_images(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	static double last_timestamp = 0.0;

	if (have_localization && (stereo_image->timestamp - last_timestamp) > (1.0 / images_per_second))
	{
		left_image = cvCreateImage(cvSize(image_width, image_height), IPL_DEPTH_8U, 3);

		//Copying left image of message
		memcpy(left_image->imageData, stereo_image->raw_left, stereo_image->image_size);
		cvCvtColor(left_image, left_image, CV_BGR2RGB);

		image_number++;
		char image_name[100] = "";
		sprintf(image_name, "/dados/log/image_%04d_%lf_%lf_%lf.png",
				image_number, stereo_image->timestamp, localize_message.globalpos.x, localize_message.globalpos.y);
		cout << "Image Number " << image_name << endl;
		cvSaveImage(image_name, left_image);

		cvReleaseImage(&left_image);

		last_timestamp = stereo_image->timestamp;
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

// none...

///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *msg)
{
    localize_message = *msg;

    have_localization = true;
}


void
carmen_bumblebee_basic_stereoimage_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
    save_images(stereo_image);
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
subscribe_camera_mapping_traffic_light_messages()
{
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) carmen_bumblebee_basic_stereoimage_handler, CARMEN_SUBSCRIBE_LATEST);
    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) carmen_localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


static void
read_parameters(int argc, char **argv)
{
    int num_items;
    char bumblebee_string[256];

    sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera);

    carmen_param_t param_list[] =
    {
        { bumblebee_string, (char*) "width", CARMEN_PARAM_INT, &image_width, 0, NULL},
        { bumblebee_string, (char*) "height", CARMEN_PARAM_INT, &image_height, 0, NULL}
    };

    num_items = sizeof (param_list) / sizeof (param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);
}


int
main(int argc, char **argv)
{
    carmen_ipc_initialize(argc, argv);
    carmen_param_check_version(argv[0]);

    if (argc != 3)
        carmen_die("%s: Wrong number of parameters. Traffic Light requires 2 parameters and received %d parameter(s)."
        		   "\nUsage:\n %s <camera_number> <images_per_second>\n", argv[0], argc - 1, argv[0]);

    camera = atoi(argv[1]);
    images_per_second = atof(argv[2]);

    read_parameters(argc, argv);
    subscribe_camera_mapping_traffic_light_messages();

    carmen_ipc_dispatch();

    return (EXIT_SUCCESS);
}
