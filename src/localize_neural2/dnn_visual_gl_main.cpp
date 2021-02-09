
/**
 * @description
 * DNN Global Localizer
 *
 * @author Alberto F. De Souza
 */

#include <stdio.h>
#include <string.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>

#include <carmen/carmen.h>
#include <carmen/gps_nmea_interface.h>
#include <carmen/fused_odometry_interface.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/web_cam_interface.h>
#include <carmen/stereo_interface.h>
#include <carmen/base_ackerman_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/xsens_interface.h>

#include "network.h"
#include "parser.h"

using namespace std;
using namespace cv;

static int camera;
static int bumblebee_basic_width;
static int bumblebee_basic_height;

network net;
char **learned_poses;


image
convert_image_msg_to_darknet_image(unsigned int w, unsigned int h, unsigned char *data)
{
	unsigned int c = 3;    // Number of channels
    image image = make_image(w, h, c);

    if (data == NULL)
    	return image;

    for (unsigned int k = 0; k < c; ++k)
    {
    	for (unsigned int j = 0; j < h; ++j)
    	{
    		for (unsigned int i = 0; i < w; ++i)
    		{
    			int dst_index = i + (w * j) + (w * h * k);
    			int src_index = k + (c * i) + (c * w * j);
    			image.data[dst_index] = (float) (data[src_index] / 255.0);    // 255 because of conversion Uchar (byte) ti float
    		}
    	}
    }

    return (image);
}
///////////////////////////////////////////////////////////////////////////////////////////////


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
bumblebee_basic_handler(carmen_bumblebee_basic_stereoimage_message *stereo_image)
{
	image img = convert_image_msg_to_darknet_image(stereo_image->width, stereo_image->height, stereo_image->raw_right);
//	image img_without_car_hood = crop_image(img, 320, 240, 640, 380);
//	image resized_img = resize_min(img_without_car_hood, net.w);
	image resized_img = resize_min(img, net.w);
	image cropped_img = crop_image(resized_img, (resized_img.w - net.w) / 2, (resized_img.h - net.h) / 2, net.w, net.h);

	float *predictions = network_predict(net, cropped_img.data);

	int selected_pose_label;
	top_k(predictions, net.outputs, 1, &selected_pose_label);

	double x, y, theta;
	char predicted_image_file_name[2048];
	sscanf(learned_poses[selected_pose_label], "%lf %lf %lf %s", &x, &y, &theta, predicted_image_file_name);
	printf("confidence %lf, %lf %lf %lf %s\n", predictions[selected_pose_label], x, y, theta, predicted_image_file_name);
//	printf("confidence %lf, %s\n", predictions[selected_pose_label], learned_poses[selected_pose_label]);

   	Mat pose_image = imread(predicted_image_file_name, IMREAD_COLOR);
    imshow("dnn_visual_gl", pose_image);
    waitKey(1);


//    free_image(img_without_car_hood);
	free_image(resized_img);
	free_image(cropped_img);
}


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("log_filter: disconnected\n");
		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
read_parameters(int argc, char **argv)
{
	char bumblebee_string[256];
	char camera_string[256];

	carmen_param_t param_cmd_list[] =
	{
		{(char *) "commandline", (char *) "camera_id", CARMEN_PARAM_INT, &camera, 0, NULL},
	};

	carmen_param_install_params(argc, argv, param_cmd_list, sizeof(param_cmd_list)/sizeof(param_cmd_list[0]));

	sprintf(camera_string, "%s%d", "camera", camera);
	sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera);

	carmen_param_t param_list[] =
	{
		{bumblebee_string, (char *) "width", CARMEN_PARAM_INT, &bumblebee_basic_width, 0, NULL},
		{bumblebee_string, (char *) "height", CARMEN_PARAM_INT, &bumblebee_basic_height, 0, NULL},
	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list)/sizeof(param_list[0]));
}


void
initialize_structures(char *cfgfile, char *weightfile, char *learned_poses_filename)
{
    net = parse_network_cfg_custom(cfgfile, 1, 0);
    if (weightfile)
        load_weights(&net, weightfile);

    set_batch_network(&net, 1);
    srand(2222222);

    fuse_conv_batchnorm(net);
    calculate_binary_weights(net);

    learned_poses = get_labels(learned_poses_filename);
}


void
subscribe_messages()
{
	carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) bumblebee_basic_handler, CARMEN_SUBSCRIBE_LATEST);
}
///////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc, char *argv[])
{
	signal(SIGINT, shutdown_module);
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	initialize_structures(argv[1], argv[2], argv[3]);

	read_parameters(argc, argv);

	subscribe_messages();
	carmen_ipc_dispatch();

	return (0);
}

