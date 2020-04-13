
#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_camera_calibration.h>
//#include <carmen/moving_objects_messages.h>
//#include <carmen/moving_objects_interface.h>
//#include <carmen/traffic_light_interface.h>
//#include <carmen/traffic_light_messages.h>
//#include <carmen/rddf_messages.h>
//#include <carmen/laser_ldmrs_utils.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include "libefficientdet.h"

using namespace std;
using namespace cv;


int camera;
int camera_side;
int width, height;

static const cv::Vec3b
colormap_semantic[] =
{
    cv::Vec3b(0, 0, 0), //0:unlabeled
	cv::Vec3b(0, 199, 0), // 1: car 142,0,0
	cv::Vec3b(200, 40, 255), // 2: bicycle
	cv::Vec3b(90, 30, 150), // 3: motorcycle
	cv::Vec3b(70, 0, 0), //4: truck
    cv::Vec3b(100, 80, 0), //5: other-vehicle
    cv::Vec3b(60, 20, 220), //6: person
    cv::Vec3b(200, 40, 255), //7: bicyclist
    cv::Vec3b(90, 30, 150), //8: motorcyclist
    cv::Vec3b(128, 64, 128), //9:road
    cv::Vec3b(128, 64, 128),//10: parking
    cv::Vec3b(75, 0, 75),//11: sidewalk
    cv::Vec3b(75, 0, 175),//12: other-ground
    cv::Vec3b(70, 70, 70), //13: building
    cv::Vec3b(50, 120, 255), //14: fence
    cv::Vec3b(35, 142, 107),//15: vegetation
    cv::Vec3b(35, 142, 107),//16: trunk
    cv::Vec3b(80, 240, 150),//17: terrain
	cv::Vec3b(153, 153, 153),//18: pole
	cv::Vec3b(0, 220, 220), //19: traffic-sign
};

vector<bbox_t>
filter_predictions_of_interest_efficientdet(vector<bbox_t> &predictions)
{
	vector<bbox_t> filtered_predictions;

	for (unsigned int i = 0; i < predictions.size(); i++)
	{
		if (predictions[i].obj_id > 0 && predictions[i].obj_id <= 9)    
		{
			filtered_predictions.push_back(predictions[i]);
		}
	}
	return (filtered_predictions);
}

void
show_detections(cv::Mat image, vector<bbox_t> predictions)
{
    for (unsigned int i = 0; i < predictions.size(); i++)
    {
    	int color_mapped = 0;
    	switch (predictions[i].obj_id)
    	{
    	case 1: //person
    		color_mapped = 6;
    		break;
    	case 2: //bicycle
    		color_mapped = 2;
    		break;
    	case 3: //car
    		color_mapped = 1;
    		break;
    	case 4: //motorbike
    		color_mapped = 3;
    		break;
        case 5: //airplane
    		color_mapped = 5;
    		break;
    	case 6: //bus
    		color_mapped = 5;
    		break;
    	case 7: //train
    		color_mapped = 5;
    		break;
    	case 8: //truck
    		color_mapped = 4;
    		break;
        case 9: //boat
    		color_mapped = 5;
    		break;
    	}
    	cv::rectangle(image, cv::Point(predictions[i].x, predictions[i].y), cv::Point((predictions[i].x + predictions[i].w), (predictions[i].y + predictions[i].h)),
    			colormap_semantic[color_mapped], 2);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////



void
image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	if (image_msg == NULL)
		return;

	double fps;
	static double start_time = 0.0;
	
	unsigned char *img;

	if (camera_side == 0)
		img = image_msg->raw_left;
	else
		img = image_msg->raw_right;

    //double timestamp = image_msg->timestamp;

    Size size(width,height);
	cv::Mat image_cv = cv::Mat(cv::Size(image_msg->width, image_msg->height), CV_8UC3, img);
	float IMAGE_HEIGHT_CROP = 0.91; //camera5
	cv::cvtColor(image_cv, image_cv, cv::COLOR_RGB2BGR);
	cv::Rect myROI(0, 0, (int) image_msg->width, (int) image_msg->height * IMAGE_HEIGHT_CROP); 
	image_cv = image_cv(myROI);

    cv::Mat imgResized;
    resize(image_cv, imgResized, size);

	unsigned char *resized_img = imgResized.data;

	vector<bbox_t> predictions = run_EfficientDet(resized_img, width, height);
	
	fps = 1.0 / (carmen_get_time() - start_time);
	start_time = carmen_get_time();
	printf("FPS= %.2f\n", fps);
    
	predictions = filter_predictions_of_interest_efficientdet(predictions);
    show_detections(imgResized, predictions);

	char frame_rate[25];
	sprintf(frame_rate, "FPS = %.2f", fps);
    putText(imgResized, frame_rate, Point(10, 25), FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 2);
    imshow("Image EfficientDet", imgResized);
	cv::waitKey(1);
	//publish_moving_objects_message(image_msg->timestamp, &msg);

}


void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
        carmen_ipc_disconnect();
        cvDestroyAllWindows();

        printf("EfficientDet: Disconnected.\n");
        exit(0);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////


void
subscribe_messages()
{
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);
}

void
read_parameters(int argc, char **argv)
{
	if ((argc != 3))
		carmen_die("%s: Wrong number of parameters. camera_to_efficientdet requires 2 parameter and received %d. \n Usage: %s <camera_number> <camera_side(0-left; 1-right)\n>", argv[0], argc - 1, argv[0]);

	camera = atoi(argv[1]);             // Define the camera to be used
    camera_side = atoi(argv[2]);        // 0 For left image 1 for right image

    int num_items;
    

    char bumblebee_string[256];
    char camera_string[256];

    sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera); // Geather the camera ID
    sprintf(camera_string, "%s%d", "camera", camera);

    carmen_param_t param_list[] =
    {
        {bumblebee_string, (char *)"width", CARMEN_PARAM_INT, &width, 0, NULL},
		{bumblebee_string, (char *)"height", CARMEN_PARAM_INT, &height, 0, NULL},
    };

    num_items = sizeof(param_list) / sizeof(param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);
    initialize_Efficientdet();
}

int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	read_parameters(argc, argv);

	subscribe_messages();

	signal(SIGINT, shutdown_module);

	setlocale(LC_ALL, "C");

	carmen_ipc_dispatch();

	return 0;
}