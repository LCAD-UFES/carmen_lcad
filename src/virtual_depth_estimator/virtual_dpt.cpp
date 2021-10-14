#include <carmen/carmen.h>
#include <vector>
#include <algorithm>
#include <fstream>
#include <carmen/libdpt.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int camera;
carmen_localize_ackerman_globalpos_message ackerman_message;
carmen_point_t globalpos;
carmen_pose_3D_t pose;
int rddf_received = 0;
int localize_received = 0;


double
euclidean_distance (double x1, double x2, double y1, double y2)
{
	return ( sqrt(pow(x2-x1,2) + pow(y2-y1,2)) );
}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void
image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	double img_timestamp = image_msg->timestamp;
	unsigned char *depth_pred;
	double fps;
	static double start_time = 0.0;
	start_time = carmen_get_time();

	depth_pred = libdpt_process_image(image_msg->width, image_msg->height, image_msg->raw_right, img_timestamp);
    // printf("%f, %f, %f, %f\n",preds[0], preds[1], preds[2], preds[3]);
	char info[128];
	cv::Mat imgdepth = cv::Mat(image_msg->height, image_msg->width, CV_16U, depth_pred);
	fps = 1.0 / (carmen_get_time() - start_time);
	sprintf(info, "FPS %.2f", fps);
    putText(imgdepth, info, cv::Point(320, 450), cv::FONT_HERSHEY_PLAIN, 1, cvScalar(255, 255, 0, 255), 1);
	cv::imshow("Depth Prediction Transformer", imgdepth);
	cv::waitKey(1);
}

void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
    	carmen_ipc_disconnect();
        printf("Virtual_DPT: Disconnected.\n");
        exit(0);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Subscribes                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void
subscribe_messages()
{
	carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);
}

void
read_parameters(char **argv)
{
	camera = atoi(argv[1]);
}


int
main(int argc , char **argv)
{
	if(argc!=2)
	{
		printf("É necessário passar o ID da câmera como parâmetro.\nExemplo: ./virtual_dpt 3\n");
		exit(1);
	}

	carmen_ipc_initialize(argc, argv);

	signal(SIGINT, shutdown_module);

	read_parameters(argv);
	
	/* Register Python Context for dpt*/
	initialize_python_context();

	printf("Aguardando mensagem\n");

	subscribe_messages();

	carmen_ipc_dispatch();

	return 0;
}
