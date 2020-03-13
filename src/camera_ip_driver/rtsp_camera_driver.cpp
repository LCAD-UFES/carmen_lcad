#include "rtsp_camera_driver.h"

char * rtsp_address;
Mat cameraMatrix, distCoeffs, newcameramtx, R1;
///////////////////////////////////////////////////////////////////////////////////////////////
//																							 //
// Publishers																			     //
//																						     //
///////////////////////////////////////////////////////////////////////////////////////////////

void publish_image_message(int camera_number,
						   carmen_bumblebee_basic_stereoimage_message *msg)
{
	msg->timestamp = carmen_get_time();
	carmen_bumblebee_basic_publish_message(camera_number, msg);
}
///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//																							 //
// Handlers																					 //
//																							 //
///////////////////////////////////////////////////////////////////////////////////////////////

void signal_handler(int sig)
{
	printf("Signal %d received, exiting program ...\n", sig);
	exit(1);
}
///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//																						     //
// Initializations																		     //
//																						     //
///////////////////////////////////////////////////////////////////////////////////////////////

int read_parameters(int argc, char **argv, carmen_bumblebee_basic_stereoimage_message *msg)
{
	if (argc != 2)
		carmen_die(
			"--- Wrong number of parameters. ---\nUsage: %s <camera_number>\n",
			argv[0]);

	int camera_number = atoi(argv[1]);
	
	char camera[256];

	sprintf(camera, "%s%d", "bumblebee_basic", camera_number);
	double fx_factor, fy_factor, cu_factor, cv_factor, k1, k2, p1, p2, k3;

	carmen_param_t param_list[] = {
		{ camera, (char *)"width", CARMEN_PARAM_INT, &msg->width, 0, NULL},
		{ camera, (char *)"height", CARMEN_PARAM_INT, &msg->height, 0, NULL},
		{ camera, (char*) "fx", CARMEN_PARAM_DOUBLE, &fx_factor, 0, NULL },
        { camera, (char*) "fy", CARMEN_PARAM_DOUBLE, &fy_factor, 0, NULL },
        { camera, (char*) "cu", CARMEN_PARAM_DOUBLE, &cu_factor, 0, NULL },
        { camera, (char*) "cv", CARMEN_PARAM_DOUBLE, &cv_factor, 0, NULL },
		{ camera, (char*) "k1", CARMEN_PARAM_DOUBLE, &k1, 0, NULL },
		{ camera, (char*) "k2", CARMEN_PARAM_DOUBLE, &k2, 0, NULL },
		{ camera, (char*) "p1", CARMEN_PARAM_DOUBLE, &p1, 0, NULL },
		{ camera, (char*) "p2", CARMEN_PARAM_DOUBLE, &p2, 0, NULL },
		{ camera, (char*) "k3", CARMEN_PARAM_DOUBLE, &k3, 0, NULL },
		{ camera, (char*) "rtsp_address", CARMEN_PARAM_STRING, &rtsp_address, 0, NULL },
	};

	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
  	
	cameraMatrix = (cv::Mat_<double>(3, 3) << fx_factor, 0, cu_factor, 0, fy_factor, cv_factor, 0, 0, 1);
	newcameramtx = (cv::Mat_<double>(3, 3) << fx_factor, 0, cu_factor, 0, fy_factor, cv_factor, 0, 0, 1);
	distCoeffs = (cv::Mat_<double>(5,1) << k1, k2, p1, p2, k3);
	R1 = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
	
	return (camera_number);
}

void initialize_message(carmen_bumblebee_basic_stereoimage_message *msg)
{
	msg->image_size = msg->width * msg->height * 3; // 3 channels RGB
	//msg->isRectified = 1;
	msg->host = carmen_get_host();
}

int main(int argc, char **argv)
{
	carmen_bumblebee_basic_stereoimage_message msg;
	
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	int camera_number = read_parameters(argc, argv, &msg);
	
	initialize_message(&msg);
	
	carmen_bumblebee_basic_define_messages(camera_number);
	
	VideoCapture vcap;
	Mat image;
	Mat imgResized;
	Mat dst;

	Size size(msg.width,msg.height);
	
	//Init rectified parameters
	Mat MapX;
	Mat MapY;
	
	initUndistortRectifyMap(cameraMatrix, distCoeffs, R1, newcameramtx, size, CV_16SC2, MapX, MapY);
	
	string videoStreamAddress = string(rtsp_address);
	
	if (!vcap.open(videoStreamAddress))
	{
		cout << "Error opening video stream or file" << endl;
		return -1;
	}

	while (1)
	{

		if (!vcap.read(image))
		{
			cout << "No frame" << endl;
			waitKey();
		}
		else
		{
			resize(image, imgResized, size);
			//rectifying the image
			remap(imgResized, dst, MapX, MapY, INTER_LINEAR);
			cvtColor(imgResized, imgResized, CV_RGB2BGR);
			cvtColor(dst, dst, CV_RGB2BGR);
			msg.raw_left = imgResized.data;
			msg.raw_right = dst.data;
			publish_image_message(camera_number, &msg);
		}
		if (waitKey(1) >= 0)
			break;

	}
}
