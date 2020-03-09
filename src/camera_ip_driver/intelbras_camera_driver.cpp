#include "intelbras_camera_driver.h"

char * rtsp_address;
Mat cameraMatrix, distCoeffs;
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
	if (argc != 3)
		carmen_die(
			"--- Wrong number of parameters. ---\nUsage: %s <camera_number> <rtsp_address>\n",
			argv[0]);

	// int frame_rate, brightness, contrast,
	int camera_number = atoi(argv[1]);

	char intelbras_camera_number[256];

	sprintf(intelbras_camera_number, "%s%d", "camera", camera_number);
	
	double fx_factor, fy_factor, cu_factor, cv_factor, k1, k2, p1, p2, k3;
	
	carmen_param_t param_list[] = {
		{intelbras_camera_number, (char *)"width", CARMEN_PARAM_INT, &msg->width, 0, NULL},
		{intelbras_camera_number,  (char *)"height", CARMEN_PARAM_INT, &msg->height, 0, NULL},
		{ intelbras_camera_number, (char*) "fx", CARMEN_PARAM_DOUBLE, &fx_factor, 0, NULL },
        { intelbras_camera_number, (char*) "fy", CARMEN_PARAM_DOUBLE, &fy_factor, 0, NULL },
        { intelbras_camera_number, (char*) "cu", CARMEN_PARAM_DOUBLE, &cu_factor, 0, NULL },
        { intelbras_camera_number, (char*) "cv", CARMEN_PARAM_DOUBLE, &cv_factor, 0, NULL },
		{ intelbras_camera_number, (char*) "k1", CARMEN_PARAM_DOUBLE, &k1, 0, NULL },
		{ intelbras_camera_number, (char*) "k2", CARMEN_PARAM_DOUBLE, &k2, 0, NULL },
		{ intelbras_camera_number, (char*) "p1", CARMEN_PARAM_DOUBLE, &p1, 0, NULL },
		{ intelbras_camera_number, (char*) "p2", CARMEN_PARAM_DOUBLE, &p2, 0, NULL },
		{ intelbras_camera_number, (char*) "k3", CARMEN_PARAM_DOUBLE, &k3, 0, NULL },
	};

	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	rtsp_address = argv[2];

	cameraMatrix = Mat::zeros(3, 3, CV_64FC1);
	distCoeffs = Mat::zeros(5, 1, CV_64FC1);
	
	cameraMatrix.at<double>(0, 0) = fx_factor;
	cameraMatrix.at<double>(1, 1) = fy_factor;
	cameraMatrix.at<double>(0, 2) = cu_factor;
	cameraMatrix.at<double>(1, 2) = cv_factor;
	cameraMatrix.at<double>(2, 2) = 1;
	
	distCoeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);

	return (camera_number);
}

void initialize_message(carmen_bumblebee_basic_stereoimage_message *msg)
{
	msg->image_size = msg->width * msg->height * 3; // 3 channels RGB
	msg->isRectified = 1;
//	msg->raw_left = (unsigned char *)calloc(msg->image_size,
//											sizeof(unsigned char));
//	msg->raw_right = &msg.raw_left; // This is a monocular camera, both pointers point to the same image
	msg->host = carmen_get_host();

	//printf("\nWidth %d Height %d Image Size %d Is Rectified %d Host %s\n\n", msg->width, msg->height, msg->image_size, msg->isRectified, msg->host);
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
	Mat Map1(size, CV_32FC1);
	Mat Map2(size, CV_32FC1);
	Mat R1 = Mat::zeros(3, 3, CV_64FC1);

	initUndistortRectifyMap(cameraMatrix, distCoeffs, R1, cameraMatrix, size, CV_16SC2, Map1, Map2);
	
	
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
			// imshow("Intelbras Window", image);
			cvtColor(image, image, CV_RGB2BGR);
			resize(image, imgResized, size);
			//rectifying the image
			remap(imgResized, dst, Map1, Map2, INTER_LINEAR);
			msg.raw_left = dst.data;
			msg.raw_right = msg.raw_left;
			publish_image_message(camera_number, &msg);
		}
		if (waitKey(1) >= 0)
			break;

	}
}
