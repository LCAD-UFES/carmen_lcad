#include "unv_camera_driver.h"

char * tcp_ip_address;
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

	// int frame_rate, brightness, contrast,
	int camera_number = atoi(argv[1]);

	char unv_camera_number[256];

	sprintf(unv_camera_number, "%s%d", "camera", camera_number);

	carmen_param_t param_list[] = {
		 {unv_camera_number, (char *)"width",
		  CARMEN_PARAM_INT, &msg->width, 0, NULL},
		 {unv_camera_number,
		  (char *)"height", CARMEN_PARAM_INT, &msg->height, 0, NULL},
		// {unv_camera_number, (char *)"frame_rate", CARMEN_PARAM_INT,
		//  &frame_rate, 0, NULL},
		// {unv_camera_number, (char *)"brightness",
		//  CARMEN_PARAM_INT, &brightness, 0, NULL},
		// {unv_camera_number,
		//  (char *)"contrast", CARMEN_PARAM_INT, &contrast, 0, NULL},
		 //{unv_camera_number, (char *)"ip", CARMEN_PARAM_STRING,
		 //&tcp_ip_address, 0, NULL},
	};

	int num_items = sizeof(param_list) / sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	tcp_ip_address = argv[2];
	// sprintf(cam_config, "%d*%d*%d*%d*%d*", msg->width, msg->height, frame_rate,
			// brightness, contrast);
	// sprintf(cam_config, "%d*%d", msg->width, msg->height);

	return (camera_number);
}

void initialize_message(carmen_bumblebee_basic_stereoimage_message *msg)
{
	msg->image_size = msg->width * msg->height * 3; // 3 channels RGB
//	msg->isRectified = 1;
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
	Mat dst;

	Size size(msg.width,msg.height);
	
	string videoStreamAddress = "rtsp://admin:123456@";
	videoStreamAddress += string(tcp_ip_address);
	videoStreamAddress += string("/media/video1.cgi?.mjpg");
	
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
			// imshow("UNV Window", image);
			cvtColor(image, image, CV_RGB2BGR);
			resize(image, dst, size);
			msg.raw_left = dst.data;
			msg.raw_right = msg.raw_left;
			publish_image_message(camera_number, &msg);
		}
		if (waitKey(1) >= 0)
			break;

	}
}
