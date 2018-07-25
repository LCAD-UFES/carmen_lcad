#include <carmen/carmen.h>
#include <carmen/camera_messages.h>
#include <carmen/camera_interface.h>
#include <carmen/bumblebee_basic_interface.h>
#include <sys/socket.h>
#include <netdb.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

char* tcp_ip_address;

#define PORT "3457"
#define SOCKET_DATA_PACKET_SIZE	2048

using namespace std;
using namespace cv;


int
stablished_connection_with_server()
{
	struct addrinfo host_info;       // The struct that getaddrinfo() fills up with data.
	struct addrinfo *host_info_list; // Pointer to the to the linked list of host_info's.
	int pi_socket = 0, status;

	if ((pi_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("--- Socket creation error! ---\n");
		return -1;
	}
	memset(&host_info, 0, sizeof host_info);

	host_info.ai_family = AF_UNSPEC;     // IP version not specified. Can be both.
	host_info.ai_socktype = SOCK_STREAM; // Use SOCK_STREAM for TCP or SOCK_DGRAM for UDP.

	status = getaddrinfo(tcp_ip_address, PORT, &host_info, &host_info_list);

	if (status != 0)
	{
		printf("--- Get_Addrinfo ERROR! ---\n");
		return (-1);
	}
	status = connect(pi_socket, host_info_list->ai_addr, host_info_list->ai_addrlen);

	if(status < 0)
	{
		printf("--- Connection Failed! ---\n");
		return (-1);
	}

	printf("--- Connection established successfully! ---\n");

	return (pi_socket);
}


int
trying_to_reconnect()
{
	int pi_socket = stablished_connection_with_server();

	while (pi_socket == -1)
	{
		sleep((unsigned int) 5);
		pi_socket = stablished_connection_with_server();
	}
	return (pi_socket);
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//																							 //
// Publishers																			     //
//																						     //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_image_message(int camera_number, carmen_bumblebee_basic_stereoimage_message *msg)
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


void
signal_handler(int sig)
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


int
read_parameters(int argc, char **argv, carmen_bumblebee_basic_stereoimage_message *msg, char *cam_config)
{
	if (argc != 2)
		carmen_die("--- Wrong number of parameters. ---\nUsage: %s <camera_number>\n", argv[0]);

	int frame_rate, brightness, contrast, camera_number = atoi(argv[1]);

	char pi_camera_number[256];

	sprintf(pi_camera_number, "%s%d", "camera", camera_number);

	carmen_param_t param_list[] =
	{
		{pi_camera_number, (char*)"width",      CARMEN_PARAM_INT,    &msg->width,     0, NULL},
		{pi_camera_number, (char*)"height",     CARMEN_PARAM_INT,    &msg->height,    0, NULL},
		{pi_camera_number, (char*)"frame_rate", CARMEN_PARAM_INT,    &frame_rate,     0, NULL},
		{pi_camera_number, (char*)"brightness", CARMEN_PARAM_INT,    &brightness,     0, NULL},
		{pi_camera_number, (char*)"contrast",   CARMEN_PARAM_INT,    &contrast,       0, NULL},
		{pi_camera_number, (char*)"ip",         CARMEN_PARAM_STRING, &tcp_ip_address, 0, NULL},
	};

	int num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	sprintf(cam_config, "%d*%d*%d*%d*%d*", msg->width, msg->height, frame_rate, brightness, contrast);

	return (camera_number);
}


void
initialize_message(carmen_bumblebee_basic_stereoimage_message *msg)
{
	msg->image_size = msg->width * msg->height * 3; // 3 channels RGB
	msg->isRectified = 1;
	msg->raw_left = (unsigned char *) calloc(msg->image_size, sizeof(unsigned char));
	msg->raw_right = msg->raw_left;  // This is a monocular camera, both pointers point to the same image
	msg->host = carmen_get_host();

	//printf("\nWidth %d Height %d Image Size %d Is Rectified %d Host %s\n\n", msg->width, msg->height, msg->image_size, msg->isRectified, msg->host);
}


int
main()
{
//	carmen_bumblebee_basic_stereoimage_message msg;

//	carmen_ipc_initialize(argc, argv);

//	carmen_param_check_version(argv[0]);

//	int camera_number = read_parameters(argc, argv, &msg, cam_config);
//	initialize_message(&msg);

//	carmen_bumblebee_basic_define_messages(camera_number);

	int pi_socket = stablished_connection_with_server();

	int valread;
	while (1)
	{
		unsigned char rpi_imu_data[SOCKET_DATA_PACKET_SIZE];
		// The socket returns the number of bytes read, 0 in case of connection lost, -1 in case of error
		valread = recv(pi_socket, rpi_imu_data, SOCKET_DATA_PACKET_SIZE, MSG_WAITALL);

		if (valread == 0) // Connection lost due to server shutdown.
		{
			close(pi_socket);
			pi_socket = trying_to_reconnect();
			continue;
		}
		else if ((valread == -1) || (valread != SOCKET_DATA_PACKET_SIZE))
			continue;

		printf("%s", rpi_imu_data);
//		publish_image_message(camera_number, &msg);

		//imshow("Pi Camera Driver", cv_image);  waitKey(1);
	}
}
