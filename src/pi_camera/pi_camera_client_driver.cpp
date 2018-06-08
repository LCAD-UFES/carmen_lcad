#include <carmen/carmen.h>
#include <carmen/camera_messages.h>
#include <sys/socket.h>
#include <netdb.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#define PORT "3457"
//#define TCP_IP_ADDRESS "192.168.0.15"


int image_width;    // These variables will be read from the carmen_fordscape.ini file
int image_height;
char* tcp_ip_address;
char* port;

using namespace std;
using namespace cv;


int 
old_main()
{
	struct addrinfo host_info;       // The struct that getaddrinfo() fills up with data.
	struct addrinfo *host_info_list; // Pointer to the to the linked list of host_info's.
	int sock = 0, valread, status;
	unsigned char raw_image[640 * 480 * 3] = {0};

	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("\n--- Socket creation error! ---\n");
		return -1;
	}

	memset(&host_info, 0, sizeof host_info);

	host_info.ai_family = AF_UNSPEC;     // IP version not specified. Can be both.
	host_info.ai_socktype = SOCK_STREAM; // Use SOCK_STREAM for TCP or SOCK_DGRAM for UDP.

	status = getaddrinfo(tcp_ip_address, port, &host_info, &host_info_list);

	if (status != 0)
	{
		printf("--- Erro no Get_Addrinfo! ---\n");
		return (-1);
	}

	status = connect(sock, host_info_list->ai_addr, host_info_list->ai_addrlen);

	if(status < 0)
	{
		printf("\n--- Connection Failed! ---\n");
		return (-1);
	}

	printf("Connection stablished!\n");

	while (1)
	{
		valread = recv(sock, raw_image, 640 * 480 * 3, MSG_WAITALL);

		if (valread == 0)
		{
			printf("Message not Received! ---\n");
		}
		//Mat open_cv_image = Mat(480, 640, CV_8UC3, raw_image, 3 * 640);
		//imshow("Neural Object Detector", open_cv_image);
		//waitKey(1);
	}

	return 0;
}


int
stablished_connection_with_server()
{
	struct addrinfo host_info;       // The struct that getaddrinfo() fills up with data.
	struct addrinfo *host_info_list; // Pointer to the to the linked list of host_info's.
	int sock = 0, status;

	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("\n--- Socket creation error! ---\n");
		return -1;
	}

	memset(&host_info, 0, sizeof host_info);

	host_info.ai_family = AF_UNSPEC;     // IP version not specified. Can be both.
	host_info.ai_socktype = SOCK_STREAM; // Use SOCK_STREAM for TCP or SOCK_DGRAM for UDP.

	status = getaddrinfo(tcp_ip_address, port, &host_info, &host_info_list);

	if (status != 0)
	{
		printf("--- Get_Addrinfo ERROR! ---\n");
		return (-1);
	}
	status = connect(sock, host_info_list->ai_addr, host_info_list->ai_addrlen);

	if(status < 0)
	{
		printf("\n--- Connection Failed! ---\n");
		return (-1);
	}

	printf("--- Connection stablished successfully! ---\n");

	return (sock);
}



///////////////////////////////////////////////////////////////////////////////////////////////
//																							 //
// Publishers																			     //
//																						     //
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
//																						     //
// Initializations																		     //
//																						     //
///////////////////////////////////////////////////////////////////////////////////////////////



void
read_parameters(int argc, char **argv, int camera_number)
{
	char pi_camera_number[256];

	sprintf(pi_camera_number, "%s%d", "pi_camera", camera_number);

	carmen_param_t param_list[] =
	{
		{pi_camera_number, (char*)"width",  CARMEN_PARAM_INT,    &image_width,    0, NULL},
		{pi_camera_number, (char*)"height", CARMEN_PARAM_INT,    &image_height,   0, NULL},
		{pi_camera_number, (char*)"ip",     CARMEN_PARAM_STRING, &tcp_ip_address, 0, NULL},
		{pi_camera_number, (char*)"port",   CARMEN_PARAM_STRING, &port,           0, NULL},
	};

	int num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);
}


void
define_messages(int camera_number)
{
  IPC_RETURN_TYPE err;

  char *message_name = (char*) malloc(128 * sizeof(char));

  sprintf(message_name, "carmen_pi_cameraimage%d", camera_number);

  err = IPC_defineMsg("carmen_camera_image", IPC_VARIABLE_LENGTH, CARMEN_CAMERA_IMAGE_FMT);

  carmen_test_ipc_exit(err, "Could not define", "carmen_camera_image");

  free(message_name);
}


int
main(int argc, char **argv)
{
	int valread;
	unsigned char raw_image[640 * 480 * 3] = {0};

	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	if (argc != 2)
	    carmen_die("%s: Wrong number of parameters. \nUsage: %s <camera_number>\n", argv[0], argv[0]);

	int camera_number = atoi(argv[1]);

	read_parameters(argc, argv, camera_number);

	define_messages(camera_number);

	//printf("%d %d %s %s\n", image_width, image_height, tcp_ip_address, port);

	int socket = stablished_connection_with_server();

	while (1)
	{
		valread = recv(socket, raw_image, 640 * 480 * 3, MSG_WAITALL);

		if (valread == 0)
		{
			printf("-- Message not Received! ---\n");
		}
		Mat open_cv_image = Mat(480, 640, CV_8UC3, raw_image, 3 * 640);
		imshow("Neural Object Detector", open_cv_image);
		waitKey(1);
	}
	//carmen_ipc_dispatch();
}
