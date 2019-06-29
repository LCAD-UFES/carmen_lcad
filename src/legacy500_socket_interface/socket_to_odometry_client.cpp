#include <carmen/carmen.h>
#include <control.h>
#include <sys/socket.h>
#include <netdb.h>

#define	NUM_MOTION_COMMANDS_PER_VECTOR	200

//char *tcp_ip_address = "192.168.0.1";
char *tcp_ip_address = (char *) "127.0.0.1";

#define PORT "3458"


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

	if (status < 0)
	{
		printf("--- Connection Failed! ---\n");
		return (-1);
	}

	printf("--- Connection established successfully! ---\n");

	return (pi_socket);
}


void
extract_odometry_from_socket_and_send_base_ackerman_msg(double *array)
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_base_ackerman_odometry_message odometry;

	odometry.x     = array[0];
	odometry.y     = array[1];
	odometry.theta = array[2];
	odometry.v     = array[3];
	odometry.phi   = array[4];
	odometry.host  = carmen_get_host();
	odometry.timestamp = carmen_get_time();

//	printf ("%lf %lf %lf %lf %lf\n", array[0], array[1], array[2], array[3], array[4]);

	err = IPC_publishData(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, &odometry);
	carmen_test_ipc(err, "Could not publish base_odometry_message", CARMEN_BASE_ACKERMAN_ODOMETRY_NAME);
}
//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Publishers                                                                                   //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
publish_robot_ackerman_velocity_message(double *array)
{
	IPC_RETURN_TYPE err = IPC_OK;
	carmen_robot_ackerman_velocity_message odometry;

	odometry.v     = array[0];
	odometry.phi   = array[1];
	odometry.timestamp = carmen_get_time();
	odometry.host  = carmen_get_host();

	//printf ("%lf %lf %lf %lf %lf\n", array[0], array[1], array[2], array[3], array[4]);

	err = IPC_publishData(CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME, &odometry);
	carmen_test_ipc(err, "Could not publish ford_escape_hybrid message named carmen_robot_ackerman_velocity_message", CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME);

}
//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Handlers                                                                                     //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
shutdown_module(int x)            // Handles ctrl+c
{
	if (x == SIGINT)
	{
		carmen_ipc_disconnect();
		carmen_warn("\nDisconnected.\n");
		exit(0);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Inicializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


int
initialize_ipc(void)
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME, IPC_VARIABLE_LENGTH, CARMEN_ROBOT_ACKERMAN_VELOCITY_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME);


	return 0;
}


int
main(int argc, char **argv)
{
	double array[5];
	int result = 0;

	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_module);
	if (initialize_ipc() < 0)
		carmen_die("Error in initializing ipc...\n");

	if (argc == 2)
		tcp_ip_address = argv[1];

	int pi_socket = stablished_connection_with_server();

	while (1)
	{
		result = recv(pi_socket, array, 40, MSG_WAITALL); // The socket returns the number of bytes read, 0 in case of connection lost, -1 in case of error

//		printf ("Result %d\n", result);
//		printf ("%lf %lf %lf %lf %lf\n", array[0], array[1], array[2], array[3], array[4]);

		if (result == -1 || result == 0)
		{
			printf("--- Disconnected ---\n");
			sleep(3);
			pi_socket = stablished_connection_with_server();
		}
		else
		{
			publish_robot_ackerman_velocity_message(array);
		}
	}
	return (0);
}
