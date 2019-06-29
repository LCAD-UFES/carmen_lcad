#include <carmen/carmen.h>
#include <control.h>
#include <sys/socket.h>
#include <netdb.h>

#define	NUM_MOTION_COMMANDS_PER_VECTOR	200

//#define tcp_ip_address "10.9.8.181"
//char *tcp_ip_address = "192.168.0.1";
char *tcp_ip_address = (char *) "127.0.0.1";

#define PORT "3457"


int
apply_system_latencies(carmen_ackerman_motion_command_p current_motion_command_vector, int nun_motion_commands)
{
	int i, j;

	for (i = 0; i < nun_motion_commands; i++)
	{
		j = i;
		for (double lat = 0.0; lat < 0.2; j++)
		{
			if (j >= nun_motion_commands)
				break;
			lat += current_motion_command_vector[j].time;
		}
		if (j >= nun_motion_commands)
			break;
		current_motion_command_vector[i].phi = current_motion_command_vector[j].phi;
//		current_motion_command_vector[i].phi = current_motion_command_vector[j].phi;
//		current_motion_command_vector[i].v = current_motion_command_vector[j].v;
//		current_motion_command_vector[i].x = current_motion_command_vector[j].x;
//		current_motion_command_vector[i].y = current_motion_command_vector[j].y;
//		current_motion_command_vector[i].theta = current_motion_command_vector[j].theta;
//		current_motion_command_vector[i].time = current_motion_command_vector[j].time;
	}

	for (i = 0; i < nun_motion_commands; i++)
	{
		j = i;
		for (double lat = 0.0; lat < 0.6; j++)
		{
			if (j >= nun_motion_commands)
				break;
			lat += current_motion_command_vector[j].time;
		}
		if (j >= nun_motion_commands)
			break;
		current_motion_command_vector[i].v = current_motion_command_vector[j].v;
	}

	return (i);
}


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


//int
//trying_to_reconnect()
//{
//	int pi_socket = stablished_connection_with_server();
//
//	while (pi_socket == -1)
//	{
//		sleep(3);
//		pi_socket = stablished_connection_with_server();
//	}
//	return (pi_socket);
//}


void
build_socket_message(carmen_base_ackerman_motion_command_message *motion_command_message, double *array)
{
	int size = motion_command_message->num_motion_commands;

	array[0] = double(size);
	array[1] = DBL_MAX;

	for (int i = 0; i < size; i++)
	{
		array[(i * 6) + 2] = motion_command_message->motion_command[i].x;
		array[(i * 6) + 3] = motion_command_message->motion_command[i].y;
		array[(i * 6) + 4] = motion_command_message->motion_command[i].theta;
		array[(i * 6) + 5] = motion_command_message->motion_command[i].v;
		array[(i * 6) + 6] = motion_command_message->motion_command[i].phi;
		array[(i * 6) + 7] = motion_command_message->motion_command[i].time;
	}
}


void
send_motion_command_via_socket(double* array)
{
	static int pi_socket = 0;
	int result = 0;

	if (pi_socket == 0)
		pi_socket = stablished_connection_with_server();
																							// 2 + 6 * 200
	result = send(pi_socket, array, 9616, MSG_NOSIGNAL);									// The socket returns the number of bytes read, 0 in case of connection lost, -1 in case of error

	if (result == 0 || result == -1)														// 0 Connection lost due to server shutdown -1 Could not connect
	{
		close(pi_socket);
		pi_socket = stablished_connection_with_server();
		return;
	}
	//printf("Sent %lf --- %lf --- %lf\n", array[0], array[1], array[2]);
}
//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Handlers                                                                                     //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
motion_command_handler(carmen_base_ackerman_motion_command_message *motion_command_message)
{
	//printf("Motion command!/n");

	if (motion_command_message->num_motion_commands < 1)
			return;

	if (motion_command_message->num_motion_commands > NUM_MOTION_COMMANDS_PER_VECTOR)
		motion_command_message->num_motion_commands = NUM_MOTION_COMMANDS_PER_VECTOR;

	double array[1202]; // 2 + 200
	build_socket_message(motion_command_message, array);

	send_motion_command_via_socket(array);
}


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


static int
subscribe_to_relevant_messages()
{
	carmen_base_ackerman_subscribe_motion_command(NULL, (carmen_handler_t) motion_command_handler, CARMEN_SUBSCRIBE_LATEST);

	return (0);
}


//static int
//initialize_ipc(void)
//{
//	IPC_RETURN_TYPE err;
//
//	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_MOTION_COMMAND_NAME, IPC_VARIABLE_LENGTH, CARMEN_BASE_ACKERMAN_MOTION_COMMAND_FMT);
//	if (err != IPC_OK)
//		return -1;
//
//	return 0;
//}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_module);

//	if (initialize_ipc() < 0)
//		carmen_die("Error in initializing ipc...\n");

	if (subscribe_to_relevant_messages() < 0)
		carmen_die("Error subscribing to messages...\n");

	carmen_ipc_dispatch();

	exit(0);
}
