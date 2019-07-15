#include <carmen/carmen.h>
#include <control.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#define	NUM_MOTION_COMMANDS_PER_VECTOR	200
#define NUM_DOUBLES_IN_SOCKET_MOTION_COMMAND (1 + NUM_MOTION_COMMANDS_PER_VECTOR * 6)

//char *ip_address = "192.168.0.1";
char *ip_address = (char *) "127.0.0.1";

#define PORT "3457"


int
stablished_connection_with_server_tcp_ip()
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

	status = getaddrinfo(ip_address, PORT, &host_info, &host_info_list);

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
stablished_connection_with_server(struct sockaddr_in *client_address)
{
    int new_socket;
//    struct sockaddr_in address;

    // Creating socket file descriptor
    if ((new_socket = socket(AF_INET, SOCK_DGRAM, 0)) == 0)
    {
        perror("--- Socket Failed ---\n");
        return (-1);
    }

//    address.sin_family = AF_INET;
//    address.sin_addr.s_addr = INADDR_ANY;
//    address.sin_port = htons(atoi(PORT));

    // Forcefully attaching socket to the port defined
//    if (bind(new_socket, (struct sockaddr *) &address, sizeof(address)) < 0)
//    {
//        perror("--- Bind Failed ---\n");
//        return (-1);
//    }
//    printf("--- Bind successful! ---\n");

    client_address->sin_family = AF_INET;
    client_address->sin_addr.s_addr = inet_addr(ip_address);
    client_address->sin_port = htons(atoi(PORT));

	return (new_socket);
}


void
build_socket_message(carmen_base_ackerman_motion_command_message *motion_command_message, double *array)
{
	int size = motion_command_message->num_motion_commands;

	array[0] = size;

	for (int i = 0; i < size; i++)
	{
		array[(i * 6) + 1] = motion_command_message->motion_command[i].x;
		array[(i * 6) + 2] = motion_command_message->motion_command[i].y;
		array[(i * 6) + 3] = motion_command_message->motion_command[i].theta;
		array[(i * 6) + 4] = motion_command_message->motion_command[i].v;
		array[(i * 6) + 5] = motion_command_message->motion_command[i].phi;
		array[(i * 6) + 6] = motion_command_message->motion_command[i].time;
	}
}


void
send_motion_command_via_socket_tcp_ip(double* array)
{
	static int pi_socket = 0;
	int result = 0;

	if (pi_socket == 0)
		pi_socket = stablished_connection_with_server_tcp_ip();

	result = send(pi_socket, array, NUM_DOUBLES_IN_SOCKET_MOTION_COMMAND * sizeof(double), MSG_NOSIGNAL);

	if (result <= 0)
	{
		close(pi_socket);
		pi_socket = stablished_connection_with_server_tcp_ip();

		return;
	}
}


void
send_motion_command_via_socket(double* array)
{
	static struct sockaddr_in client_address;
	static int pi_socket = 0;
	if (pi_socket == 0)
		pi_socket = stablished_connection_with_server(&client_address);

	sendto(pi_socket, (void *) array, NUM_DOUBLES_IN_SOCKET_MOTION_COMMAND * sizeof(double), 0, (struct sockaddr *) &client_address, sizeof(struct sockaddr_in));
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
	if (motion_command_message->num_motion_commands < 1)
			return;

	if (motion_command_message->num_motion_commands > NUM_MOTION_COMMANDS_PER_VECTOR)
		motion_command_message->num_motion_commands = NUM_MOTION_COMMANDS_PER_VECTOR;

	double array[NUM_DOUBLES_IN_SOCKET_MOTION_COMMAND];
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


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_module);

	if (argc == 2)
		ip_address = argv[1];

	if (subscribe_to_relevant_messages() < 0)
		carmen_die("Error subscribing to messages...\n");

	carmen_ipc_dispatch();

	exit(0);
}
