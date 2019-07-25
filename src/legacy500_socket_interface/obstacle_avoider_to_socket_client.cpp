#include <carmen/carmen.h>
#include <control.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>

#define	NUM_MOTION_COMMANDS_PER_VECTOR	200
#define NUM_DOUBLES_IN_SOCKET_MOTION_COMMAND (1 + NUM_MOTION_COMMANDS_PER_VECTOR * 6)

#define DEFAULT_PORT_NUMBER 3457
#define DEFAULT_IP_ADDRESS "127.0.0.1"

char *ip_address = (char *) DEFAULT_IP_ADDRESS;
int port_number = DEFAULT_PORT_NUMBER;
bool bit_enabled = false;

char *help_msg = "Usage: ./obstacle_avoider_to_socket_client [options]\n"
			"This application sends motion commands to the Integration Server via UDP\n"
			"Available options:\n"
			"-ip_address <ip>\tIntegration Server IP address (default: 127.0.0.1)\n"
			"-port_number <port>\tIntegration Server is listening for UDP datagrams at this port number (default: 3457)\n"
			"-bit\t\t\tBuit-in test\n"
			"-help\t\t\tPrint this help message\n";

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

	char temp[256];
	sprintf(temp, "%d", port_number);
	status = getaddrinfo(ip_address, temp, &host_info, &host_info_list);

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
    client_address->sin_port = htons(port_number);

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
// Built-in tests                                                                               //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////
void
socket_test(int size, double xmin, double xmax, double ymin, double ymax, double thetamin, double thetamax, double vmin, double vmax, double phimin, double phimax, double timemin, double timemax)
{
	double array[NUM_DOUBLES_IN_SOCKET_MOTION_COMMAND];
	double x = xmin, deltax = (xmax - xmin)/size;;
	double y = ymin, deltay = (ymax - ymin)/size;;
	double theta = thetamin, deltatheta = (thetamax - thetamin)/size;;
	double v = vmin, deltav = (vmax - vmin)/size;
	double phi = phimin, deltaphi = (phimax - phimin)/size;
	double time = timemin, deltatime = (timemax - timemin)/size;

	array[0] = size;
	printf ("n: %d\n", (int) array[0]);
	for (int i = 0; i < size; i++)
	{
		array[(i * 6) + 1] = x;
		array[(i * 6) + 2] = y;
		array[(i * 6) + 3] = theta;
		array[(i * 6) + 4] = v;
		array[(i * 6) + 5] = phi;
		array[(i * 6) + 6] = time;
		x += deltax;
		y += deltay;
		theta += deltatheta;
		v += deltav;
		phi += deltaphi;
		time += deltatime;

		printf ("i: %d >>> x: %lf y: %lf theta: %lf v: %lf phi: %lf time: %lf\n", i, x, y, theta, v, phi, time);
	}

	send_motion_command_via_socket(array);
}


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


void parse_args(int argc, char **argv)
{
	for (int i = 1; i < argc; i++)
	{
		bool valid_arg = false;
		
		if (strcmp(argv[i], "-ip_address") == 0)
		{
			if (argc > ++i)
			{
				ip_address = argv[i];
				printf ("ip_address: %s\n", ip_address);
				valid_arg = true;
			}
		}
		else if (strcmp(argv[i], "-port_number") == 0)
		{
			if (argc > ++i)
			{
				port_number = atoi(argv[i]);
				printf ("port_number: %d\n", port_number);
				valid_arg = true;
			}
		}
		else if (strcmp(argv[i], "-bit") == 0)
		{
			bit_enabled = true;
			printf ("bit: true");
			valid_arg = true;
		}
		else if (strcmp(argv[i], "-help") == 0)
		{
			printf("%s\n", help_msg);
			exit(0);
		}
		

		if (!valid_arg)
		{
			printf("%s\n", help_msg);
			exit(-1);
		}
	}
}


int
main(int argc, char **argv)
{
	parse_args(argc, argv);

	if (bit_enabled)
	{
		socket_test(80, -10.0, 10.0, -5.0, 5.0, -3.0, 3.0, 0.0, 10.0, -2.0, 2.0, 0.1, 1.0);
	}
	else
	{
		carmen_ipc_initialize(argc, argv);
		carmen_param_check_version(argv[0]);
		signal(SIGINT, shutdown_module);

		if (subscribe_to_relevant_messages() < 0)
			carmen_die("Error subscribing to messages...\n");

		carmen_ipc_dispatch();
	}

	exit(0);
}
