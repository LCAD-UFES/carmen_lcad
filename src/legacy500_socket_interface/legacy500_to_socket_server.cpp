#include <carmen/carmen.h>
#include <control.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#define PORT 3458

int server_fd;

char *udp_client_address;


int
connect_with_client_tcp_ip()
{
    int new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("--- Socket Failed ---\n");
        return (-1);
    }
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
    {
        perror("--- Setsockopt Failed ---\n");
        return (-1);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);
    
    // Forcefully attaching socket to the port defined
    if (bind(server_fd, (struct sockaddr*) &address, sizeof(address)) < 0)
    {
        perror("--- Bind Failed ---\n");
        return (-1);
    }
    if (listen(server_fd, 3) < 0)
    {
        perror("-- Listen Failed ---\n");
        return (-1);
    }
    
    printf("--- Waiting for connection! --\n");
    if ((new_socket = accept(server_fd, (struct sockaddr*) &address, (socklen_t*) &addrlen)) < 0)
    {
        perror("--- Accept Failed ---\n");
        return (-1);
    }
    printf("--- Connection established successfully! ---\n");

	return (new_socket);
}


int
connect_with_client(struct sockaddr_in *client_address)
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
//    address.sin_port = htons(PORT);

    // Forcefully attaching socket to the port defined
//    if (bind(server_fd, (struct sockaddr *) &address, sizeof(address)) < 0)
//    {
//        perror("--- Bind Failed ---\n");
//        return (-1);
//    }
//    printf("--- Bind successful! ---\n");

    client_address->sin_family = AF_INET;
    client_address->sin_addr.s_addr = inet_addr(udp_client_address);
    client_address->sin_port = htons(PORT);

	return (new_socket);
}


void
build_odometry_socket_msg(carmen_base_ackerman_odometry_message *msg, double *array)
{
	array[0] = msg->x;
	array[1] = msg->y;
	array[2] = msg->theta;
	array[3] = msg->v;
	array[4] = msg->phi;
}


///////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Handlers                                                                                     //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
base_ackerman_odometry_handler(carmen_base_ackerman_odometry_message *msg)
{
	double array[5];

	build_odometry_socket_msg(msg, array);

	static struct sockaddr_in client_address;
	static int pi_socket = 0;
	if (pi_socket == 0)
		pi_socket = connect_with_client(&client_address);

	sendto(pi_socket, (void *) array, 40, 0, (struct sockaddr *) &client_address, sizeof(struct sockaddr_in));
}


void
robot_ackerman_velocity_handler_tcp_ip(carmen_robot_ackerman_velocity_message *msg)
{
	double array[5];
	static int pi_socket = 0;
	int result = 0;

	if (pi_socket == 0)
		pi_socket = connect_with_client_tcp_ip();

	array[0] = msg->v;
	array[1] = msg->phi;

	//printf ("v: %lf phi: %lf\n", array[0], array[1]);

	result = send(pi_socket, array, 40, MSG_NOSIGNAL);					// The socket returns the number of bytes read, 0 in case of connection lost, -1 in case of error

	if (result == 0 || result == -1)									// 0 Connection lost due to server shutdown -1 Could not connect
	{
		close(pi_socket);
		pi_socket = connect_with_client_tcp_ip();
		return;
	}
}

#define SEND_TRUEPOSE

#ifdef SEND_TRUEPOSE
void
robot_ackerman_velocity_handler(carmen_robot_ackerman_velocity_message *msg)
{
	double array[6];

	static struct sockaddr_in client_address;
	static int pi_socket = 0;
	if (pi_socket == 0)
		pi_socket = connect_with_client(&client_address);

	array[0] = msg->v;
	array[1] = msg->phi;
	array[2] = 1.0;
	array[3] = 2.0;
	array[4] = 3.0;
	array[5] = 4.0;

	//printf ("v: %lf phi: %lf\n", array[0], array[1]);

	sendto(pi_socket, (void *) array, 48, 0, (struct sockaddr *) &client_address, sizeof(struct sockaddr_in));
}

#else

void
robot_ackerman_velocity_handler(carmen_robot_ackerman_velocity_message *msg)
{
	double array[5];

	static struct sockaddr_in client_address;
	static int pi_socket = 0;
	if (pi_socket == 0)
		pi_socket = connect_with_client(&client_address);

	array[0] = msg->v;
	array[1] = msg->phi;

	//printf ("v: %lf phi: %lf\n", array[0], array[1]);

	sendto(pi_socket, (void *) array, 40, 0, (struct sockaddr *) &client_address, sizeof(struct sockaddr_in));
}
#endif

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
subscribe_to_relevant_messages()
{
	// Message published by simulator_ackerman
	carmen_base_ackerman_subscribe_odometry_message_2(NULL, (carmen_handler_t) base_ackerman_odometry_handler, CARMEN_SUBSCRIBE_LATEST);

	// Message published by ford_escape_hybrid
	carmen_robot_ackerman_subscribe_velocity_message_2(NULL, (carmen_handler_t) robot_ackerman_velocity_handler, CARMEN_SUBSCRIBE_LATEST);

	return (0);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_module);

	if (argc == 2)
		udp_client_address = argv[1];

	if (subscribe_to_relevant_messages() < 0)
		carmen_die("Error subscribing to messages...\n");

	carmen_ipc_dispatch();

	return (0);
}
