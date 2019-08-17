#include <carmen/carmen.h>
#include <control.h>
#include <sys/socket.h>
#include <netdb.h>

#define PORT 3457

#define	NUM_MOTION_COMMANDS_PER_VECTOR	200
#define NUM_DOUBLES_IN_SOCKET_MOTION_COMMAND (1 + NUM_MOTION_COMMANDS_PER_VECTOR * 6)

//#define USE_TCP_IP

#ifdef USE_TCP_IP

int server_fd;

int
connect_with_client()
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
    address.sin_port = htons( PORT );
    
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

#else

int
connect_with_client()
{
    int new_socket;
    struct sockaddr_in address;

    // Creating socket file descriptor
    if ((new_socket = socket(AF_INET, SOCK_DGRAM, 0)) == 0)
    {
        perror("--- Socket Failed ---\n");
        return (-1);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Forcefully attaching socket to the port defined
    if (bind(new_socket, (struct sockaddr *) &address, sizeof(address)) < 0)
    {
        perror("--- Bind Failed ---\n");
        return (-1);
    }
    printf("--- Bind successful! ---\n");

	return (new_socket);
}
#endif
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
extract_motion_command_vetor_from_socket_and_send_msg(double *msg)
{
	int size = msg[0];
	carmen_ackerman_motion_command_t motion_command[size];

	for (int i = 0; i < size; i++)
	{
		motion_command[i].x     = msg[(i * 6) + 1];
		motion_command[i].y     = msg[(i * 6) + 2];
		motion_command[i].theta = msg[(i * 6) + 3];
		motion_command[i].v     = msg[(i * 6) + 4];
		motion_command[i].phi   = msg[(i * 6) + 5];
		motion_command[i].time  = msg[(i * 6) + 6];

//		printf ("i: %d >>> x: %lf [m] y: %lf [m] theta: %lf [rad] (%lf [deg]) v: %lf [m/s] phi: %lf [rad] (%lf [deg]) time: %lf [s]\n", i,
//				motion_command[i].x,
//				motion_command[i].y,
//				motion_command[i].theta,
//				carmen_radians_to_degrees(motion_command[i].theta),
//				motion_command[i].v,
//				motion_command[i].phi,
//				carmen_radians_to_degrees(motion_command[i].phi),
//				motion_command[i].time);

	}

	carmen_base_ackerman_publish_motion_command_2(motion_command, size, carmen_get_time());
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


int
main(int argc, char **argv)
{
	double msg[NUM_DOUBLES_IN_SOCKET_MOTION_COMMAND];
	int result = 0;

	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_module);

	int pi_socket = connect_with_client();

	while (1)
	{
#ifdef USE_TCP_IP
		result = recv(pi_socket, msg, NUM_DOUBLES_IN_SOCKET_MOTION_COMMAND * sizeof(double), MSG_WAITALL); // The socket returns the number of bytes read, 0 in case of connection lost, -1 in case of error

		if (result == -1 || result == 0)
		{
			printf("--- Disconnected ---\n");
            close(server_fd);
            sleep(3);
            
            pi_socket = connect_with_client();
        }
		else
		{
			extract_motion_command_vetor_from_socket_and_send_msg(msg);
		}
#else
	    struct sockaddr_in client_address;
	    socklen_t len = sizeof(struct sockaddr_in);
		result = recvfrom(pi_socket, (char *) msg, NUM_DOUBLES_IN_SOCKET_MOTION_COMMAND * sizeof(double), 0, (struct sockaddr *) &client_address, &len);

		if (result > 0)
			extract_motion_command_vetor_from_socket_and_send_msg(msg);
#endif
    }

   return (0);
}
