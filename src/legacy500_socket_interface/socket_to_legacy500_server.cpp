#include <carmen/carmen.h>
#include <control.h>
#include <sys/socket.h>
#include <netdb.h>

#define PORT 3457

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
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
extract_motion_command_vetor_from_socket_and_send_msg(float *msg)
{
	int size = (int)msg[0];
	carmen_ackerman_motion_command_t motion_command[size];

	for (int i = 0; i < size; i++)
	{
		motion_command[i].x     = msg[(i * 6) + 2];
		motion_command[i].y     = msg[(i * 6) + 3];
		motion_command[i].theta = msg[(i * 6) + 4];
		motion_command[i].v     = msg[(i * 6) + 5];
		motion_command[i].phi   = msg[(i * 6) + 6];
		motion_command[i].time  = msg[(i * 6) + 7];
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


// DEFINIR A MENSAGEM TODO

int
main(int argc, char **argv)
{
	float msg[9616]; // 2 + 6 * 200 * 8
	int result = 0;

	carmen_ipc_initialize(argc, argv);

	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_module);

	int pi_socket = connect_with_client();

	while (1)
	{
		result = recv(pi_socket, msg, 9616, MSG_WAITALL); // The socket returns the number of bytes read, 0 in case of connection lost, -1 in case of error

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
    }

   return (0);
}
