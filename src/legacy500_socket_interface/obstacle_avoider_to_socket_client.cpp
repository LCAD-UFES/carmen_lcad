#include <carmen/carmen.h>
#include <control.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#define	NUM_MOTION_COMMANDS_PER_VECTOR	200
#define NUM_DOUBLES_IN_SOCKET_MOTION_COMMAND (1 + NUM_MOTION_COMMANDS_PER_VECTOR * 6)

#define DEFAULT_PORT_NUMBER 3457
#define DEFAULT_IP_ADDRESS "127.0.0.1"

//#define MIN(a,b) ((a <= b) ? a : b)
//#define MAX(a,b) ((a >= b) ? a : b)
//#define RAD2DEG(a) (a / 0.01745329252)
//#define DEG2RAD(a) (a * 0.01745329252)

static char *ip_address = (char *) DEFAULT_IP_ADDRESS;
static int port_number = DEFAULT_PORT_NUMBER;
typedef enum selected_bit_enum
{
	BIT_NONE = 0,
	BIT_SOCKET = 1,
	BIT_VELOCITY = 2,
	BIT_STEERING_ANGLE = 3,
	BIT_COMMAND_OVERRIDE = 4,
	BIT_MANUAL = 5,
	BIT_EMERGENCY_STOP = 6,
	BIT_MAX_VELOCITY = 7,
} selected_bit_t;
static selected_bit_t selected_bit = BIT_NONE;

static const char * const help_msg = "Usage: ./obstacle_avoider_to_socket_client [options]\n"
			"This application sends motion commands to the Integration Server via UDP\n"
			"Available options:\n"
			"-ip_address <ip>\tIntegration Server IP address (default: 127.0.0.1)\n"
			"-port_number <port>\tIntegration Server is listening for UDP datagrams at this port number (default: 3457)\n"
			"-bit SOCKET|VELOCITY|STEERING_ANGLE|COMMAND_OVERRIDE|MANUAL\tBuilt-in tests\n"
			"-help\t\t\tPrint this help message\n";


int
stablished_connection_with_server(struct sockaddr_in *client_address)
{
	int new_socket;

	// Creating socket file descriptor
	if ((new_socket = socket(AF_INET, SOCK_DGRAM, 0)) == 0)
	{
		perror("--- Socket Failed ---\n");
		return (-1);
	}

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
static void
socket_test(int size, double xmin, double xmax, double ymin, double ymax, double thetamin, double thetamax, double vmin, double vmax, double phimin, double phimax, double timemin, double timemax)
{
	double array[NUM_DOUBLES_IN_SOCKET_MOTION_COMMAND];
	double x = xmin, deltax = (xmax - xmin)/(size - 1);
	double y = ymin, deltay = (ymax - ymin)/(size - 1);
	double theta = thetamin, deltatheta = (thetamax - thetamin)/(size - 1);
	double v = vmin, deltav = (vmax - vmin)/(size - 1);
	double phi = phimin, deltaphi = (phimax - phimin)/(size - 1);
	double time = timemin, deltatime = (timemax - timemin)/(size - 1);

	array[0] = size;
	printf ("number_of_motion_commands: %d\n", (int) array[0]);
	for (int i = 0; i < size; i++)
	{
		array[(i * 6) + 1] = x;
		array[(i * 6) + 2] = y;
		array[(i * 6) + 3] = theta;
		array[(i * 6) + 4] = v;
		array[(i * 6) + 5] = phi;
		array[(i * 6) + 6] = time;
		printf ("i: %d >>> x: %lf [m] y: %lf [m] theta: %lf [rad] (%lf [deg]) v: %lf [m/s] phi: %lf [rad] (%lf [deg]) time: %lf [s]\n", i, x, y, theta, carmen_radians_to_degrees(theta), v, phi, carmen_radians_to_degrees(phi), time);
		x += deltax;
		y += deltay;
		theta += deltatheta;
		v += deltav;
		phi += deltaphi;
		time += deltatime;

	}

	send_motion_command_via_socket(array);
}

static void
velocity_test(void)
{
	double array[NUM_DOUBLES_IN_SOCKET_MOTION_COMMAND];
	const double speed[] = {2.5, 5.0, 7.5, 10.0, 5.0, 5.0, 2.5, 2.5, 0.0};
	const int size = sizeof(speed)/sizeof(double);
	int i;
	double x = .0, y = .0, theta = .0, phi = .0, time = 20.0;

	array[0] = size;
	printf ("number_of_motion_commands: %d\n", (int) array[0]);
	for (i=0; i < size; i++)
	{
		array[(i * 6) + 1] = x;
		array[(i * 6) + 2] = y;
		array[(i * 6) + 3] = theta;
		array[(i * 6) + 4] = speed[i];
		array[(i * 6) + 5] = phi;
		array[(i * 6) + 6] = time;
		printf ("i: %d >>> x: %lf [m] y: %lf [m] theta: %lf [rad] (%lf [deg]) v: %lf [m/s] phi: %lf [rad] (%lf [deg]) time: %lf [s]\n", i, x, y, theta, carmen_radians_to_degrees(theta), speed[i], phi, carmen_radians_to_degrees(phi), time);
	}

	send_motion_command_via_socket(array);
}

static void
steering_test(void)
{
	double array[NUM_DOUBLES_IN_SOCKET_MOTION_COMMAND];
	const double phi_deg[] = {-30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 0.0};
	const int size = sizeof(phi_deg)/sizeof(double);
	int i;

	double x = .0, y = .0, theta = .0, speed = .1, time = 10.0;

	array[0] = size;
	printf ("number_of_motion_commands: %d\n", (int) array[0]);
	for (i=0; i < size; i++)
	{
		array[(i * 6) + 1] = x;
		array[(i * 6) + 2] = y;
		array[(i * 6) + 3] = theta;
		array[(i * 6) + 4] = speed;
		array[(i * 6) + 5] = carmen_degrees_to_radians(phi_deg[i]);
		array[(i * 6) + 6] = time;
		printf ("i: %d >>> x: %lf [m] y: %lf [m] theta: %lf [rad] (%lf [deg]) v: %lf [m/s] phi: %lf [rad] (%lf [deg]) time: %lf [s]\n", i, x, y, theta, carmen_radians_to_degrees(theta), speed, carmen_degrees_to_radians(phi_deg[i]), phi_deg[i], time);
	}

	send_motion_command_via_socket(array);
}

static void
max_velocity_test(double max_speed)
{
	double array[NUM_DOUBLES_IN_SOCKET_MOTION_COMMAND];
	double x = .0, y = .0, theta = .0, speed = max_speed, phi = .0, time = 30.0;
	int i = 0;

	// Send command to maintain 10 m/s
	array[0] = 1.0;
	array[1] = x;
	array[2] = y;
	array[3] = theta;
	array[4] = speed;
	array[5] = phi;
	array[6] = time;
	printf ("i: %d >>> x: %lf [m] y: %lf [m] theta: %lf [rad] (%lf [deg]) v: %lf [m/s] phi: %lf [rad] (%lf [deg]) time: %lf [s]\n", i, x, y, theta, carmen_radians_to_degrees(theta), speed, phi, carmen_radians_to_degrees(phi), time);
	send_motion_command_via_socket(array);
}

static void
emergency_stop_test(double max_speed, int local_port)
{
	// Send command to maintain max_speed
	max_velocity_test(max_speed);

	// Bind local socket to listen odometry
	int local_socket;
	struct sockaddr_in local_address;

	local_address.sin_family = AF_INET;
	local_address.sin_addr.s_addr = INADDR_ANY;
	local_address.sin_port = htons(local_port);
	printf("Creating socket to listen to port %d...\n", local_port);
	local_socket = socket(AF_INET, SOCK_DGRAM, 0);
	printf("Binding...\n");
	bind(local_socket, (struct sockaddr *) &local_address, sizeof(local_address));
	
	// Receive odometry data
	double odometry_speed = 0.0;
	double odometry_array[5];
	int result;
	printf("Waiting velocity reach %.1f...\n", max_speed);
	while((result = recvfrom(local_socket, (char *) odometry_array, sizeof(odometry_array), 0, NULL, NULL)) > 0)
	{
		odometry_speed = odometry_array[0];
		if (odometry_speed > max_speed - 0.25)
		{
			printf("Velocity = %.1f. Reducing to zero now.\n", odometry_speed);
			break;
		}
	}

	// Send command to stop
	max_velocity_test(0.0);
}

static void
command_override_test(int number_motion_commands, double override_rate)
{
	double max_sleep_time = 1e6/override_rate;
	struct timeval st, et;

	for (int i = 0; i < number_motion_commands; i++)
	{
		gettimeofday(&st,NULL);
		
		printf("overrides: %d override_rate: %lf [Hz]\n", i, override_rate);

		socket_test(10, i, i, -5.0, 5.0, -3.0, 3.0, 0.0, 10.0, -2.0, 2.0, 1.0, 1.0);
		
		gettimeofday(&et,NULL);

		double elapsed_time = ((et.tv_sec - st.tv_sec) * 1000000) + (et.tv_usec - st.tv_usec);

		double sleep_time = MAX(.0, max_sleep_time - elapsed_time);

		usleep(sleep_time);

		printf("sleep_time: %lf [s] elapsed_time: %lf\n", sleep_time/1e6, elapsed_time/1e6);
	}
}

static void
manual_test()
{
	double array[NUM_DOUBLES_IN_SOCKET_MOTION_COMMAND];
	double v = .0;
	double phi = .0;
	double time = .0;
	while (1)
	{
		printf("Enter desired speed and steering angle:\n");
		printf("v = ? [m/s]\n");
		scanf("%lf", &v);
		printf("phi = ? [deg]\n");
		scanf("%lf", &phi);
		printf("time = ? [s]\n");
		scanf("%lf", &time);

		// Number of commands
		array[0] = 1.0;
		
		// Just one command
		array[1] = 0.0;
		array[2] = 0.0;
		array[3] = 0.0;
		array[4] = v;
		array[5] = carmen_degrees_to_radians(phi);
		array[6] = time;

		/*
		// Number of commands
		array[0] = 3.0;
		
		// First command
		array[1] = 3.0;
		array[2] = 3.0;
		array[3] = 3.0;
		array[4] = v;
		array[5] = DEG2RAD(phi);
		array[6] = 5.0;

		// Second command
		array[7] = 2.0;
		array[8] = 2.0;
		array[9] = 2.0;
		array[10] = v;
		array[11] = DEG2RAD(phi);
		array[12] = 5.0;

		// Third command
		array[13] = 1.0;
		array[14] = 1.0;
		array[15] = 1.0;
		array[16] = v;
		array[17] = DEG2RAD(phi);
		array[18] = 5.0;*/

		send_motion_command_via_socket(array);
		printf("Command sent successfully.\n");
	}
}


static void
execute_test(void)
{
	switch (selected_bit)
	{
	case BIT_NONE:
		printf("Nothing to do.\n");
		break;

	case BIT_SOCKET:
		socket_test(10, -10.0, 10.0, -5.0, 5.0, -3.0, 3.0, 0.0, 10.0, -2.0, 2.0, 1.0, 1.0);
		printf("BIT socket done.\n");
		break;

	case BIT_VELOCITY:
		velocity_test();
		printf("BIT velocity done.\n");
		break;

	case BIT_STEERING_ANGLE:
		steering_test();
		printf("BIT steering angle done.\n");
		break;

	case BIT_EMERGENCY_STOP:
		emergency_stop_test(10.0, 3458);
		printf("BIT emergency stop done.\n");
		break;

	case BIT_MAX_VELOCITY:
		max_velocity_test(10.0);
		printf("BIT max velocity done.\n");
		break;

	case BIT_COMMAND_OVERRIDE:
		command_override_test(10, 20.0);
		printf("BIT command override done.\n");
		break;

	case BIT_MANUAL:
		manual_test();
		printf("BIT manual done.\n");
		break;

	default:
		printf("ERROR: Invalid buint-in test selected: %d.\n", selected_bit);
		break;
	}
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
			if (argc > ++i)
			{
				if (strcmp(argv[i], "SOCKET") == 0)
				{
					selected_bit = BIT_SOCKET;
					printf ("bit: SOCKET\n");
					valid_arg = true;
				}
				else if (strcmp(argv[i], "VELOCITY") == 0)
				{
					selected_bit = BIT_VELOCITY;
					printf ("bit: VELOCITY\n");
					valid_arg = true;
				}
				else if (strcmp(argv[i], "STEERING_ANGLE") == 0)
				{
					selected_bit = BIT_STEERING_ANGLE;
					printf ("bit: STEERING_ANGLE\n");
					valid_arg = true;
				}
				else if (strcmp(argv[i], "EMERGENCY_STOP") == 0)
				{
					selected_bit = BIT_EMERGENCY_STOP;
					printf ("bit: EMERGENCY_STOP\n");
					valid_arg = true;
				}
				else if (strcmp(argv[i], "MAX_VELOCITY") == 0)
				{
					selected_bit = BIT_MAX_VELOCITY;
					printf ("bit: MAX_VELOCITY\n");
					valid_arg = true;
				}
				else if (strcmp(argv[i], "COMMAND_OVERRIDE") == 0)
				{
					selected_bit = BIT_COMMAND_OVERRIDE;
					printf ("bit: COMMAND_OVERRIDE\n");
					valid_arg = true;
				}
				else if (strcmp(argv[i], "MANUAL") == 0)
				{
					selected_bit = BIT_MANUAL;
					printf ("bit: MANUAL\n");
					valid_arg = true;
				}
			}
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
	if (selected_bit != BIT_NONE)
	{
		execute_test();
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
