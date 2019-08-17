#include <carmen/carmen.h>
#include <carmen/carmen_gps.h>
#include <control.h>
#include <sys/socket.h>
#include <netdb.h>

#define	NUM_MOTION_COMMANDS_PER_VECTOR	200

#define DEFAULT_PORT_NUMBER 3458

static int port_number = DEFAULT_PORT_NUMBER;

typedef enum selected_bit_enum
{
	BIT_NONE = 0,
	BIT_SOCKET = 1,
} selected_bit_t;
static selected_bit_t selected_bit = BIT_NONE;

static const char * const help_msg = "Usage: ./socket_to_odometry_client [options]\n"
		"This application receives odometry from the Integration Server via UDP\n"
		"Available options:\n"
		"-port_number <port>\tThis process is listening for UDP datagrams at this port number (default: 3458)\n"
		"-bit SOCKET\t\tBuilt-in tests\n"
		"-help\t\t\tPrint this help message\n";

int
stablished_connection_with_server()
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
	address.sin_port = htons(port_number);

	// Forcefully attaching socket to the port defined
	if (bind(new_socket, (struct sockaddr *) &address, sizeof(address)) < 0)
	{
		perror("--- Bind Failed ---\n");
		return (-1);
	}
	printf("--- Bind successful! ---\n");

	return (new_socket);
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

	odometry.v   = array[0];
	odometry.phi = carmen_normalize_theta(array[1]);

	odometry.timestamp = carmen_get_time();
	odometry.host  = carmen_get_host();

	//printf("Odometry: %lf [m/s] %lf [rad]\n", odometry.v, odometry.phi);

	err = IPC_publishData(CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME, &odometry);
	carmen_test_ipc(err, "Could not publish ford_escape_hybrid message named carmen_robot_ackerman_velocity_message", CARMEN_ROBOT_ACKERMAN_VELOCITY_NAME);
}

//#define TRUE_POSE_CORRECTION

#ifdef TRUE_POSE_CORRECTION
// Displacement between the AHRS pose (lat, lon, alt, theta) and the middle of rear axle along the airplane longitudinal axis
#define LONGITUDINAL_OFFSET     4.0 // [m]

// Heading bias: the heading is in the range [-pi, pi]  and the zero is aligned to the y-axis (North)
#define HEADING_BIAS           	.0  // [rad]

static void
publish_simulator_ackerman_external_truepose_message(double *array)
{
	carmen_simulator_ackerman_truepos_message truepose;

	truepose.v     			= array[0];
	truepose.phi   			= carmen_normalize_theta(array[1]);
	truepose.truepose.theta	= carmen_normalize_theta(array[2]-HEADING_BIAS);

	double altitude	 = array[3];
	double latitude  = array[4];
	double longitude = array[5];

	//printf("True pose (v, phi, theta): %lf [m/s] %lf [rad] %lf [rad]\n", truepose.v, truepose.phi, truepose.truepose.theta);
	//printf("True pose (lat, lon, alt): %lf [deg] %lf [deg] %lf [m]\n", latitude, longitude, altitude);

	// Transformando o z utilizando como altitude o sea_level
	Gdc_Coord_3d gdc = Gdc_Coord_3d(latitude, longitude, altitude);

	Utm_Coord_3d utm;
	Gdc_To_Utm_Converter::Init();
	Gdc_To_Utm_Converter::Convert(gdc, utm);

	truepose.truepose.x =   utm.y - LONGITUDINAL_OFFSET*cos(truepose.truepose.theta);
	truepose.truepose.y = -(utm.x - LONGITUDINAL_OFFSET*sin(truepose.truepose.theta));

	//printf("True pose (x, y): %lf [m] %lf [m]\n", truepose.truepose.x, truepose.truepose.y);

	truepose.odometrypose = truepose.truepose;

	truepose.timestamp = carmen_get_time();
	truepose.host  = carmen_get_host();

	carmen_simulator_ackerman_publish_external_truepose(&truepose);
}

#else
static void
publish_simulator_ackerman_external_truepose_message(double *array)
{
	carmen_simulator_ackerman_truepos_message truepose;

	truepose.v     			= array[0];
	truepose.phi   			= carmen_normalize_theta(array[1]);
	truepose.truepose.theta	= carmen_normalize_theta(array[2]);

	double altitude	 = array[3];
	double latitude  = array[4];
	double longitude = array[5];

	//printf("True pose (v, phi, theta): %lf [m/s] %lf [rad] %lf [rad]\n", truepose.v, truepose.phi, truepose.truepose.theta);
	//printf("True pose (lat, lon, alt): %lf [deg] %lf [deg] %lf [m]\n", latitude, longitude, altitude);

	// Transformando o z utilizando como altitude o sea_level
	Gdc_Coord_3d gdc = Gdc_Coord_3d(latitude, longitude, altitude);

	Utm_Coord_3d utm;
	Gdc_To_Utm_Converter::Init();
	Gdc_To_Utm_Converter::Convert(gdc, utm);

	truepose.truepose.x	= utm.y;
	truepose.truepose.y	= -utm.x;

	//printf("True pose (x, y): %lf [m] %lf [m]\n", truepose.truepose.x, truepose.truepose.y);

	truepose.odometrypose = truepose.truepose;

	truepose.timestamp = carmen_get_time();
	truepose.host  = carmen_get_host();

	carmen_simulator_ackerman_publish_external_truepose(&truepose);
}
#endif

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
// Built-in tests                                                                               //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


static void
socket_test(void)
{
	double array[6];
	int local_socket = stablished_connection_with_server();
	
	int result = recvfrom(local_socket, (void *) array, sizeof(array), 0, NULL, NULL);

	double v     = array[0];
	double phi   = array[1];
	double theta = carmen_normalize_theta(array[2]);

	double altitude	 = array[3];
	double latitude  = array[4];
	double longitude = array[5];

	double normalized_theta = carmen_normalize_theta(theta);
	Gdc_Coord_3d gdc = Gdc_Coord_3d(latitude, longitude, altitude);

	Utm_Coord_3d utm;
	Gdc_To_Utm_Converter::Init();
	Gdc_To_Utm_Converter::Convert(gdc, utm);

	printf("Received %d bytes\n", result);
	printf("  long = %f deg (%f UTM)\n", longitude, utm.y);
	printf("  lat = %f deg (%f UTM)\n", latitude, -utm.x);
	printf("  phi = %f rad (%f deg)\n", phi, -carmen_radians_to_degrees(phi));
	printf("  v = %f m/s\n", v);
	printf("  theta = %f rad (%f deg)\n", theta, -carmen_radians_to_degrees(theta));
	printf("  normalized theta = %f rad (%f deg)\n", normalized_theta, -carmen_radians_to_degrees(normalized_theta));
	printf("  alt = %f m\n", altitude);
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
		socket_test();
		printf("BIT socket done.\n");
		break;

	default:
		printf("ERROR: Invalid buint-in test selected: %d.\n", selected_bit);
		break;
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

void parse_args(int argc, char **argv)
{
	for (int i = 1; i < argc; i++)
	{
		bool valid_arg = false;
		
		if (strcmp(argv[i], "-port_number") == 0)
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
		double array[10];
		int result = 0;

		carmen_ipc_initialize(argc, argv);
		carmen_param_check_version(argv[0]);
		signal(SIGINT, shutdown_module);
		if (initialize_ipc() < 0)
			carmen_die("Error in initializing ipc...\n");

		int pi_socket = stablished_connection_with_server();

		while (1)
		{
			result = recvfrom(pi_socket, (void *) array, sizeof(array), 0, NULL, NULL);
			
			//printf("Received %d bytes\n", result);
			//printf("  v = %f m/s\n", array[0]);
			//printf("  phi = %f rad\n", array[1]);
			
			if (result > 0)
			{
				publish_robot_ackerman_velocity_message(array);
				publish_simulator_ackerman_external_truepose_message(array);
			}
		}
	}
	return (0);
}
