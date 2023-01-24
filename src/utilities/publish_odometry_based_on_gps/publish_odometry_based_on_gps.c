#include <carmen/carmen.h>
#include <carmen/gps_xyz_interface.h>
#include <carmen/base_ackerman_messages.h>
#include <carmen/base_ackerman_interface.h>


int gps_to_use = 1;
double max_velocity = 0.0;
carmen_base_ackerman_odometry_message *last_odometry = NULL;
char *outfile = NULL;
FILE *fp = NULL;

void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		if (outfile)
		{
			free(outfile);
			if (fp)
				free(fp);
		}

		carmen_ipc_disconnect();
		printf("publish_odometry_based_on_gps: disconnected\n");
		exit(0);
	}
}


static int 
initialize_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, IPC_VARIABLE_LENGTH, CARMEN_BASE_ACKERMAN_ODOMETRY_FMT);
	carmen_test_ipc_exit(err, "Could not define message", CARMEN_BASE_ACKERMAN_ODOMETRY_NAME);

	return 0;
}


void
publish_odometry(double x, double y, double theta, double v, double phi, double timestamp)
{
	IPC_RETURN_TYPE err = IPC_OK;
	static carmen_base_ackerman_odometry_message odometry;

	odometry.x = x;
	odometry.y = y;
	odometry.theta = theta;
	odometry.v = v;
	odometry.phi = phi;
	odometry.timestamp = timestamp;

	err = IPC_publishData(CARMEN_BASE_ACKERMAN_ODOMETRY_NAME, &odometry);
	carmen_test_ipc(err, "Could not publish base_odometry_message", CARMEN_BASE_ACKERMAN_ODOMETRY_NAME);
}


void
base_ackerman_odometry_handler(carmen_base_ackerman_odometry_message *msg)
{
	last_odometry = msg;
}


void
gps_xyz_handler(carmen_gps_xyz_message *message)
{
	printf("%d\n", message->nr );
	if (message->nr != gps_to_use)
		return;
printf("%d\n", message->nr );
	static int first_time = 1;
	double v, theta;
	static double last_v, last_theta, last_x, last_y, last_timestamp;

	if (first_time)
	{
		last_v = 0.0;
		last_theta = 0.0;
		last_x = message->x;
		last_y = message->y;
		last_timestamp = message->timestamp;

		first_time = 0;
		return;
	}

	v = sqrt((message->x - last_x)*(message->x - last_x) + (message->y - last_y)*(message->y - last_y)) / (message->timestamp - last_timestamp);
	theta = atan2(message->y - last_y, message->x - last_x);

	if (v > max_velocity)
	{
		v = last_v;
		theta = last_theta;
	}
	else
	{
		last_v = v;
		last_theta = theta;
		last_x = message->x;
		last_y = message->y;
		last_timestamp = message->timestamp;
	}

	if (last_odometry && outfile)
	{
		if (!fp)
			fp = fopen(outfile, "w");
		fprintf(fp, "%lf\t%lf\t%lf\t%lf\n", v, last_odometry->v, theta, last_odometry->theta);
	}

	publish_odometry(last_x, last_y, theta, v, 0.0, message->timestamp);
}


static void 
read_parameters(int argc, char *argv[])
{
	carmen_param_t param_list[]= 
	{
		{"robot", "max_velocity", CARMEN_PARAM_DOUBLE, &max_velocity, 1, NULL}
	};
	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
}


int
main(int argc, char **argv)
{
	gps_to_use = atoi(argv[1]);

	if (argc == 3)
	{
		outfile = (char*) malloc(strlen(argv[2])*sizeof(char));
		strcpy(outfile, argv[1]);
	}

	signal(SIGINT, shutdown_module);
	carmen_ipc_initialize(argc, argv);

	read_parameters(argc, argv);
	initialize_message();

	carmen_gps_xyz_subscribe_message(NULL, (carmen_handler_t) gps_xyz_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_base_ackerman_subscribe_odometry_message(NULL, (carmen_handler_t) base_ackerman_odometry_handler, CARMEN_SUBSCRIBE_LATEST);

	fprintf(stderr, "INFO: *******************************************\n");
	fprintf(stderr, "INFO: ********** ODOMETRY BASED ON GPS **********\n");
	fprintf(stderr, "INFO: *******************************************\n");

	IPC_dispatch();

	return 0;
}

