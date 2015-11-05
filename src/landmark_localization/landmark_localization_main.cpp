/*********************************************************
	---   Skeleton Module Application ---
 **********************************************************/

#include <carmen/carmen.h>
#include <carmen/landmark_localization_interface.h>

static carmen_pose_3D_t car_pose_g;
static carmen_pose_3D_t camera_pose_g;
static carmen_pose_3D_t sensor_board_pose_g;

/*********************************************************
		   --- Publishers ---
 **********************************************************/



/*********************************************************
		   --- Handlers ---
 **********************************************************/




void 
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("landmark_localization: disconnected.\n");

		exit(0);
	}
}

static int read_parameters(int argc, char **argv)
{
	int num_items, camera;
	char camera_string[256];

	if(argc > 2)
		camera = atoi(argv[1]);
	else
	{
		printf("Invalid number of parameters. Usage: ./landmark_localization [camera_number]");
		exit(0);
	}


	sprintf(camera_string, "%s%d", "camera", camera);

	carmen_param_t param_list[] =
	{
			{(char *) "car", 			  (char *) "x", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.x), 0, NULL},
			{(char *) "car", 			  (char *) "y", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.y), 0, NULL},
			{(char *) "car", 			  (char *) "z", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.position.z), 0, NULL},
			{(char *) "car", 			  (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.yaw), 0, NULL},
			{(char *) "car", 			  (char *) "pitch", CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.pitch), 0, NULL},
			{(char *) "car", 			  (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(car_pose_g.orientation.roll), 0, NULL},

			{(char *) "sensor_board_1", (char *) "x", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.position.x), 0, NULL},
			{(char *) "sensor_board_1", (char *) "y", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.position.y), 0, NULL},
			{(char *) "sensor_board_1", (char *) "z", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.position.z), 0, NULL},
			{(char *) "sensor_board_1", (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.orientation.yaw), 0, NULL},
			{(char *) "sensor_board_1", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.orientation.pitch), 0, NULL},
			{(char *) "sensor_board_1", (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(sensor_board_pose_g.orientation.roll), 0, NULL},

			{(char *) camera_string, 	  (char *) "x", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.x), 0, NULL},
			{(char *) camera_string,    (char *) "y", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.y), 0, NULL},
			{(char *) camera_string,    (char *) "z", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.z), 0, NULL},
			{(char *) camera_string,    (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.yaw), 0, NULL},
			{(char *) camera_string,    (char *) "pitch", CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.pitch), 0, NULL},
			{(char *) camera_string,    (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.roll), 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	return 0;
}

int 
main(int argc, char **argv) 
{
	/* Connect to IPC Server */
	carmen_ipc_initialize(argc, argv);

	/* Check the param server version */
	carmen_param_check_version(argv[0]);

	/* Read Parameters from .ini file */
	read_parameters(argc, argv);

	/* Register shutdown cleaner handler */
	signal(SIGINT, shutdown_module);

	/* Define messages that your module publishes */
	carmen_landmark_localization_define_messages();

	/* Subscribe to sensor messages */

	/* Loop forever waiting for messages */
	carmen_ipc_dispatch();

	return (0);
}
