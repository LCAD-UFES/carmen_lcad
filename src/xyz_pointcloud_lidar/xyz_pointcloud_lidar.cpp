#include "xyz_pointcloud_lidar_interface.h"

#include <carmen/carmen.h>
#include <carmen/ipc.h>


//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Publishers																					//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////

static void
publish_fake_xyz_lidar()
{
	static bool first_time = true;
	static carmen_xyz_pointcloud_lidar_message variable_msg;
	int num_points = 30000;

	if (first_time)
	{
		variable_msg.points = (carmen_vector_3D_t *) malloc (num_points * sizeof(carmen_vector_3D_t));

		for (int i = 0 ; i < num_points; i++)
		{
			if (i < num_points/3) {
				variable_msg.points[i].x = 1.0;
				variable_msg.points[i].y = ((i % 100) / 100.0);
				variable_msg.points[i].z = ((i / 100) / 100.0);
			} else if (i < 2*num_points/3) {
				variable_msg.points[i].x = ((i % 100) / 100.0);
				variable_msg.points[i].y = 1.0;
				variable_msg.points[i].z = (((i - num_points/3) / 100) / 100.0);
			} else {
				variable_msg.points[i].x = ((i % 100) / 100.0);
				variable_msg.points[i].y = (((i - 2*num_points/3) / 100) / 100.0);
				variable_msg.points[i].z = 1.0;
			}
		}
		variable_msg.host = carmen_get_host();
		variable_msg.num_of_points = num_points;
		variable_msg.timestamp = 1.0;
		first_time = false;
	}

	carmen_xyz_pointcloud_lidar_publish_xyz_pointcloud_message(&variable_msg, 0);
	carmen_xyz_pointcloud_lidar_publish_xyz_pointcloud_message(&variable_msg, 1);
	carmen_xyz_pointcloud_lidar_publish_xyz_pointcloud_message(&variable_msg, 2);
	carmen_xyz_pointcloud_lidar_publish_xyz_pointcloud_message(&variable_msg, 3);
}

//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Handlers																						//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////





//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Inicializations																				//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////

static void 
shutdown_module(int sig)
{
	(void) sig;

	static int done = 0;

	if (!done)
	{
		carmen_ipc_disconnect();
		printf("xyz_pointcloud_lidar disconnected from IPC.\n");
		fflush(stdout);

	}

	exit(0);
}


static void
carmen_xyz_pointcloud_define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_XYZ_POINTCLOUD_LIDAR_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_XYZ_POINTCLOUD_LIDAR_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_XYZ_POINTCLOUD_LIDAR_MESSAGE_NAME);

	char message_name[64];

	for (int i = 0; i < 4; i++)
	{
		carmen_xyz_pointcloud_lidar_create_variable_xyz_pointcloud_message_name(i, message_name);
		err = IPC_defineMsg(message_name, IPC_VARIABLE_LENGTH, CARMEN_XYZ_POINTCLOUD_LIDAR_MESSAGE_FMT);
		carmen_test_ipc_exit(err, "Could not define", message_name);
	}
}

int 
main(int argc, char **argv)
{	
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_module);

    carmen_xyz_pointcloud_define_messages();

	carmen_ipc_addPeriodicTimer(1.0 / 10.0, (TIMER_HANDLER_TYPE) publish_fake_xyz_lidar, NULL);
	
    carmen_ipc_dispatch();
    return 0;
}
