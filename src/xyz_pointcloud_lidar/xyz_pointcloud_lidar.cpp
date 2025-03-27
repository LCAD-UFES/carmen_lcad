#include "xyz_pointcloud_lidar_messages.h"

#include <carmen/carmen.h>
#include <carmen/ipc.h>


//////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////
//																								//
// Publishers																					//
//																								//
//////////////////////////////////////////////////////////////////////////////////////////////////

void
publish_fake_xyz_lidar()
{
	static int first = 1;
	static char* ptr = "a";
	static carmen_xyz_pointcloud_lidar_message xyz_pointcloud_message;
    xyz_pointcloud_message.num_of_points = 100;
    xyz_pointcloud_message.timestamp = 0.0;
    xyz_pointcloud_message.host = ptr;
    xyz_pointcloud_message.time_spent_by_each_point = 0.000001;

	IPC_RETURN_TYPE err;

	// if (first)
	// {
	// 	xyz_pointcloud_message.points = (carmen_vector_3D_t *) malloc (xyz_pointcloud_message.num_of_points * sizeof(carmen_vector_3D_t));
	// 	first = 0;
	// }

	// for(int i = 0; i < xyz_pointcloud_message.num_of_points; i++)
    // {
    //     if (i < 10000) {
    //         xyz_pointcloud_message.points[i].x = (i % 100) / 100.0;
    //         xyz_pointcloud_message.points[i].y = (i / 100) / 100.0;
    //         xyz_pointcloud_message.points[i].z = 1;
    //     } else if (i < 20000) {
    //         xyz_pointcloud_message.points[i].x = (i % 100) / 100.0;
    //         xyz_pointcloud_message.points[i].y = 1;
    //         xyz_pointcloud_message.points[i].z = (i / 100) / 100.0;
    //     } else {
    //         xyz_pointcloud_message.points[i].x = 1;
    //         xyz_pointcloud_message.points[i].y = (i / 100) / 100.0;
    //         xyz_pointcloud_message.points[i].z = (i % 100) / 100.0;
    //     }
    // }

	err = IPC_publishData(CARMEN_XYZ_POINTCLOUD_LIDAR_MESSAGE_NAME, &xyz_pointcloud_message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_XYZ_POINTCLOUD_LIDAR_MESSAGE_NAME);
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
define_messages()
{
	IPC_RETURN_TYPE err;

    err = IPC_defineMsg(CARMEN_XYZ_POINTCLOUD_LIDAR_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_XYZ_POINTCLOUD_LIDAR_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_XYZ_POINTCLOUD_LIDAR_MESSAGE_NAME);
}

int 
main(int argc, char **argv)
{
    
	signal(SIGINT, shutdown_module);
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

    define_messages();

	carmen_ipc_addPeriodicTimer(1.0 / 30.0, (TIMER_HANDLER_TYPE) publish_fake_xyz_lidar, NULL);
    carmen_ipc_dispatch();
    return 0;
}
