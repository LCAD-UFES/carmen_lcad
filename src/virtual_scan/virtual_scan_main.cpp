#include <carmen/carmen.h>
#include <carmen/virtual_scan_interface.h>
#include <carmen/global_graphics.h>
#include "virtual_scan.h"


double d_max;

carmen_mapper_virtual_laser_message virtual_laser_message;

///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_virtual_scan(carmen_mapper_virtual_scan_message *virtual_scan)
{
	virtual_laser_message.host = carmen_get_host();
	virtual_laser_message.num_positions = virtual_scan->num_points;
	virtual_laser_message.positions = virtual_scan->points;
	virtual_laser_message.colors = (char *) malloc(virtual_laser_message.num_positions * sizeof(char));
	for (int i = 0; i < virtual_laser_message.num_positions; i++)
		virtual_laser_message.colors[i] = CARMEN_RED;

	carmen_mapper_publish_virtual_laser_message(&virtual_laser_message, carmen_get_time());

	free(virtual_laser_message.colors);
}


///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_mapper_virtual_scan_message_handler(carmen_mapper_virtual_scan_message *message)
{
	detect_and_track_moving_objects(message);
	publish_virtual_scan(message);
}


static void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();

		printf("\nVirtual Scan: disconnected.\n");
		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Initializations                                                                           //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_virtual_scan_install_params(int argc, char *argv[])
{
	carmen_param_t laser_param_list[] =
	{
		{(char *) "polar_slam", (char *) "laser_max_range", CARMEN_PARAM_DOUBLE, &d_max, 0, NULL}
	};

	carmen_param_install_params(argc, argv, laser_param_list, sizeof(laser_param_list) / sizeof(laser_param_list[0]));
}


void
carmen_virtual_scan_define_messages()
{

}


void
carmen_virtual_scan_subscribe_messages()
{
	carmen_mapper_subscribe_virtual_scan_message(NULL, (carmen_handler_t) carmen_mapper_virtual_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	carmen_virtual_scan_install_params(argc, argv);
	carmen_virtual_scan_define_messages();

	signal(SIGINT, shutdown_module);
	carmen_virtual_scan_subscribe_messages();

	carmen_ipc_dispatch();

	return (0);
}
