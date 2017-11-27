#include <carmen/carmen.h>
#include <carmen/road_mapper_interface.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/map_server_interface.h>

carmen_localize_ackerman_globalpos_message g_robot_pose;


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
localize_ackerman_globalpos_handler(carmen_localize_ackerman_globalpos_message *globalpos)
{
	g_robot_pose = *globalpos;
	printf("x: %lf | y: %lf\n", globalpos->pose.position.x, globalpos->pose.position.y);
}


void
map_server_localize_map_handler(carmen_map_server_localize_map_message *map_message)
{
	printf("********* %s %lf\n", map_message->host, map_message->timestamp);
}


void 
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("lane_detection: disconnected.\n");

		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                              //
// Initializations                                                                              //
//                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////


void
subscribe_messages()
{
	carmen_localize_ackerman_subscribe_globalpos_message(NULL,
		(carmen_handler_t) localize_ackerman_globalpos_handler,
		CARMEN_SUBSCRIBE_LATEST);

	carmen_map_server_subscribe_localize_map_message(NULL,
		(carmen_handler_t) map_server_localize_map_handler,
		CARMEN_SUBSCRIBE_LATEST);
}


int
main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	signal(SIGINT, shutdown_module);

	subscribe_messages();
	carmen_ipc_dispatch();

	return (0);
}
