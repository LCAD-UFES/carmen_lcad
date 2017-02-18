#include "udatmo_detector.h"
#include "udatmo_interface.h"

#include <carmen/carmen.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/rddf_interface.h>

void define_messages(void)
{
	IPC_RETURN_TYPE err = IPC_defineMsg(CARMEN_UDATMO_MOVING_OBSTACLES_MESSAGE_NAME,
										IPC_VARIABLE_LENGTH,
										CARMEN_UDATMO_MOVING_OBSTACLES_MESSAGE_FMT);

	carmen_test_ipc_exit(err, "Could not define", CARMEN_UDATMO_MOVING_OBSTACLES_MESSAGE_NAME);
}

static void globalpos_handler(carmen_localize_ackerman_globalpos_message *globalpos)
{
	carmen_udatmo_detector_update_globalpos(globalpos);
	carmen_udatmo_moving_obstacles_message *message = carmen_udatmo_detector_detect();
	carmen_udatmo_publish_moving_obstacles_message(message);
}

static void subscribe_messages(void)
{
	carmen_obstacle_distance_mapper_subscribe_message(NULL, (carmen_handler_t) carmen_udatmo_detector_update_distance_map, CARMEN_SUBSCRIBE_LATEST);	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) globalpos_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_rddf_subscribe_road_profile_message(NULL, (carmen_handler_t) carmen_udatmo_detector_update_rddf, CARMEN_SUBSCRIBE_LATEST);
}

static void shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("\nuDATMO: disconnected.\n");
		exit(0);
	}
}

int main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);

	define_messages();
	carmen_udatmo_detector_setup(argc, argv);
	subscribe_messages();

	carmen_ipc_dispatch();

	return 0;
}
