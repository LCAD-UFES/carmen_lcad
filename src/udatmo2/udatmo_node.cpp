#include "udatmo_node.h"

#include "detector.h"

#include <carmen/rddf_interface.h>

using udatmo::Detector;

static Detector &detector() {
	static Detector *detector = NULL;
	if (detector == NULL)
		detector = new Detector();

	return *detector;
}

void carmen_udatmo_define_messages(void)
{
	IPC_RETURN_TYPE err = IPC_defineMsg(CARMEN_UDATMO_MOVING_OBSTACLES_MESSAGE_NAME,
										IPC_VARIABLE_LENGTH,
										CARMEN_UDATMO_MOVING_OBSTACLES_MESSAGE_FMT);

	carmen_test_ipc_exit(err, "Could not define", CARMEN_UDATMO_MOVING_OBSTACLES_MESSAGE_NAME);
}

void carmen_udatmo_install_params(int argc, char *argv[])
{
	detector().setup(argc, argv);
}

static void distance_map_handler(carmen_obstacle_distance_mapper_message *message)
{
	detector().update(message);
}

static void globalpos_handler(carmen_localize_ackerman_globalpos_message *message)
{
	detector().update(message);
}

static void rddf_handler(carmen_rddf_road_profile_message *message)
{
	detector().update(message);
}

void carmen_udatmo_subscribe_messages(void)
{
	carmen_obstacle_distance_mapper_subscribe_message(NULL, (carmen_handler_t) distance_map_handler, CARMEN_SUBSCRIBE_LATEST);	carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) globalpos_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_rddf_subscribe_road_profile_message(NULL, (carmen_handler_t) rddf_handler, CARMEN_SUBSCRIBE_LATEST);
}
