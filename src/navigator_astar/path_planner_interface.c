#include "path_planner_interface.h"

void carmen_path_planner_subscribe_road_profile_message(carmen_path_planner_road_profile_message *message,
														void (*handler)(carmen_path_planner_road_profile_message*),
														carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message((char*) CARMEN_PATH_PLANNER_ROAD_PROFILE_MESSAGE_NAME,
							 (char*) CARMEN_PATH_PLANNER_ROAD_PROFILE_MESSAGE_FMT,
							 message, sizeof (carmen_path_planner_road_profile_message),
							 (carmen_handler_t) handler,
							 subscribe_how);
}
