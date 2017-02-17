#ifndef PATH_PLANNER_INTERFACE_H
#define PATH_PLANNER_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "path_planner_messages.h"
#include <carmen/carmen.h>

void carmen_path_planner_subscribe_road_profile_message(carmen_path_planner_road_profile_message *message,
														void (*handler)(carmen_path_planner_road_profile_message*),
														carmen_subscribe_t subscribe_how);

#ifdef __cplusplus
}
#endif

#endif
