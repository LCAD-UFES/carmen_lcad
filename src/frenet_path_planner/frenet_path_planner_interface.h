
#ifndef _CARMEN_FRENET_PATH_PLANNER_INTERFACE_H_
#define _CARMEN_FRENET_PATH_PLANNER_INTERFACE_H_

#include <carmen/carmen.h>
#include <carmen/frenet_path_planner_messages.h>


#ifdef __cplusplus
extern "C"
{
#endif

void
carmen_frenet_path_planner_subscribe_plan_message(carmen_frenet_path_planner_plan_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_frenet_path_planner_unsubscribe_plan_message(carmen_handler_t handler);

void
carmen_frenet_path_planner_define_messages();

void
carmen_frenet_path_planner_publish_plan_message(carmen_frenet_path_planner_plan_message *message);

#ifdef __cplusplus
}
#endif

#endif

// @}

