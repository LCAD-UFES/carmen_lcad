
#ifndef _CARMEN_ROUTE_PLANNER_INTERFACE_H_
#define _CARMEN_ROUTE_PLANNER_INTERFACE_H_

#include <carmen/carmen.h>
#include <carmen/route_planner_messages.h>

#ifdef __cplusplus
extern "C"
{
#endif

	void
	carmen_route_planner_subscribe_road_network_message(carmen_route_planner_road_network_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

	void
	carmen_route_planner_unsubscribe_road_network_message(carmen_handler_t handler);

	void
	carmen_route_planner_define_messages();

	void
	carmen_route_planner_publish_road_network_message(carmen_route_planner_road_network_message *route_planner_road_network_message);

#ifdef __cplusplus
}
#endif

#endif
