
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

	void
	carmen_route_planner_subscribe_predefined_route_message(carmen_route_planner_predefined_route_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

	void
	carmen_route_planner_unsubscribe_predefined_route_message(carmen_handler_t handler);

	void
	carmen_route_planner_set_predefined_route(char *predefined_route, int code);

	void
	carmen_route_planner_subscribe_destination_message(carmen_route_planner_destination_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

	void
	carmen_route_planner_unsubscribe_destination_message(carmen_handler_t handler);

	void
	carmen_route_planner_set_destination(char *destination, carmen_point_t destination_point);

	void
	carmen_route_planner_subscribe_pallet_and_destination_message(carmen_route_planner_pallet_and_destination_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

	void
	carmen_route_planner_unsubscribe_pallet_and_destination_message(carmen_handler_t handler);

	void
	carmen_route_planner_set_pallet_and_destination(char *pallet, carmen_point_t pallet_point, char *destination, carmen_point_t destination_point);

	void
	carmen_route_planner_subscribe_route_status_change_message(carmen_route_planner_route_status_change_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

	void
	carmen_route_planner_unsubscribe_route_status_change_message(carmen_handler_t handler);

	void
	change_route_status(int route_id, int status);

	void
	carmen_route_planner_subscribe_route_reload_message(carmen_route_planner_route_reload_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

	void
	carmen_route_planner_unsubscribe_route_reload_message(carmen_handler_t handler);

	void
	reload_all_routes(char *road_network_id);

	void
	carmen_route_planner_subscribe_route_list_request_message(carmen_route_planner_route_list_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

	void
	carmen_route_planner_unsubscribe_route_list_request_message(carmen_handler_t handler);

	void
	request_route_list(carmen_position_t center, double range);

	void
	carmen_route_planner_subscribe_route_list_response_message(carmen_route_planner_route_list_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

	void
	carmen_route_planner_unsubscribe_route_list_response_message(carmen_handler_t handler);

	void
	respond_route_list(carmen_position_t center, double range, int number_of_routes, route_t *routes);

	void
	carmen_route_planner_subscribe_node_ids_ahead_request_message(carmen_route_planner_node_ids_ahead_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

	void
	carmen_route_planner_unsubscribe_node_ids_ahead_request_message(carmen_handler_t handler);

	void
	request_node_ids_ahead(carmen_point_t initial_point, double meters_ahead);

	void
	carmen_route_planner_subscribe_node_ids_ahead_response_message(carmen_route_planner_node_ids_ahead_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

	void
	carmen_route_planner_unsubscribe_node_ids_ahead_response_message(carmen_handler_t handler);

	void
	respond_node_ids_ahead(carmen_point_t initial_point, double meters_ahead, int num_of_node_ids_ahead, int *node_ids_ahead);


#ifdef __cplusplus
}
#endif

#endif
