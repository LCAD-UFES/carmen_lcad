/*
 * map_server_interface.h
 *
 *  Created on: 26/09/2012
 *      Author: romulo
 */

#ifndef MAP_SERVER_INTERFACE_H_
#define MAP_SERVER_INTERFACE_H_

#include <carmen/carmen.h>
#include <carmen/localize_ackerman_core.h>
#include <carmen/localize_ackerman_likelihood_map.h>
#include <carmen/map_server_messages.h>
#include <carmen/mapper_messages.h>
#include <carmen/mapper_interface.h>


#ifdef __cplusplus
extern "C" {
#endif

//Subscribers
void carmen_map_server_subscribe_offline_map(
		carmen_map_server_offline_map_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);

void carmen_map_server_subscribe_multi_height_offline_map(
		carmen_map_server_offline_map_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how,
		int height_level);

void carmen_map_server_subscribe_road_map(
		carmen_map_server_road_map_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);

void carmen_map_server_subscribe_lane_map(
		carmen_map_server_lane_map *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);

void carmen_map_server_subscribe_compact_lane_map(
		carmen_map_server_compact_lane_map_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);

void carmen_map_server_subscribe_cost_map(
		carmen_map_server_cost_map *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);

void carmen_map_server_subscribe_multi_height_cost_map(
		carmen_map_server_cost_map *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how,
		int height_level);

void carmen_map_server_subscribe_compact_cost_map(
		carmen_map_server_compact_cost_map_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);

void carmen_map_server_subscribe_multi_height_compact_cost_map(
		carmen_map_server_compact_cost_map_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how,
		int height_level);

void carmen_map_server_subscribe_localize_map_message(
		carmen_map_server_localize_map_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how);


//Publishers
void carmen_map_server_publish_offline_map_message(carmen_map_t *carmen_map, double timestamp);

void carmen_map_server_publish_multi_height_offline_map_message(carmen_map_t *carmen_map, double timestamp, int height_level);

void carmen_map_server_publish_road_map_message(carmen_map_t *carmen_road_map, double timestamp);

void carmen_map_server_publish_lane_map_message(carmen_map_t *carmen_map, double timestamp);

void carmen_map_server_publish_compact_lane_map_message(carmen_compact_map_t *carmen_map, double timestamp);

void carmen_map_server_publish_cost_map_message(carmen_map_t *carmen_map, double timestamp);

void carmen_map_server_publish_multi_height_cost_map_message(carmen_map_t *carmen_map, double timestamp, int height_level);

void carmen_map_server_publish_compact_cost_map_message(carmen_compact_map_t *carmen_map, double timestamp);

void carmen_map_server_publish_multi_height_compact_cost_map_message(carmen_compact_map_t *carmen_map, double timestamp, int height_level);

void carmen_map_server_publish_localize_map_message(carmen_localize_ackerman_map_t* localize_map);

//Defines
void carmen_map_server_define_offline_map_message(void);

void carmen_map_server_define_multi_height_offline_map_message(int height_level);

void carmen_map_server_define_road_map_message(void);

void carmen_map_server_define_lane_map_message(void);

void carmen_map_server_define_compact_lane_map_message(void);

void carmen_map_server_define_cost_map_message(void);

void carmen_map_server_define_multi_height_cost_map_message(int height_level);

void carmen_map_server_define_compact_cost_map_message(void);

void carmen_map_server_define_multi_height_compact_cost_map_message(int height_level);

void carmen_map_server_define_localize_map_message(void);

//Other
//int carmen_map_server_get_current_offline_map(carmen_map_t *current_map);
void carmen_map_server_copy_offline_map_from_message(carmen_map_t *current_map, carmen_map_server_offline_map_message *offline_map_message);

void carmen_map_server_localize_map_message_to_localize_map(carmen_map_server_localize_map_message *message, carmen_localize_ackerman_map_t* localize_map);

void carmen_cpy_compact_cost_message_to_compact_map(carmen_compact_map_t* compact_cost_map,carmen_map_server_compact_cost_map_message* message);

void carmen_cpy_compact_lane_message_to_compact_map(carmen_compact_map_t* compact_lane_map, carmen_map_server_compact_lane_map_message* message);

#ifdef __cplusplus
}
#endif

#endif /* MAP_SERVER_INTERFACE_H_ */
