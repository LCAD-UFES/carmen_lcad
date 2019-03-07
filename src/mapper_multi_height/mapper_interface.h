#ifndef MAPPER_INTERFACE_H
#define MAPPER_INTERFACE_H

#include "mapper_messages.h"

#ifdef __cplusplus
extern "C"
{
#endif

void carmen_mapper_copy_map_from_message(carmen_map_t *current_map, carmen_mapper_map_message *online_map_message);

void
carmen_mapper_subscribe_map_message(carmen_mapper_map_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_mapper_subscribe_map_level1_message(carmen_mapper_map_message *message,
					carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_mapper_subscribe_virtual_laser_message(carmen_mapper_virtual_laser_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_mapper_subscribe_virtual_scan_message(carmen_mapper_virtual_scan_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_mapper_moving_objects_raw_map_subscribe_message(carmen_mapper_map_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

//void
//carmen_mapper_distance_map_subscribe_message(carmen_mapper_distance_map_message *message,
//			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_mapper_unsubscribe_map_message(carmen_handler_t handler);

void
carmen_mapper_unsubscribe_map_level1_message(carmen_handler_t handler);

void
carmen_mapper_unsubscribe_message(carmen_handler_t handler);

void
carmen_mapper_unsubscribe_virtual_laser_message(carmen_handler_t handler);

void
carmen_mapper_unsubscribe_virtual_scan_message(carmen_handler_t handler);

void
carmen_mapper_moving_objects_raw_map_unsubscribe_message(carmen_handler_t handler);

void
carmen_mapper_define_messages();

void
carmen_mapper_define_map_message();

void
carmen_mapper_define_virtual_laser_message();

void
carmen_mapper_define_virtual_scan_message();

void
carmen_mapper_publish_map_message(carmen_map_t *carmen_map, double timestamp);

void
carmen_mapper_publish_map_level_message(carmen_map_t *carmen_map, double timestamp, int height_level);

void
carmen_mapper_publish_virtual_laser_message(carmen_mapper_virtual_laser_message *virtual_laser_message, double timestamp);

void
carmen_mapper_publish_virtual_scan_message(carmen_mapper_virtual_scan_message *virtual_scan_message, double timestamp);

void
carmen_mapper_moving_objects_raw_map_publish_message(carmen_map_t *carmen_map, double timestamp);

//void
//carmen_mapper_publish_distance_map_message(carmen_mapper_distance_map *distance_map, double timestamp);

#ifdef __cplusplus
}
#endif

#endif


// @}

