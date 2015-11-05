#ifndef GRID_MAPPING_INTERFACE_H
#define GRID_MAPPING_INTERFACE_H

#include "grid_mapping_messages.h"

#ifdef __cplusplus
extern "C"
{
#endif

void carmen_grid_mapping_copy_map_from_message(carmen_map_t *current_map, carmen_grid_mapping_message *online_map_message);

void
carmen_grid_mapping_subscribe_message(carmen_grid_mapping_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_grid_mapping_moving_objects_raw_map_subscribe_message(carmen_grid_mapping_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_grid_mapping_unsubscribe_message(carmen_handler_t handler);

void
carmen_grid_mapping_moving_objects_raw_map_unsubscribe_message(carmen_handler_t handler);

void
carmen_grid_mapping_define_messages();

void
carmen_grid_mapping_publish_message(carmen_map_t *carmen_map, double timestamp);

void
carmen_grid_mapping_moving_objects_raw_map_publish_message(carmen_map_t *carmen_map, double timestamp);

#ifdef __cplusplus
}
#endif

#endif


// @}

