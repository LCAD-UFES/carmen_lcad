#include <prob_map.h>
#include "obstacle_distance_mapper_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

void carmen_obstacle_distance_mapper_subscribe_message(carmen_obstacle_distance_mapper_map_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how);

void carmen_obstacle_distance_mapper_publish_distance_map_message(carmen_prob_models_distance_map *distance_map,
																double timestamp);

#ifdef __cplusplus
}
#endif


// @}

