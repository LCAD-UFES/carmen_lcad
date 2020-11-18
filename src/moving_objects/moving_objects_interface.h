#include "moving_objects_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

/*************
 * grid map *
 ************/
void
carmen_moving_objects_map_subscribe_message(carmen_moving_objects_map_message *message,
	       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_moving_objects_map_unsubscribe_message(carmen_handler_t handler);

void
carmen_moving_objects_map_publish_message(carmen_moving_objects_map_message *moving_objects_map_message);

void
carmen_moving_objects_map_define_messages();

/*************
 * laser *
 ************/

void
carmen_moving_objects_point_clouds_subscribe_message(carmen_moving_objects_point_clouds_message *message,
	       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_moving_objects_point_clouds_unsubscribe_message(carmen_handler_t handler);

void
carmen_moving_objects_point_clouds_publish_message(carmen_moving_objects_point_clouds_message *carmen_moving_objects_point_clouds_message);

void
carmen_moving_objects_point_clouds_define_messages();

IPC_RETURN_TYPE
carmen_moving_objects_point_clouds_define_messages_generic(int message_id);

void
carmen_moving_objects_point_clouds_subscribe_message_generic(int message_id, carmen_moving_objects_point_clouds_message *message, carmen_handler_t handler,
	carmen_subscribe_t subscribe_how);

IPC_RETURN_TYPE
carmen_moving_objects_point_clouds_publish_message_generic(int message_id, carmen_moving_objects_point_clouds_message *msg);

void
carmen_moving_objects_point_clouds_unsubscribe_message_generic(int message_id, carmen_handler_t handler);


#ifdef __cplusplus
}
#endif


