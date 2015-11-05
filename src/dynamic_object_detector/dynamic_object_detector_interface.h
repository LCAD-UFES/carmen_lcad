#include <carmen/dynamic_object_detector_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_dynamic_object_detector_subscribe_clustered_objects_message(carmen_dynamic_object_detector_clustered_objects_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_dynamic_object_detector_unsubscribe_clustered_objects_message(carmen_handler_t handler);

void
carmen_dynamic_object_detector_define_messages();

#ifdef __cplusplus
}
#endif


// @}

