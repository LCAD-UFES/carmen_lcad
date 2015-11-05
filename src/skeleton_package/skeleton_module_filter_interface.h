#include <carmen/skeleton_module_filter_messages.h>

#ifdef __cplusplus
extern "C" {
#endif

void
carmen_skeleton_module_filter_subscribe_upkeymessage(carmen_skeleton_module_filter_upkeymessage_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_skeleton_module_filter_unsubscribe_upkeymessage(carmen_handler_t handler);

void
carmen_skeleton_module_filter_define_messages();

#ifdef __cplusplus
}
#endif


// @}

