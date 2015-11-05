#include <carmen/skeleton_module_sensor_messages.h>

#ifdef __cplusplus
extern "C" 
{
#endif

void
carmen_skeleton_module_sensor_subscribe_keymessage(carmen_skeleton_module_sensor_keymessage_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_skeleton_module_sensor_unsubscribe_keymessage(carmen_handler_t handler);

void
carmen_skeleton_module_sensor_define_messages();

#ifdef __cplusplus
}
#endif


// @}

