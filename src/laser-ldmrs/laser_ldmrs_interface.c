#include <carmen/carmen.h>
#include <carmen/skeleton_module_sensor_messages.h>

void
carmen_skeleton_module_sensor_subscribe_keymessage(carmen_skeleton_module_sensor_keymessage_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_SKELETON_MODULE_SENSOR_KEYMESSAGE_NAME, 
                           CARMEN_SKELETON_MODULE_SENSOR_KEYMESSAGE_FMT,
                           message, sizeof(carmen_skeleton_module_sensor_keymessage_message), 
			   handler, subscribe_how);
}


void
carmen_skeleton_module_sensor_unsubscribe_keymessage(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_SKELETON_MODULE_SENSOR_KEYMESSAGE_NAME, handler);
}


void
carmen_skeleton_module_sensor_define_messages()
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_SKELETON_MODULE_SENSOR_KEYMESSAGE_NAME, IPC_VARIABLE_LENGTH, 
	CARMEN_SKELETON_MODULE_SENSOR_KEYMESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_SKELETON_MODULE_SENSOR_KEYMESSAGE_NAME); 
}

