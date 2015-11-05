#include <carmen/carmen.h>
#include <carmen/skeleton_module_filter_messages.h>

void
carmen_skeleton_module_filter_subscribe_upkeymessage(carmen_skeleton_module_filter_upkeymessage_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_SKELETON_MODULE_FILTER_UPKEYMESSAGE_NAME, 
                           CARMEN_SKELETON_MODULE_FILTER_UPKEYMESSAGE_FMT,
                           message, sizeof(carmen_skeleton_module_filter_upkeymessage_message), 
			   handler, subscribe_how);
}


void
carmen_skeleton_module_filter_unsubscribe_upkeymessage(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_SKELETON_MODULE_FILTER_UPKEYMESSAGE_NAME, handler);
}


void
carmen_skeleton_module_filter_define_messages()
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_SKELETON_MODULE_FILTER_UPKEYMESSAGE_NAME, IPC_VARIABLE_LENGTH, 
	CARMEN_SKELETON_MODULE_FILTER_UPKEYMESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_SKELETON_MODULE_FILTER_UPKEYMESSAGE_NAME); 
}

