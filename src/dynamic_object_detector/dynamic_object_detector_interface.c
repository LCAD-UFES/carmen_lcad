#include <carmen/carmen.h>
#include <carmen/dynamic_object_detector_messages.h>

void
carmen_dynamic_object_detector_subscribe_clustered_objects_message(carmen_dynamic_object_detector_clustered_objects_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_DYNAMIC_OBJECT_DETECTOR_CLUSTERED_OBJECTS_NAME,
  												 CARMEN_DYNAMIC_OBJECT_DETECTOR_CLUSTERED_OBJECTS_FMT,
                           message, sizeof(carmen_dynamic_object_detector_clustered_objects_message),
													 handler, subscribe_how);
}


void
carmen_dynamic_object_detector_unsubscribe_clustered_objects_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_DYNAMIC_OBJECT_DETECTOR_CLUSTERED_OBJECTS_NAME, handler);
}


void
carmen_dynamic_object_detector_define_messages()
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_DYNAMIC_OBJECT_DETECTOR_CLUSTERED_OBJECTS_NAME, IPC_VARIABLE_LENGTH,
  		CARMEN_DYNAMIC_OBJECT_DETECTOR_CLUSTERED_OBJECTS_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_DYNAMIC_OBJECT_DETECTOR_CLUSTERED_OBJECTS_NAME);
}

