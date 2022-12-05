#include "aruco_interface.h"


char*
aruco_set_message_name(int detector_id)
{
	char *message_name = (char*) malloc (64 * sizeof(char));
	sprintf(message_name, "carmen_aruco_message%d", detector_id);
	return message_name;
}

IPC_RETURN_TYPE 
aruco_define_message(int detector_id)
{
  IPC_RETURN_TYPE err;

  char *message_name = aruco_set_message_name(detector_id);
  err = IPC_defineMsg(message_name, IPC_VARIABLE_LENGTH, CARMEN_ARUCO_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define", message_name);
  free(message_name);
  return err;
}

void
aruco_subscribe_message(int detector_id, carmen_aruco_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
    char *message_name = aruco_set_message_name(detector_id);
	carmen_subscribe_message(message_name, (char*) CARMEN_ARUCO_MESSAGE_FMT, message, sizeof(carmen_aruco_message), handler, subscribe_how);
	free(message_name);
}

IPC_RETURN_TYPE
aruco_publish_message(int detector_id, const carmen_aruco_message *message)
{
    IPC_RETURN_TYPE err;

	char *message_name = aruco_set_message_name(detector_id);

	err = IPC_publishData(message_name, (carmen_aruco_message *)message);
	carmen_test_ipc_exit(err, "Could not publish", message_name);
	free(message_name);

	return err;
}

void
aruco_unsubscribe_message(int detector_id, carmen_handler_t handler)
{
    char *message_name = aruco_set_message_name(detector_id);
	carmen_unsubscribe_message(message_name, handler);
	free(message_name);
}
