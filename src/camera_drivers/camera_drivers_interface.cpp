#include "camera_drivers_interface.h"


char*
camera_drivers_set_message_name(int camera_id)
{
	char *message_name = (char*) malloc (64 * sizeof(char));
	sprintf(message_name, "camera%d", camera_id);
	return message_name;
}


IPC_RETURN_TYPE
camera_drivers_define_message(int camera_id)
{
  IPC_RETURN_TYPE err;

  char *message_name = camera_drivers_set_message_name(camera_id);
  err = IPC_defineMsg(message_name, IPC_VARIABLE_LENGTH, CAMERA_FMT);
  carmen_test_ipc_exit(err, "Could not define", message_name);
  free(message_name);
  return err;
}


void
camera_drivers_subscribe_message(int camera_id, camera_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	char *message_name = camera_drivers_set_message_name(camera_id);
	carmen_subscribe_message(message_name, (char*) CAMERA_FMT, message, sizeof(camera_message), handler, subscribe_how);
	free(message_name);
	//printf("Subscribe to Camera %d Image Messages!\n\n", camera_id);
}


IPC_RETURN_TYPE
camera_drivers_publish_message(int camera_id, const camera_message *message)
{
	IPC_RETURN_TYPE err;

	char *message_name = camera_drivers_set_message_name(camera_id);

	err = IPC_publishData(message_name, (camera_message *)message);
	carmen_test_ipc_exit(err, "Could not publish", message_name);
	free(message_name);

	return err;
}


void
camera_drivers_unsubscribe_message(int camera_id, carmen_handler_t handler)
{
	char *message_name = camera_drivers_set_message_name(camera_id);
	carmen_unsubscribe_message(message_name, handler);
	free(message_name);
}