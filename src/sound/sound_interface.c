#include <carmen/carmen.h>
#include <carmen/sound_interface.h>

void
carmen_microphone_subscribe_message(carmen_microphone_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message (
  	CARMEN_MICROPHONE_MESSAGE_NAME,
  	CARMEN_MICROPHONE_MESSAGE_FMT,
  	message,
  	sizeof(carmen_microphone_message),
  	handler,
  	subscribe_how);
}


void
carmen_microphone_unsubscribe_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_MICROPHONE_MESSAGE_NAME, handler);
}


void
carmen_microphone_define_messages()
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_MICROPHONE_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_MICROPHONE_MESSAGE_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_MICROPHONE_MESSAGE_NAME);
}

