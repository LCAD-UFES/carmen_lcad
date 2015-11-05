#include <carmen/carmen.h>
#include <carmen/neural_localizer_messages.h>

void
carmen_neural_localizer_subscribe_place_command_message(carmen_neural_localizer_place_command_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_NEURAL_LOCALIZER_PLACE_COMMAND_NAME,
  												 CARMEN_NEURAL_LOCALIZER_PLACE_COMMAND_FMT,
                           message, sizeof(carmen_neural_localizer_place_command_message),
                           handler, subscribe_how);
}


void
carmen_neural_localizer_unsubscribe_place_command_message(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_NEURAL_LOCALIZER_PLACE_COMMAND_NAME, handler);
}


void
carmen_neural_localizer_define_messages()
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(CARMEN_NEURAL_LOCALIZER_PLACE_COMMAND_NAME, IPC_VARIABLE_LENGTH,
  		CARMEN_NEURAL_LOCALIZER_PLACE_COMMAND_FMT);
  carmen_test_ipc_exit(err, "Could not define", CARMEN_NEURAL_LOCALIZER_PLACE_COMMAND_NAME);
}
