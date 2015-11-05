#include <carmen/carmen.h>
#include <carmen/neural_global_localizer_messages.h>
#include <carmen/neural_global_localizer_interface.h>

void
carmen_neural_global_localizer_subscribe_globalpos_message(carmen_neural_global_localizer_globalpos_message *globalpos,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_NEURAL_GLOBAL_LOCALIZER_GLOBALPOS_MESSAGE_NAME,
			CARMEN_NEURAL_GLOBAL_LOCALIZER_GLOBALPOS_MESSAGE_FMT,
			globalpos, sizeof(carmen_neural_global_localizer_globalpos_message),
			handler, subscribe_how);
}

void
carmen_neural_global_localizer_unsubscribe_globalpos_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_NEURAL_GLOBAL_LOCALIZER_GLOBALPOS_MESSAGE_NAME, handler);
}

void
carmen_neural_global_localizer_define_messages()
{
	  IPC_RETURN_TYPE err;

	  err = IPC_defineMsg(CARMEN_NEURAL_GLOBAL_LOCALIZER_GLOBALPOS_MESSAGE_NAME, IPC_VARIABLE_LENGTH, CARMEN_NEURAL_GLOBAL_LOCALIZER_GLOBALPOS_MESSAGE_FMT);
	  carmen_test_ipc_exit(err, "Could not define", CARMEN_NEURAL_GLOBAL_LOCALIZER_GLOBALPOS_MESSAGE_NAME);
}
