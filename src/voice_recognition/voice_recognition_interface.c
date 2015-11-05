#include <carmen/carmen.h>
#include <carmen/voice_recognition_messages.h>

void
carmen_voice_recognition_subscribe_message (carmen_voice_recognition_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_VOICE_RECOGNITION_MESSAGE_NAME,
			CARMEN_VOICE_RECOGNITION_MESSAGE_FMT,
			message, sizeof(carmen_voice_recognition_message),
			handler, subscribe_how);
}


void
carmen_voice_recognition_unsubscribe_message (carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_VOICE_RECOGNITION_MESSAGE_NAME, handler);
}


void
carmen_voice_recognition_define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_VOICE_RECOGNITION_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_VOICE_RECOGNITION_MESSAGE_FMT);

	carmen_test_ipc_exit(err, "Could not define", CARMEN_VOICE_RECOGNITION_MESSAGE_NAME);
}

