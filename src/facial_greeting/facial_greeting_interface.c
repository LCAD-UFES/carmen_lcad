#include <carmen/carmen.h>
#include <carmen/facial_greeting_interface.h>

void
carmen_facial_greeting_subscribe_default_message(carmen_facial_greeting_default_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_FACIAL_GREETING_DEFAULT_MESSAGE_NAME,
			CARMEN_FACIAL_GREETING_DEFAULT_MESSAGE_FMT,
			message, sizeof(carmen_facial_greeting_default_message),
			handler, subscribe_how);
}


void
carmen_facial_greeting_unsubscribe_default_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_FACIAL_GREETING_DEFAULT_MESSAGE_NAME, handler);
}


void
carmen_facial_greeting_define_default_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_FACIAL_GREETING_DEFAULT_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_FACIAL_GREETING_DEFAULT_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_FACIAL_GREETING_DEFAULT_MESSAGE_NAME);
}

void
carmen_facial_greeting_subscribe_face_recog_message(carmen_face_recog_message *message,
		carmen_handler_t handler,
		carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_FACE_RECOG_MESSAGE_NAME,
			CARMEN_FACE_RECOG_MESSAGE_FMT,
			message, sizeof(carmen_face_recog_message),
			handler, subscribe_how);
}


void
carmen_facial_greeting_unsubscribe_face_recog_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_FACE_RECOG_MESSAGE_NAME, handler);
}

void
carmen_facial_greeting_define_face_recog_message()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_FACE_RECOG_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_FACE_RECOG_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_FACE_RECOG_MESSAGE_NAME);
}

