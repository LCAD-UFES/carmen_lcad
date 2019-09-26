#include "joystick_interface.h"

void
carmen_joystick_subscribe_status_message(carmen_joystick_status_message *message, carmen_handler_t handler, carmen_subscribe_t subscribe_how)
{
	carmen_subscribe_message(CARMEN_JOYSTICK_STATUS_MESSAGE_NAME,
			CARMEN_JOYSTICK_STATUS_MESSAGE_FMT,
			message, sizeof(carmen_joystick_status_message),
			handler, subscribe_how);
}


void
carmen_joystick_unsubscribe_status_message(carmen_handler_t handler)
{
	carmen_unsubscribe_message(CARMEN_JOYSTICK_STATUS_MESSAGE_NAME, handler);
}


void
carmen_joystick_define_messages()
{
	IPC_RETURN_TYPE err;

	err = IPC_defineMsg(CARMEN_JOYSTICK_STATUS_MESSAGE_NAME, IPC_VARIABLE_LENGTH,
			CARMEN_JOYSTICK_STATUS_MESSAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", CARMEN_JOYSTICK_STATUS_MESSAGE_NAME);
}
